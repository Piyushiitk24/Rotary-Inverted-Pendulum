"""
Discrete-Time Controller for Furuta Pendulum
=============================================
This implements the EXACT same controller as the firmware (main.cpp),
including all the quirks, filters, and discretization.

This allows direct comparison between simulation and hardware.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional
from furuta_model import FurutaParams


@dataclass
class ControllerParams:
    """
    Controller parameters - matching firmware exactly.
    """
    # Inner loop (pendulum stabilization) - in step units!
    ACC_KP: float = 742.0      # steps/s² per degree of alpha
    ACC_KD: float = 54.6       # steps/s² per (deg/s) of alpha_dot
    ACC_KI: float = 0.0        # steps/s² per (deg·s) of alpha integral
    
    # Outer loop (arm centering)
    KTHETA: float = 15.0       # steps/s² per degree of theta
    KTHETADOT: float = 60.0    # steps/s² per (deg/s) of theta_dot
    outer_loop_enabled: bool = True
    ALPHA_GATE_DEG: float = 3.0   # Only apply outer loop when |α| < this
    OUTER_MAX_ACC: float = 1500.0  # Clamp outer loop contribution
    
    # Filters
    D_FILT_ALPHA: float = 0.35   # Alpha derivative low-pass (α = 0.35)
    D_FILT_THETA: float = 0.20   # Theta derivative low-pass
    
    # Limits
    MAX_ACC_STEPS: float = 20000.0   # Max acceleration command
    MAX_SPEED_STEPS: float = 25000.0  # Max velocity command
    ALPHADOT_CLAMP: float = 300.0    # deg/s
    THETADOT_CLAMP: float = 300.0    # deg/s
    ALPHA_INT_CLAMP: float = 3.0     # deg·s
    
    # Deadband
    ALPHA_DEADBAND_DEG: float = 0.3
    
    # Engage ramp
    RAMP_TIME_MS: float = 50.0  # Time to ramp up to full power
    
    # Velocity leak (prevents drift)
    VEL_LEAK: float = 2.0  # 1/s
    
    # Control loop rate
    LOOP_HZ: float = 200.0
    
    # Position limits
    LIM_MOTOR_DEG: float = 80.0
    LIM_PEND_DEG: float = 30.0


@dataclass 
class ControllerState:
    """
    Internal state of the controller (persists between calls).
    """
    # Filtered derivatives
    alpha_dot_filt: float = 0.0
    theta_dot_filt: float = 0.0
    
    # Previous values for differentiation
    last_alpha_raw: float = 0.0
    last_theta: float = 0.0
    
    # Integrators
    alpha_int: float = 0.0
    
    # Commanded trajectory (internal state)
    theta_dot_cmd: float = 0.0   # steps/s
    theta_cmd: float = 0.0       # steps
    
    # Timing
    engage_time_ms: float = 0.0
    time_ms: float = 0.0
    
    # Mode
    is_engaged: bool = False
    is_fallen: bool = False


class FurutaController:
    """
    Discrete-time controller matching firmware exactly.
    
    Input: alpha (deg), theta (deg) - sensor readings
    Output: acceleration command (steps/s²) -> position command (steps)
    """
    
    def __init__(self, 
                 ctrl_params: Optional[ControllerParams] = None,
                 model_params: Optional[FurutaParams] = None):
        self.cp = ctrl_params if ctrl_params else ControllerParams()
        self.mp = model_params if model_params else FurutaParams()
        self.state = ControllerState()
        self.dt = 1.0 / self.cp.LOOP_HZ
        
    def reset(self):
        """Reset controller state."""
        self.state = ControllerState()
        
    def engage(self, current_time_ms: float = 0.0):
        """Engage the controller (start balancing)."""
        self.state.is_engaged = True
        self.state.is_fallen = False
        self.state.engage_time_ms = current_time_ms
        self.state.alpha_int = 0.0
        self.state.theta_dot_cmd = 0.0
        self.state.theta_cmd = 0.0
        
    def disengage(self):
        """Disengage the controller."""
        self.state.is_engaged = False
        
    def step(self, alpha_deg: float, theta_deg: float, 
             time_ms: float) -> Tuple[float, dict]:
        """
        Execute one control step.
        
        Args:
            alpha_deg: Pendulum angle in degrees (0 = upright)
            theta_deg: Arm angle in degrees (0 = center)
            time_ms: Current time in milliseconds
            
        Returns:
            position_cmd_steps: Position command in steps
            debug_info: Dictionary with internal values for logging
        """
        cp = self.cp
        s = self.state
        s.time_ms = time_ms
        
        # Check limits
        if abs(theta_deg) > cp.LIM_MOTOR_DEG or abs(alpha_deg) > cp.LIM_PEND_DEG:
            s.is_fallen = True
            s.is_engaged = False
            return 0.0, {"fallen": True, "reason": "limit_exceeded"}
        
        if not s.is_engaged:
            # Just update filters passively
            self._update_filters_idle(alpha_deg, theta_deg)
            return 0.0, {"engaged": False}
        
        # === DERIVATIVE COMPUTATION (from raw angle, no deadband) ===
        d_alpha = alpha_deg - s.last_alpha_raw
        # Wrap difference
        while d_alpha > 180: d_alpha -= 360
        while d_alpha < -180: d_alpha += 360
        
        alpha_dot = d_alpha / self.dt
        s.last_alpha_raw = alpha_deg
        
        # Low-pass filter on alpha_dot
        s.alpha_dot_filt += cp.D_FILT_ALPHA * (alpha_dot - s.alpha_dot_filt)
        s.alpha_dot_filt = np.clip(s.alpha_dot_filt, -cp.ALPHADOT_CLAMP, cp.ALPHADOT_CLAMP)
        
        # Theta derivative
        theta_dot = (theta_deg - s.last_theta) / self.dt
        s.last_theta = theta_deg
        s.theta_dot_filt += cp.D_FILT_THETA * (theta_dot - s.theta_dot_filt)
        s.theta_dot_filt = np.clip(s.theta_dot_filt, -cp.THETADOT_CLAMP, cp.THETADOT_CLAMP)
        
        # === DEADBAND for P and I terms ===
        alpha_ctrl = alpha_deg
        if abs(alpha_ctrl) < cp.ALPHA_DEADBAND_DEG:
            alpha_ctrl = 0.0
            
        # === INTEGRATOR ===
        s.alpha_int += alpha_ctrl * self.dt
        s.alpha_int = np.clip(s.alpha_int, -cp.ALPHA_INT_CLAMP, cp.ALPHA_INT_CLAMP)
        
        # === INNER LOOP: PD(+I) on pendulum angle ===
        acc_cmd = (cp.ACC_KP * alpha_ctrl + 
                   cp.ACC_KD * s.alpha_dot_filt + 
                   cp.ACC_KI * s.alpha_int)
        
        # === ENGAGE RAMP ===
        ramp_elapsed = time_ms - s.engage_time_ms
        ramp = min(1.0, ramp_elapsed / cp.RAMP_TIME_MS)
        acc_cmd *= ramp
        
        # === OUTER LOOP: Position correction ===
        if (cp.outer_loop_enabled and 
            abs(alpha_deg) < cp.ALPHA_GATE_DEG and 
            abs(s.alpha_dot_filt) < 50.0):
            
            outer = -(cp.KTHETA * theta_deg + cp.KTHETADOT * s.theta_dot_filt)
            outer = np.clip(outer, -cp.OUTER_MAX_ACC, cp.OUTER_MAX_ACC)
            acc_cmd += outer
        
        # === SATURATION ===
        saturated = False
        if acc_cmd > cp.MAX_ACC_STEPS:
            acc_cmd = cp.MAX_ACC_STEPS
            saturated = True
        elif acc_cmd < -cp.MAX_ACC_STEPS:
            acc_cmd = -cp.MAX_ACC_STEPS
            saturated = True
            
        # Anti-windup
        if saturated:
            s.alpha_int -= alpha_ctrl * self.dt
            s.alpha_int = np.clip(s.alpha_int, -cp.ALPHA_INT_CLAMP, cp.ALPHA_INT_CLAMP)
        
        # === VELOCITY LEAK ===
        if cp.VEL_LEAK > 0:
            s.theta_dot_cmd *= (1.0 - cp.VEL_LEAK * self.dt)
        
        # === INTEGRATE TO VELOCITY AND POSITION ===
        if not saturated:
            s.theta_dot_cmd += acc_cmd * self.dt
            s.theta_dot_cmd = np.clip(s.theta_dot_cmd, -cp.MAX_SPEED_STEPS, cp.MAX_SPEED_STEPS)
            s.theta_cmd += s.theta_dot_cmd * self.dt
        
        # Position limits
        limit_steps = cp.LIM_MOTOR_DEG * self.mp.steps_per_deg
        if s.theta_cmd >= limit_steps:
            s.theta_cmd = limit_steps
            if s.theta_dot_cmd > 0:
                s.theta_dot_cmd = 0
        elif s.theta_cmd <= -limit_steps:
            s.theta_cmd = -limit_steps
            if s.theta_dot_cmd < 0:
                s.theta_dot_cmd = 0
        
        # === BUILD DEBUG INFO (matching firmware log format) ===
        debug = {
            "t_ms": int(time_ms),
            "alpha_raw_100": int(alpha_deg * 100),
            "alpha_dot_100": int(s.alpha_dot_filt * 100),
            "theta_100": int(theta_deg * 100),
            "theta_dot_100": int(s.theta_dot_filt * 100),
            "acc_cmd": int(acc_cmd),
            "vel_cmd": int(s.theta_dot_cmd),
            "pos_cmd": int(s.theta_cmd),
            "clamped": 1 if abs(s.theta_cmd) >= limit_steps - 0.1 else 0,
            "saturated": saturated,
            "ramp": ramp
        }
        
        return s.theta_cmd, debug
    
    def _update_filters_idle(self, alpha_deg: float, theta_deg: float):
        """Update filters while idle (keeps them warm)."""
        cp = self.cp
        s = self.state
        
        d_alpha = alpha_deg - s.last_alpha_raw
        while d_alpha > 180: d_alpha -= 360
        while d_alpha < -180: d_alpha += 360
        
        alpha_dot = d_alpha / self.dt
        s.last_alpha_raw = alpha_deg
        s.alpha_dot_filt += cp.D_FILT_ALPHA * (alpha_dot - s.alpha_dot_filt)
        s.alpha_dot_filt = np.clip(s.alpha_dot_filt, -cp.ALPHADOT_CLAMP, cp.ALPHADOT_CLAMP)
        
        theta_dot = (theta_deg - s.last_theta) / self.dt
        s.last_theta = theta_deg
        s.theta_dot_filt += cp.D_FILT_THETA * (theta_dot - s.theta_dot_filt)
        s.theta_dot_filt = np.clip(s.theta_dot_filt, -cp.THETADOT_CLAMP, cp.THETADOT_CLAMP)
    
    def steps_to_torque(self, acc_steps: float) -> float:
        """
        Convert acceleration in steps/s² to torque in N·m.
        
        This is the inverse of what the motor does:
        acc_rad = acc_steps / steps_per_rad
        tau = J_eff * acc_rad
        """
        acc_rad = acc_steps / self.mp.steps_per_rad
        # Use effective inertia (arm + pendulum at tip)
        tau = self.mp.J_hat_0 * acc_rad
        return tau
    
    def pos_to_theta_rad(self, pos_steps: float) -> float:
        """Convert position in steps to arm angle in radians."""
        return pos_steps / self.mp.steps_per_rad


def test_controller():
    """Basic controller test."""
    ctrl = FurutaController()
    
    print("=" * 60)
    print("CONTROLLER TEST")
    print("=" * 60)
    
    # Simulate a small perturbation
    ctrl.engage(0.0)
    
    alpha_deg = 2.0  # 2 degree tilt
    theta_deg = 0.0
    
    for i in range(10):
        t_ms = i * 5  # 200 Hz = 5ms per step
        pos, debug = ctrl.step(alpha_deg, theta_deg, t_ms)
        print(f"t={t_ms:3d}ms: α={alpha_deg:.1f}°, acc={debug['acc_cmd']:6d}, "
              f"vel={debug['vel_cmd']:6d}, pos={debug['pos_cmd']:6d}")
        
        # Simulate pendulum responding (simplified)
        alpha_deg *= 0.95  # Assume controller is helping
        theta_deg = pos / ctrl.mp.steps_per_deg


if __name__ == "__main__":
    test_controller()
