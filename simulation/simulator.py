"""
Main Simulation Runner for Furuta Pendulum
===========================================
Combines the nonlinear dynamics model with the discrete controller
to create a complete closed-loop simulation.

Features:
- Exact firmware controller behavior
- Realistic sensor noise (optional)
- Telemetry logging in same format as hardware
- Easy parameter sweeps for gain tuning
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Callable
import time

from furuta_model import FurutaParams, FurutaDynamics
from controller import ControllerParams, FurutaController


@dataclass
class SimulationConfig:
    """Configuration for simulation."""
    # Timing
    duration_s: float = 5.0          # Total simulation time
    dt_physics: float = 0.0001       # Physics integration timestep (100kHz)
    dt_control: float = 0.005        # Controller timestep (200Hz, matching firmware)
    
    # Initial conditions
    theta_0_deg: float = 0.0         # Initial arm angle
    alpha_0_deg: float = 2.0         # Initial pendulum angle (small perturbation)
    theta_dot_0: float = 0.0         # Initial arm velocity (deg/s)
    alpha_dot_0: float = 0.0         # Initial pendulum velocity (deg/s)
    
    # Noise (optional, for realistic testing)
    sensor_noise_alpha_deg: float = 0.0  # Std dev of alpha sensor noise
    sensor_noise_theta_deg: float = 0.0  # Std dev of theta sensor noise
    
    # Disturbances
    disturbance_time_s: float = -1   # Time to apply disturbance (-1 = none)
    disturbance_alpha_deg: float = 0.0  # Impulse disturbance to pendulum
    
    # Engage timing
    engage_delay_s: float = 0.1      # Time before controller engages
    
    # Logging
    log_rate_hz: float = 50.0        # Log rate (matching firmware)


@dataclass
class SimulationResult:
    """Results from simulation."""
    # Time series
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # States (in degrees for easy comparison)
    theta: np.ndarray = field(default_factory=lambda: np.array([]))
    alpha: np.ndarray = field(default_factory=lambda: np.array([]))
    theta_dot: np.ndarray = field(default_factory=lambda: np.array([]))
    alpha_dot: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # Control signals
    acc_cmd: np.ndarray = field(default_factory=lambda: np.array([]))
    vel_cmd: np.ndarray = field(default_factory=lambda: np.array([]))
    pos_cmd: np.ndarray = field(default_factory=lambda: np.array([]))
    torque: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # Metadata
    success: bool = False
    fall_time: float = -1
    max_alpha: float = 0
    max_theta: float = 0
    
    def to_csv_string(self) -> str:
        """Export as CSV in same format as firmware logs."""
        lines = ["t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posCmd,clamped"]
        for i in range(len(self.t)):
            t_ms = int(self.t[i] * 1000)
            alpha100 = int(self.alpha[i] * 100)
            alphaDot100 = int(self.alpha_dot[i] * 100)
            theta100 = int(self.theta[i] * 100)
            thetaDot100 = int(self.theta_dot[i] * 100)
            acc = int(self.acc_cmd[i]) if i < len(self.acc_cmd) else 0
            vel = int(self.vel_cmd[i]) if i < len(self.vel_cmd) else 0
            pos = int(self.pos_cmd[i]) if i < len(self.pos_cmd) else 0
            clamped = 0  # TODO: compute from pos
            lines.append(f"{t_ms},{alpha100},{alphaDot100},{theta100},{thetaDot100},{acc},{vel},{pos},{clamped}")
        return "\n".join(lines)
    
    def save_csv(self, filename: str):
        """Save to CSV file."""
        with open(filename, 'w') as f:
            f.write(self.to_csv_string())


class FurutaSimulator:
    """
    Full closed-loop simulator for the Furuta pendulum.
    """
    
    def __init__(self, 
                 model_params: Optional[FurutaParams] = None,
                 ctrl_params: Optional[ControllerParams] = None,
                 config: Optional[SimulationConfig] = None):
        
        self.model_params = model_params if model_params else FurutaParams()
        self.ctrl_params = ctrl_params if ctrl_params else ControllerParams()
        self.config = config if config else SimulationConfig()
        
        self.dynamics = FurutaDynamics(self.model_params)
        self.controller = FurutaController(self.ctrl_params, self.model_params)
        
    def run(self, verbose: bool = True) -> SimulationResult:
        """
        Run the simulation.
        
        Returns:
            SimulationResult with all time series data
        """
        cfg = self.config
        mp = self.model_params
        
        # Initialize result
        result = SimulationResult()
        
        # Convert initial conditions to radians
        state = np.array([
            np.radians(cfg.theta_0_deg),
            np.radians(cfg.alpha_0_deg),
            np.radians(cfg.theta_dot_0),
            np.radians(cfg.alpha_dot_0)
        ])
        
        # Storage for results
        t_list = []
        theta_list = []
        alpha_list = []
        theta_dot_list = []
        alpha_dot_list = []
        acc_list = []
        vel_list = []
        pos_list = []
        torque_list = []
        
        # Simulation state
        current_torque = 0.0
        t = 0.0
        last_control_t = -cfg.dt_control  # Force immediate control update
        last_log_t = 0.0
        log_period = 1.0 / cfg.log_rate_hz
        
        # Reset controller
        self.controller.reset()
        engaged = False
        
        if verbose:
            print(f"Starting simulation: {cfg.duration_s}s, α₀={cfg.alpha_0_deg}°")
            print(f"Controller: KP={self.ctrl_params.ACC_KP}, KD={self.ctrl_params.ACC_KD}")
            print("-" * 60)
        
        # Main simulation loop
        n_steps = int(cfg.duration_s / cfg.dt_physics)
        start_time = time.time()
        
        for step in range(n_steps):
            t = step * cfg.dt_physics
            
            # Get current angles in degrees (with optional noise)
            theta_deg = np.degrees(state[0])
            alpha_deg = np.degrees(state[1])
            theta_dot_deg = np.degrees(state[2])
            alpha_dot_deg = np.degrees(state[3])
            
            if cfg.sensor_noise_alpha_deg > 0:
                alpha_deg += np.random.normal(0, cfg.sensor_noise_alpha_deg)
            if cfg.sensor_noise_theta_deg > 0:
                theta_deg += np.random.normal(0, cfg.sensor_noise_theta_deg)
            
            # Apply disturbance if scheduled
            if (cfg.disturbance_time_s > 0 and 
                abs(t - cfg.disturbance_time_s) < cfg.dt_physics):
                state[1] += np.radians(cfg.disturbance_alpha_deg)
                if verbose:
                    print(f"  Disturbance applied at t={t:.3f}s: Δα={cfg.disturbance_alpha_deg}°")
            
            # Engage controller after delay
            if not engaged and t >= cfg.engage_delay_s:
                self.controller.engage(t * 1000)
                engaged = True
                if verbose:
                    print(f"  Controller engaged at t={t:.3f}s")
            
            # Control update at discrete rate
            if t - last_control_t >= cfg.dt_control:
                last_control_t = t
                
                pos_cmd, debug = self.controller.step(
                    alpha_deg, theta_deg, t * 1000
                )
                
                # Check for fall
                if debug.get("fallen", False):
                    if verbose:
                        print(f"  FALLEN at t={t:.3f}s - {debug.get('reason', 'unknown')}")
                    result.fall_time = t
                    break
                
                # Convert controller output to torque
                if engaged and not debug.get("fallen", False):
                    acc_cmd = debug.get("acc_cmd", 0)
                    current_torque = self.controller.steps_to_torque(acc_cmd)
                    
                    # Store control signals at log rate
                    if t - last_log_t >= log_period:
                        last_log_t = t
                        t_list.append(t)
                        theta_list.append(theta_deg)
                        alpha_list.append(alpha_deg)
                        theta_dot_list.append(theta_dot_deg)
                        alpha_dot_list.append(alpha_dot_deg)
                        acc_list.append(debug.get("acc_cmd", 0))
                        vel_list.append(debug.get("vel_cmd", 0))
                        pos_list.append(debug.get("pos_cmd", 0))
                        torque_list.append(current_torque)
                else:
                    current_torque = 0.0
            
            # Physics integration step (Euler for simplicity, RK4 for accuracy)
            def tau_func(t_inner, state_inner):
                return current_torque
            
            # Simple Euler integration (fast)
            dstate = self.dynamics.state_derivative(t, state, tau_func)
            state = state + dstate * cfg.dt_physics
            
            # Track extremes
            result.max_alpha = max(result.max_alpha, abs(alpha_deg))
            result.max_theta = max(result.max_theta, abs(theta_deg))
        
        # Finalize result
        result.t = np.array(t_list)
        result.theta = np.array(theta_list)
        result.alpha = np.array(alpha_list)
        result.theta_dot = np.array(theta_dot_list)
        result.alpha_dot = np.array(alpha_dot_list)
        result.acc_cmd = np.array(acc_list)
        result.vel_cmd = np.array(vel_list)
        result.pos_cmd = np.array(pos_list)
        result.torque = np.array(torque_list)
        
        result.success = result.fall_time < 0
        
        elapsed = time.time() - start_time
        if verbose:
            print("-" * 60)
            if result.success:
                print(f"✓ Simulation completed: {cfg.duration_s}s in {elapsed:.2f}s real time")
                print(f"  Max |α| = {result.max_alpha:.2f}°, Max |θ| = {result.max_theta:.2f}°")
            else:
                print(f"✗ Pendulum fell at t={result.fall_time:.3f}s")
        
        return result


def run_gain_sweep(kp_values: List[float], kd_values: List[float],
                   alpha_0: float = 5.0, duration: float = 3.0) -> dict:
    """
    Sweep through gain combinations and test stability.
    
    Returns dict mapping (kp, kd) -> success
    """
    results = {}
    
    for kp in kp_values:
        for kd in kd_values:
            ctrl_params = ControllerParams(ACC_KP=kp, ACC_KD=kd)
            config = SimulationConfig(
                duration_s=duration,
                alpha_0_deg=alpha_0,
                engage_delay_s=0.05
            )
            
            sim = FurutaSimulator(ctrl_params=ctrl_params, config=config)
            result = sim.run(verbose=False)
            
            results[(kp, kd)] = {
                'success': result.success,
                'fall_time': result.fall_time,
                'max_alpha': result.max_alpha,
                'max_theta': result.max_theta
            }
            
            status = "✓" if result.success else f"✗ ({result.fall_time:.2f}s)"
            print(f"KP={kp:6.0f}, KD={kd:5.1f}: {status}")
    
    return results


def main():
    """Run a basic simulation test."""
    print("=" * 60)
    print("FURUTA PENDULUM SIMULATION")
    print("=" * 60)
    
    # Test with default parameters (matching hardware)
    config = SimulationConfig(
        duration_s=3.0,
        alpha_0_deg=3.0,      # Start 3° from upright
        engage_delay_s=0.05,  # Quick engage
    )
    
    sim = FurutaSimulator(config=config)
    result = sim.run(verbose=True)
    
    if result.success:
        print("\n✓ Controller successfully balanced the pendulum!")
        print(f"  Pendulum stayed within ±{result.max_alpha:.1f}°")
        print(f"  Arm stayed within ±{result.max_theta:.1f}°")
    else:
        print(f"\n✗ Pendulum fell at t={result.fall_time:.3f}s")
    
    return result


if __name__ == "__main__":
    result = main()
