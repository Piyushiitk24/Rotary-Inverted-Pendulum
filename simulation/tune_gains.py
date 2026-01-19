#!/usr/bin/env python3
"""
Interactive Gain Tuning Tool for the Furuta Pendulum.
Run this to test different gain combinations in simulation before hardware.

Usage:
    python3 tune_gains.py              # Use default gains
    python3 tune_gains.py 800 60 20 80  # Custom: KP KD KTHETA KTHETADOT
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from furuta_model import FurutaParams, FurutaDynamics


def simulate_with_gains(kp: float, kd: float, ktheta: float, kthetadot: float,
                        initial_alpha_deg: float = 3.0,
                        duration: float = 5.0,
                        outer_sign: float = +1.0) -> dict:
    """
    Run closed-loop simulation with given gains.
    
    Args:
        kp: Pendulum angle gain (ACC_KP)
        kd: Pendulum rate gain (ACC_KD)
        ktheta: Arm position gain (KTHETA)
        kthetadot: Arm velocity gain (KTHETADOT)
        initial_alpha_deg: Initial pendulum tilt (degrees)
        duration: Simulation duration (seconds)
        outer_sign: +1 or -1 for outer loop sign
        
    Returns:
        Dictionary with simulation results
    """
    model = FurutaDynamics()
    p = model.p
    
    # Control limits (match firmware)
    MAX_ACC = 200000
    MAX_VEL = 8000
    LIM_PEND = 35 * np.pi / 180
    LIM_MOTOR = 120 * np.pi / 180
    ALPHA_GATE_DEG = 20  # Outer loop gate
    
    # Simulation setup
    dt_ctrl = 0.005  # 200 Hz control loop
    dt_sim = 0.0001  # Physics timestep
    
    # Initial state
    state = np.array([0.0, initial_alpha_deg * np.pi / 180, 0.0, 0.0])
    vel_cmd = 0.0
    pos_cmd = 0.0
    
    # Storage
    times, states, torques, cmds = [], [], [], []
    
    t = 0
    ctrl_time = 0
    tau = 0
    fell = False
    fall_time = None
    
    while t < duration:
        # Controller update at 200 Hz
        if t >= ctrl_time:
            theta, alpha, theta_dot, alpha_dot = state
            
            # Check limits
            if abs(alpha) > LIM_PEND:
                fell = True
                if fall_time is None:
                    fall_time = t
                tau = 0
                vel_cmd = 0
            elif abs(theta) > LIM_MOTOR:
                tau = 0
                vel_cmd = 0
            else:
                # Convert to degrees
                alpha_deg = np.degrees(alpha)
                alpha_dot_deg = np.degrees(alpha_dot)
                theta_deg = np.degrees(theta)
                theta_dot_deg = np.degrees(theta_dot)
                
                # Inner loop
                inner = kp * alpha_deg + kd * alpha_dot_deg
                
                # Outer loop (with gate)
                if abs(alpha_deg) < ALPHA_GATE_DEG and abs(alpha_dot_deg) < 50:
                    outer = outer_sign * (ktheta * theta_deg + kthetadot * theta_dot_deg)
                else:
                    outer = 0
                
                # Total command
                acc_cmd = inner + outer
                acc_cmd = np.clip(acc_cmd, -MAX_ACC, MAX_ACC)
                
                # Integrate to velocity/position
                vel_cmd += acc_cmd * dt_ctrl
                vel_cmd = np.clip(vel_cmd, -MAX_VEL, MAX_VEL)
                pos_cmd += vel_cmd * dt_ctrl
                
                # Convert to torque
                tau = (acc_cmd / p.steps_per_rad) * p.J_hat_0
            
            ctrl_time += dt_ctrl
        
        # Store
        times.append(t)
        states.append(state.copy())
        torques.append(tau)
        cmds.append([vel_cmd, pos_cmd])
        
        # Integrate physics
        theta_ddot, alpha_ddot = model.compute_accelerations(state, tau)
        derivs = np.array([state[2], state[3], theta_ddot, alpha_ddot])
        state = state + derivs * dt_sim
        
        t += dt_sim
    
    return {
        'times': np.array(times),
        'states': np.array(states),
        'torques': np.array(torques),
        'cmds': np.array(cmds),
        'fell': fell,
        'fall_time': fall_time,
        'gains': {'kp': kp, 'kd': kd, 'ktheta': ktheta, 'kthetadot': kthetadot}
    }


def analyze_stability(kp, kd, ktheta, kthetadot, outer_sign=+1.0):
    """Compute eigenvalues for given gains."""
    model = FurutaDynamics()
    p = model.p
    A, B = model.linearize()
    
    DEG_TO_RAD = np.pi / 180
    acc_to_tau = p.J_hat_0 / p.steps_per_rad
    
    # Convert to physical units
    k_a = (kp / DEG_TO_RAD) * acc_to_tau
    k_ad = (kd / DEG_TO_RAD) * acc_to_tau
    k_t = (ktheta / DEG_TO_RAD) * acc_to_tau
    k_td = (kthetadot / DEG_TO_RAD) * acc_to_tau
    
    # Feedback gain matrix
    K = np.array([[outer_sign * k_t, k_a, outer_sign * k_td, k_ad]])
    
    A_cl = A + B @ K
    eigs = np.linalg.eigvals(A_cl)
    
    return eigs


def plot_results(results: dict):
    """Plot simulation results."""
    t = results['times']
    states = results['states']
    gains = results['gains']
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # Pendulum angle
    ax = axes[0, 0]
    ax.plot(t, np.degrees(states[:, 1]), 'b-', linewidth=0.5)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax.axhline(y=35, color='red', linestyle=':', label='Limit')
    ax.axhline(y=-35, color='red', linestyle=':')
    ax.set_ylabel('α (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Pendulum Angle')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Arm position
    ax = axes[0, 1]
    ax.plot(t, np.degrees(states[:, 0]), 'g-', linewidth=0.5)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax.set_ylabel('θ (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Arm Position')
    ax.grid(True, alpha=0.3)
    
    # Pendulum rate
    ax = axes[1, 0]
    ax.plot(t, np.degrees(states[:, 3]), 'b-', linewidth=0.5, alpha=0.7)
    ax.set_ylabel('α̇ (deg/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Pendulum Angular Velocity')
    ax.grid(True, alpha=0.3)
    
    # Torque
    ax = axes[1, 1]
    ax.plot(t, np.array(results['torques']) * 1000, 'm-', linewidth=0.5, alpha=0.7)
    ax.set_ylabel('τ (mN·m)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Motor Torque')
    ax.grid(True, alpha=0.3)
    
    # Overall title
    status = "STABLE" if not results['fell'] else f"FELL at t={results['fall_time']:.2f}s"
    fig.suptitle(f"Gains: KP={gains['kp']}, KD={gains['kd']}, KTHETA={gains['ktheta']}, KTHETADOT={gains['kthetadot']}\n{status}", 
                 fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('tune_gains_result.png', dpi=150)
    print(f"\nSaved: tune_gains_result.png")
    plt.show()


def main():
    # Default gains (from firmware)
    kp = 742.0
    kd = 54.6
    ktheta = 15.0
    kthetadot = 60.0
    
    # Parse command line arguments
    if len(sys.argv) >= 5:
        kp = float(sys.argv[1])
        kd = float(sys.argv[2])
        ktheta = float(sys.argv[3])
        kthetadot = float(sys.argv[4])
    elif len(sys.argv) > 1:
        print("Usage: python3 tune_gains.py KP KD KTHETA KTHETADOT")
        print("Example: python3 tune_gains.py 800 60 20 80")
        sys.exit(1)
    
    print("=" * 70)
    print("FURUTA PENDULUM GAIN TUNING")
    print("=" * 70)
    print(f"\nGains: KP={kp}, KD={kd}, KTHETA={ktheta}, KTHETADOT={kthetadot}")
    
    # Eigenvalue analysis
    print("\n" + "-" * 70)
    print("EIGENVALUE ANALYSIS")
    print("-" * 70)
    
    # With minus sign (old)
    eigs_minus = analyze_stability(kp, kd, ktheta, kthetadot, outer_sign=-1.0)
    max_real_minus = max(e.real for e in eigs_minus)
    print(f"Outer = -KTHETA*θ (OLD): max λ = {max_real_minus:+.2f}")
    print(f"  Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_minus, key=lambda x: -x.real)]}")
    
    # With plus sign (new)
    eigs_plus = analyze_stability(kp, kd, ktheta, kthetadot, outer_sign=+1.0)
    max_real_plus = max(e.real for e in eigs_plus)
    print(f"Outer = +KTHETA*θ (NEW): max λ = {max_real_plus:+.2f}")
    print(f"  Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_plus, key=lambda x: -x.real)]}")
    
    if max_real_plus < 0:
        print("  → STABLE ✓")
    else:
        print("  → UNSTABLE ✗")
    
    # Simulate
    print("\n" + "-" * 70)
    print("SIMULATION (using +KTHETA*θ sign)")
    print("-" * 70)
    
    results = simulate_with_gains(kp, kd, ktheta, kthetadot, 
                                   initial_alpha_deg=3.0, 
                                   duration=5.0, 
                                   outer_sign=+1.0)
    
    if results['fell']:
        print(f"Result: FELL at t = {results['fall_time']:.2f} s")
    else:
        # Final state
        final = results['states'][-1]
        print(f"Result: STABLE")
        print(f"  Final θ = {np.degrees(final[0]):.2f}°")
        print(f"  Final α = {np.degrees(final[1]):.2f}°")
        
        # Settling metrics
        alpha_max = np.max(np.abs(np.degrees(results['states'][:, 1])))
        theta_max = np.max(np.abs(np.degrees(results['states'][:, 0])))
        print(f"  Max |α| = {alpha_max:.2f}°")
        print(f"  Max |θ| = {theta_max:.2f}°")
    
    # Plot
    plot_results(results)


if __name__ == "__main__":
    main()
