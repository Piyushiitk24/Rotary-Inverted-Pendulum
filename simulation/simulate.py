#!/usr/bin/env python3
"""
=======================================================================
FURUTA PENDULUM SIMULATION - MAIN ENTRY POINT
=======================================================================

This is the ONLY file you need to run for simulation.

USAGE:
------
1. Check hardware parameters:
   python3 simulate.py --params

2. Test with default gains:
   python3 simulate.py

3. Test with custom gains:
   python3 simulate.py --kp 800 --kd 60 --ktheta 20 --kthetadot 80

4. Test with different initial angle:
   python3 simulate.py --alpha0 5

5. Check stability only (no simulation):
   python3 simulate.py --stability-only

6. Compare old vs new outer loop sign:
   python3 simulate.py --compare-signs
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from furuta_model import FurutaParams, FurutaDynamics


def show_parameters():
    """Display all hardware parameters."""
    params = FurutaParams()
    model = FurutaDynamics(params)
    
    print("=" * 70)
    print("HARDWARE PARAMETERS (from FreeCAD geometry)")
    print("=" * 70)
    
    print(f"\n--- GEOMETRY ---")
    print(f"Arm length:        L_r = {params.L_r*1000:.1f} mm")
    print(f"Pendulum vertical: L_p = {params.rod_len_z*1000:.1f} mm")
    print(f"Pendulum axle:     L_h = {params.rod_len_x*1000:.1f} mm")
    print(f"Sphere diameter:        {params.sphere_d*1000:.1f} mm")
    
    print(f"\n--- MASSES ---")
    print(f"Arm mass:          m_r = {params.m_r*1000:.1f} g")
    print(f"Pendulum mass:     m_p = {params.m_p*1000:.2f} g")
    print(f"  (rod segment:         {params.m_rod*1000:.2f} g)")
    print(f"  (sphere:              {params.sphere_mass*1000:.2f} g)")
    
    print(f"\n--- COMPUTED DYNAMICS ---")
    print(f"Pendulum COM:      l_p = {params.l_p*1000:.2f} mm from hinge")
    print(f"Pendulum inertia:  J_1 = {params.J_1:.4e} kg·m²")
    print(f"Arm inertia:       J_0 = {params.J_hat_0:.4e} kg·m²")
    print(f"Coupling:          K   = {params.K:.4e} kg·m²")
    print(f"Gravity const:     G   = {params.G:.4e} N·m")
    
    print(f"\n--- STABILITY ---")
    print(f"Natural frequency: ω_n = {params.omega_n:.2f} rad/s ({params.f_n:.2f} Hz)")
    print(f"Fall time const:   τ   = {params.tau_fall*1000:.1f} ms")
    print(f"Open-loop poles:   {[f'{p:.2f}' for p in model.get_open_loop_poles()]}")


def analyze_stability(kp, kd, ktheta, kthetadot, outer_sign=+1.0):
    """Compute closed-loop eigenvalues."""
    model = FurutaDynamics()
    p = model.p
    A, B = model.linearize()
    
    DEG_TO_RAD = np.pi / 180
    acc_to_tau = p.J_hat_0 / p.steps_per_rad
    
    k_a = (kp / DEG_TO_RAD) * acc_to_tau
    k_ad = (kd / DEG_TO_RAD) * acc_to_tau
    k_t = (ktheta / DEG_TO_RAD) * acc_to_tau
    k_td = (kthetadot / DEG_TO_RAD) * acc_to_tau
    
    K = np.array([[outer_sign * k_t, k_a, outer_sign * k_td, k_ad]])
    A_cl = A + B @ K
    
    return np.linalg.eigvals(A_cl)


def simulate(kp, kd, ktheta, kthetadot, alpha0_deg=3.0, duration=5.0, outer_sign=+1.0, 
             ki=0.0, vel_leak=2.0, kv_theta=0.0):
    """
    Run closed-loop simulation matching EXACT main.cpp logic.
    
    Replicates the velocity-mode controller from firmware including:
    - Velocity integration with leak
    - Outer loop with CTRL_SIGN applied
    - Soft limits based on position
    - Filter-first derivative mode
    """
    model = FurutaDynamics()
    p = model.p
    
    # Limits (from main.cpp)
    MAX_ACC_STEPS = 20000.0
    MAX_SPEED_HZ = 25000.0
    LIM_PEND_DEG = 30.0
    LIM_MOTOR_DEG = 80.0
    ALPHA_GATE_DEG = 18.0
    ALPHA_DEADBAND_DEG = 0.5
    OUTER_MAX_ACC = 1000.0
    
    # Filter constants (from main.cpp)
    D_FILT_ALPHA = 0.35
    D_FILT_THETA = 0.20
    ALPHADOT_CLAMP = 300.0
    THETADOT_CLAMP = 300.0
    ALPHA_INT_CLAMP = 3.0
    
    # Signs: In simulation, physics model already has correct convention
    # alpha > 0 means pendulum tilted toward +theta direction
    # So we need alpha sign = +1 (opposite of hardware which has sensor inversion)
    motorSign = +1
    ALPHA_SIGN = +1  # Physics model convention, not sensor convention
    THETA_SIGN = +1
    CTRL_SIGN = +1
    
    # Timing
    dt_ctrl = 0.005  # 200 Hz
    dt_sim = 0.0001
    
    # Initial state
    state = np.array([0.0, alpha0_deg * np.pi / 180, 0.0, 0.0])
    
    # Controller state
    thetaDotCmdMotor = 0.0
    alphaInt = 0.0
    alphaDotFilt = 0.0
    thetaDotFilt = 0.0
    alphaFilt = 0.0
    lastAlphaFilt = 0.0
    thetaFilt = 0.0
    lastThetaFilt = 0.0
    lastAlphaRawSigned = 0.0
    lastThetaDeg = 0.0
    derivInit = False
    
    # Storage
    times, states, torques, acc_cmds, vel_cmds = [], [], [], [], []
    t, ctrl_time, tau = 0, 0, 0
    fell = False
    fall_time = None
    engage_start_t = 0.0  # Engage immediately (firmware does this too)
    engaged = False
    engage_time_ms = 0
    
    while t < duration:
        if t >= ctrl_time:
            theta, alpha, theta_dot_actual, alpha_dot_actual = state
            
            # Convert to degrees with signs
            alphaDegRaw = np.degrees(alpha)
            alphaRawSigned = ALPHA_SIGN * alphaDegRaw
            baseDegRaw = np.degrees(theta)
            thetaDeg = THETA_SIGN * baseDegRaw
            
            # Initialize filters on first step
            if not derivInit:
                alphaFilt = alphaRawSigned
                lastAlphaFilt = alphaRawSigned
                thetaFilt = thetaDeg
                lastThetaFilt = thetaDeg
                lastAlphaRawSigned = alphaRawSigned
                lastThetaDeg = thetaDeg
                alphaDotFilt = 0.0
                thetaDotFilt = 0.0
                derivInit = True
            
            # Derivative computation (filter-first mode)
            # Alpha derivative
            alphaFilt += D_FILT_ALPHA * (alphaRawSigned - alphaFilt)
            dAlphaFilt = alphaFilt - lastAlphaFilt
            # Wrap angle difference
            while dAlphaFilt > 180.0:
                dAlphaFilt -= 360.0
            while dAlphaFilt < -180.0:
                dAlphaFilt += 360.0
            alphaDotFilt = dAlphaFilt / dt_ctrl
            lastAlphaFilt = alphaFilt
            alphaDotFilt = np.clip(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP)
            
            # Theta derivative
            thetaFilt += D_FILT_THETA * (thetaDeg - thetaFilt)
            thetaDotFilt = (thetaFilt - lastThetaFilt) / dt_ctrl
            lastThetaFilt = thetaFilt
            thetaDotFilt = np.clip(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP)
            
            # Check engage conditions
            if not engaged and t >= engage_start_t:
                engaged = True
                engage_time_ms = t * 1000
            
            # Safety limits
            if abs(alphaDegRaw) > LIM_PEND_DEG or abs(baseDegRaw) > LIM_MOTOR_DEG:
                fell = True
                if fall_time is None:
                    fall_time = t
                tau, thetaDotCmdMotor = 0, 0
            elif engaged:
                # Apply deadband to alpha
                alphaCtrl = alphaRawSigned
                if abs(alphaCtrl) < ALPHA_DEADBAND_DEG:
                    alphaCtrl = 0.0
                
                # Integrate alpha error
                alphaInt += alphaCtrl * dt_ctrl
                alphaInt = np.clip(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP)
                
                # Inner loop (pendulum stabilization)
                accCmdPhysical = CTRL_SIGN * (kp * alphaCtrl + kd * alphaDotFilt + ki * alphaInt)
                
                # Outer loop (base centering) - CRITICAL: must apply CTRL_SIGN
                if abs(alphaCtrl) < ALPHA_GATE_DEG:
                    outer = -(ktheta * thetaDeg + kthetadot * thetaDotFilt)
                    outer = np.clip(outer, -OUTER_MAX_ACC, OUTER_MAX_ACC)
                    accCmdPhysical += CTRL_SIGN * outer  # KEY FIX: apply CTRL_SIGN
                
                # Ramp on engage (firmware does this)
                ramp = (t * 1000 - engage_time_ms) / 100.0  # 100ms ramp
                if ramp > 1.0:
                    ramp = 1.0
                accCmdPhysical *= ramp
                
                # Optional velocity damping
                if kv_theta > 0:
                    accCmdPhysical += -(kv_theta * thetaDotFilt)
                
                # Saturation
                sat = False
                if accCmdPhysical > MAX_ACC_STEPS:
                    accCmdPhysical = MAX_ACC_STEPS
                    sat = True
                if accCmdPhysical < -MAX_ACC_STEPS:
                    accCmdPhysical = -MAX_ACC_STEPS
                    sat = True
                
                # Anti-windup
                if sat:
                    alphaInt -= alphaCtrl * dt_ctrl
                    alphaInt = np.clip(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP)
                
                # Apply motor sign
                accCmdMotor = motorSign * accCmdPhysical
                
                # Convert acceleration command DIRECTLY to torque
                # The controller computes desired acceleration in steps/s²
                # Convert to torque: τ = (acc_steps/s² ÷ steps_per_rad) × J_hat_0
                tau = (accCmdMotor / p.steps_per_rad) * p.J_hat_0
                
                # For logging: also track what velocity would be if integrated
                thetaDotCmdMotor += accCmdMotor * dt_ctrl
                thetaDotCmdMotor = np.clip(thetaDotCmdMotor, -MAX_SPEED_HZ, MAX_SPEED_HZ)
                if vel_leak > 0:
                    thetaDotCmdMotor *= (1.0 - vel_leak * dt_ctrl)
                
                acc_cmds.append(accCmdPhysical)
                vel_cmds.append(thetaDotCmdMotor)
            else:
                tau = 0
                acc_cmds.append(0)
                vel_cmds.append(0)
            
            ctrl_time += dt_ctrl
        
        times.append(t)
        states.append(state.copy())
        torques.append(tau)
        
        theta_ddot, alpha_ddot = model.compute_accelerations(state, tau)
        state = state + np.array([state[2], state[3], theta_ddot, alpha_ddot]) * dt_sim
        t += dt_sim
    
    return {
        'times': np.array(times),
        'states': np.array(states),
        'torques': np.array(torques),
        'acc_cmds': np.array(acc_cmds) if acc_cmds else np.zeros(len(times)),
        'vel_cmds': np.array(vel_cmds) if vel_cmds else np.zeros(len(times)),
        'fell': fell,
        'fall_time': fall_time
    }


def plot_results(results, title="Simulation Results"):
    """Plot simulation results."""
    t = results['times']
    states = results['states']
    acc_cmds = results.get('acc_cmds', np.zeros(len(t)))
    vel_cmds = results.get('vel_cmds', np.zeros(len(t)))
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    
    # Pendulum angle
    axes[0, 0].plot(t, np.degrees(states[:, 1]), 'b-', lw=0.5)
    axes[0, 0].axhline(0, color='gray', ls='--', alpha=0.5)
    axes[0, 0].axhline(30, color='red', ls=':', alpha=0.5, label='Limit')
    axes[0, 0].axhline(-30, color='red', ls=':', alpha=0.5)
    axes[0, 0].set_ylabel('α (deg)')
    axes[0, 0].set_title('Pendulum Angle')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_ylim(-40, 40)
    
    # Arm position
    axes[0, 1].plot(t, np.degrees(states[:, 0]), 'g-', lw=0.5)
    axes[0, 1].axhline(0, color='gray', ls='--', alpha=0.5)
    axes[0, 1].axhline(80, color='red', ls=':', alpha=0.5, label='Limit')
    axes[0, 1].axhline(-80, color='red', ls=':', alpha=0.5)
    axes[0, 1].set_ylabel('θ (deg)')
    axes[0, 1].set_title('Arm Position (should stay near 0)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Velocities
    axes[1, 0].plot(t, np.degrees(states[:, 3]), 'b-', lw=0.5, alpha=0.7, label='α̇')
    axes[1, 0].plot(t, np.degrees(states[:, 2]), 'g-', lw=0.5, alpha=0.7, label='θ̇')
    axes[1, 0].set_ylabel('Angular velocity (deg/s)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_title('Velocities')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Acceleration command
    if len(acc_cmds) > 0:
        # Downsample to match control rate
        t_ctrl = t[::int(0.005/0.0001)][:len(acc_cmds)]
        axes[1, 1].plot(t_ctrl, acc_cmds, 'r-', lw=0.8, alpha=0.7)
        axes[1, 1].axhline(20000, color='red', ls='--', alpha=0.3, label='Limit')
        axes[1, 1].axhline(-20000, color='red', ls='--', alpha=0.3)
        axes[1, 1].set_ylabel('Acc cmd (steps/s²)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_title('Acceleration Command')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
    
    # Velocity command
    if len(vel_cmds) > 0:
        t_ctrl = t[::int(0.005/0.0001)][:len(vel_cmds)]
        axes[2, 0].plot(t_ctrl, vel_cmds, 'm-', lw=0.8, alpha=0.7)
        axes[2, 0].axhline(0, color='gray', ls='--', alpha=0.5)
        axes[2, 0].set_ylabel('Vel cmd (steps/s)')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_title('Velocity Command (should oscillate around 0)')
        axes[2, 0].grid(True, alpha=0.3)
    
    # Torque
    axes[2, 1].plot(t, np.array(results['torques']) * 1000, 'k-', lw=0.5, alpha=0.7)
    axes[2, 1].set_ylabel('τ (mN·m)')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_title('Motor Torque')
    axes[2, 1].grid(True, alpha=0.3)
    
    status = "STABLE ✓" if not results['fell'] else f"FELL at {results['fall_time']:.2f}s ✗"
    fig.suptitle(f"{title}\n{status}", fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('simulation_result.png', dpi=150)
    print(f"Saved: simulation_result.png")
    plt.show()


def compare_signs(kp, kd, ktheta, kthetadot, alpha0_deg=3.0):
    """Compare old (-) vs new (+) outer loop sign."""
    print("\n" + "=" * 70)
    print("COMPARING OUTER LOOP SIGNS")
    print("=" * 70)
    
    for sign, label in [(-1, "OLD: outer = -KTHETA*θ"), (+1, "NEW: outer = +KTHETA*θ")]:
        eigs = analyze_stability(kp, kd, ktheta, kthetadot, outer_sign=sign)
        max_real = max(e.real for e in eigs)
        status = "STABLE ✓" if max_real < 0 else "UNSTABLE ✗"
        print(f"\n{label}")
        print(f"  Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs, key=lambda x: -x.real)]}")
        print(f"  Max real part: {max_real:+.3f} → {status}")
    
    # Simulate both
    print("\n" + "-" * 70)
    print("SIMULATION COMPARISON (3s)")
    print("-" * 70)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    for i, (sign, label, color) in enumerate([(-1, "OLD (-)", "red"), (+1, "NEW (+)", "green")]):
        results = simulate(kp, kd, ktheta, kthetadot, alpha0_deg, 3.0, sign)
        
        status = "STABLE" if not results['fell'] else f"FELL at {results['fall_time']:.2f}s"
        print(f"{label}: {status}")
        
        axes[i].plot(results['times'], np.degrees(results['states'][:, 1]), color=color, lw=0.5)
        axes[i].axhline(0, color='gray', ls='--', alpha=0.5)
        axes[i].axhline(35, color='red', ls=':', alpha=0.5)
        axes[i].axhline(-35, color='red', ls=':', alpha=0.5)
        axes[i].set_xlabel('Time (s)')
        axes[i].set_ylabel('α (deg)')
        axes[i].set_title(f'{label}: {status}')
        axes[i].set_ylim(-45, 45)
        axes[i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('sign_comparison.png', dpi=150)
    print(f"\nSaved: sign_comparison.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Furuta Pendulum Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 simulate.py --params              # Show hardware parameters
  python3 simulate.py                       # Run with default gains
  python3 simulate.py --kp 800 --kd 60      # Custom inner loop gains
  python3 simulate.py --ktheta 20           # Custom outer loop gain
  python3 simulate.py --alpha0 5            # Start at 5° tilt
  python3 simulate.py --compare-signs       # Compare old vs new sign
  python3 simulate.py --stability-only      # Just check eigenvalues
        """
    )
    
    parser.add_argument('--params', action='store_true', help='Show hardware parameters')
    parser.add_argument('--compare-signs', action='store_true', help='Compare outer loop signs')
    parser.add_argument('--stability-only', action='store_true', help='Only check stability')
    
    parser.add_argument('--kp', type=float, default=742.0, help='Pendulum angle gain (default: 742)')
    parser.add_argument('--kd', type=float, default=54.6, help='Pendulum rate gain (default: 54.6)')
    parser.add_argument('--ki', type=float, default=0.0, help='Pendulum integral gain (default: 0)')
    parser.add_argument('--ktheta', type=float, default=5.0, help='Arm position gain (default: 5)')
    parser.add_argument('--kthetadot', type=float, default=10.0, help='Arm velocity gain (default: 10)')
    parser.add_argument('--vel-leak', type=float, default=2.0, help='Velocity leak rate 1/s (default: 2)')
    parser.add_argument('--kv-theta', type=float, default=0.0, help='Arm velocity damping (default: 0)')
    parser.add_argument('--alpha0', type=float, default=3.0, help='Initial pendulum angle in degrees (default: 3)')
    parser.add_argument('--duration', type=float, default=5.0, help='Simulation duration in seconds (default: 5)')
    
    args = parser.parse_args()
    
    if args.params:
        show_parameters()
        return
    
    if args.compare_signs:
        compare_signs(args.kp, args.kd, args.ktheta, args.kthetadot, args.alpha0)
        return
    
    print("=" * 70)
    print("FURUTA PENDULUM SIMULATION (matching main.cpp)")
    print("=" * 70)
    print(f"\nGains:")
    print(f"  Inner: KP={args.kp}, KD={args.kd}, KI={args.ki}")
    print(f"  Outer: KTHETA={args.ktheta}, KTHETADOT={args.kthetadot}")
    print(f"  Other: VEL_LEAK={args.vel_leak}, KV_THETA={args.kv_theta}")
    print(f"Initial angle: α₀ = {args.alpha0}°")
    
    # Stability check
    eigs = analyze_stability(args.kp, args.kd, args.ktheta, args.kthetadot, outer_sign=+1.0)
    max_real = max(e.real for e in eigs)
    print(f"\nEigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs, key=lambda x: -x.real)]}")
    print(f"Max real part: {max_real:+.3f} → {'STABLE ✓' if max_real < 0 else 'UNSTABLE ✗'}")
    
    if args.stability_only:
        return
    
    # Simulate
    print(f"\nSimulating for {args.duration}s...")
    results = simulate(args.kp, args.kd, args.ktheta, args.kthetadot, 
                       args.alpha0, args.duration, outer_sign=+1.0,
                       ki=args.ki, vel_leak=args.vel_leak, kv_theta=args.kv_theta)
    
    if results['fell']:
        print(f"Result: FELL at t = {results['fall_time']:.2f}s")
    else:
        final = results['states'][-1]
        print(f"Result: STABLE")
        print(f"  Final: θ = {np.degrees(final[0]):.2f}°, α = {np.degrees(final[1]):.4f}°")
        print(f"  Max |α| = {np.max(np.abs(np.degrees(results['states'][:, 1]))):.2f}°")
    
    # Plot
    title = f"KP={args.kp}, KD={args.kd}, KTHETA={args.ktheta}, KTHETADOT={args.kthetadot}"
    plot_results(results, title)


if __name__ == "__main__":
    main()
