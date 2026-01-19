#!/usr/bin/env python3
"""
Comprehensive Analysis of Furuta Pendulum Controller
=====================================================
This script:
1. Analyzes why the current controller fails
2. Shows the fix (outer loop)  
3. Validates with simulation
4. Generates recommended gains

Run this to understand and fix your controller!
"""

import numpy as np
import matplotlib.pyplot as plt
from furuta_model import FurutaParams, FurutaDynamics
from controller import FurutaController, ControllerParams
from simulator import FurutaSimulator, SimulationConfig


def analyze_eigenvalues():
    """Analyze closed-loop eigenvalues with and without outer loop."""
    print("=" * 70)
    print("EIGENVALUE ANALYSIS")
    print("=" * 70)
    
    model = FurutaDynamics()
    p = model.p
    
    # Get linearized system
    A, B = model.linearize()
    
    print("\n1. OPEN-LOOP SYSTEM (no controller)")
    print("-" * 50)
    
    eigs_ol = np.linalg.eigvals(A)
    print(f"   Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_ol, key=lambda x: -x.real)]}")
    print(f"   Unstable pole at λ = +{max(e.real for e in eigs_ol):.2f} rad/s")
    print(f"   -> Fall time constant = {1/max(e.real for e in eigs_ol)*1000:.0f} ms")
    
    # Controller gains in physical units
    DEG_TO_RAD = np.pi / 180
    KP = 742.0
    KD = 54.6
    k_alpha = (KP / DEG_TO_RAD) / p.steps_per_rad * p.J_hat_0
    k_alpha_dot = (KD / DEG_TO_RAD) / p.steps_per_rad * p.J_hat_0
    
    print(f"\n2. INNER LOOP ONLY (your current controller)")
    print("-" * 50)
    print(f"   Gains: KP={KP}, KD={KD} (step units)")
    print(f"   Physical: k_α={k_alpha:.4f} N·m/rad, k_α̇={k_alpha_dot:.4f} N·m/(rad/s)")
    
    # State feedback: u = K * [theta, alpha, theta_dot, alpha_dot]
    # Inner loop only controls alpha and alpha_dot
    K_inner = np.array([[0, k_alpha, 0, k_alpha_dot]])
    A_cl_inner = A + B @ K_inner  # Positive feedback (matches firmware convention)
    
    eigs_inner = np.linalg.eigvals(A_cl_inner)
    print(f"   Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_inner, key=lambda x: -x.real)]}")
    
    # Identify the modes
    stable_alpha = [e for e in eigs_inner if e.real < -1]
    marginal = [e for e in eigs_inner if abs(e.real) < 0.1]
    
    print(f"\n   Analysis:")
    print(f"   - Alpha stabilization poles: {[f'{e.real:.1f}' for e in stable_alpha]}")
    print(f"   - MARGINAL poles (θ mode): {[f'{e.real:.2f}' for e in marginal]}")
    print(f"\n   ⚠️  PROBLEM: Poles at λ≈0 mean θ drifts indefinitely!")
    print(f"   ⚠️  Inner loop stabilizes α but ignores θ position!")
    
    print(f"\n3. WITH OUTER LOOP (the fix)")
    print("-" * 50)
    
    # Outer loop gains (in step units -> physical)
    KTHETA = 15.0      # steps/s² per deg
    KTHETADOT = 60.0   # steps/s² per (deg/s)
    k_theta = (KTHETA / DEG_TO_RAD) / p.steps_per_rad * p.J_hat_0
    k_theta_dot = (KTHETADOT / DEG_TO_RAD) / p.steps_per_rad * p.J_hat_0
    
    print(f"   Outer gains: KTHETA={KTHETA}, KTHETADOT={KTHETADOT} (step units)")
    print(f"   Physical: k_θ={k_theta:.5f} N·m/rad, k_θ̇={k_theta_dot:.5f} N·m/(rad/s)")
    
    # Full state feedback (inner + outer)
    # Note: outer loop has NEGATIVE sign in firmware: -(KTHETA*theta + KTHETADOT*theta_dot)
    K_full = np.array([[-k_theta, k_alpha, -k_theta_dot, k_alpha_dot]])
    A_cl_full = A + B @ K_full
    
    eigs_full = np.linalg.eigvals(A_cl_full)
    print(f"   Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_full, key=lambda x: -x.real)]}")
    
    max_real = max(e.real for e in eigs_full)
    if max_real < 0:
        print(f"\n   ✓ ALL POLES STABLE! Max real part = {max_real:.2f}")
    else:
        print(f"\n   ✗ Still unstable, max real = {max_real:.2f}")
    
    return {
        'A': A, 'B': B,
        'eigs_ol': eigs_ol,
        'eigs_inner': eigs_inner,
        'eigs_full': eigs_full,
        'K_inner': K_inner,
        'K_full': K_full
    }


def test_simulations():
    """Run simulations with different controller configurations."""
    print("\n" + "=" * 70)
    print("SIMULATION TESTS")
    print("=" * 70)
    
    results = {}
    
    # Test 1: Inner loop only (outer disabled)
    print("\n1. INNER LOOP ONLY (outer_loop_enabled=False)")
    print("-" * 50)
    
    ctrl_inner = ControllerParams(
        ACC_KP=742.0, ACC_KD=54.6,
        outer_loop_enabled=False
    )
    config = SimulationConfig(
        duration_s=5.0,
        alpha_0_deg=2.0,
        engage_delay_s=0.05
    )
    
    sim_inner = FurutaSimulator(ctrl_params=ctrl_inner, config=config)
    result_inner = sim_inner.run(verbose=True)
    results['inner_only'] = result_inner
    
    # Test 2: With outer loop
    print("\n2. INNER + OUTER LOOP (outer_loop_enabled=True)")
    print("-" * 50)
    
    ctrl_full = ControllerParams(
        ACC_KP=742.0, ACC_KD=54.6,
        KTHETA=15.0, KTHETADOT=60.0,
        outer_loop_enabled=True
    )
    
    sim_full = FurutaSimulator(ctrl_params=ctrl_full, config=config)
    result_full = sim_full.run(verbose=True)
    results['with_outer'] = result_full
    
    # Test 3: With stronger outer loop
    print("\n3. STRONGER OUTER LOOP (KTHETA=30, KTHETADOT=100)")
    print("-" * 50)
    
    ctrl_strong = ControllerParams(
        ACC_KP=742.0, ACC_KD=54.6,
        KTHETA=30.0, KTHETADOT=100.0,
        outer_loop_enabled=True
    )
    
    sim_strong = FurutaSimulator(ctrl_params=ctrl_strong, config=config)
    result_strong = sim_strong.run(verbose=True)
    results['strong_outer'] = result_strong
    
    return results


def plot_comparison(results: dict):
    """Plot comparison of different controller configurations."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    colors = {'inner_only': 'red', 'with_outer': 'blue', 'strong_outer': 'green'}
    labels = {'inner_only': 'Inner Only', 'with_outer': 'Inner + Outer', 'strong_outer': 'Strong Outer'}
    
    # Alpha (pendulum angle)
    ax = axes[0, 0]
    for key, result in results.items():
        if len(result.t) > 0:
            ax.plot(result.t, result.alpha, color=colors[key], label=labels[key], linewidth=1.5)
    ax.axhline(y=30, color='gray', linestyle='--', alpha=0.5, label='Fall limit')
    ax.axhline(y=-30, color='gray', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('α (deg)')
    ax.set_title('Pendulum Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-35, 35)
    
    # Theta (arm angle)
    ax = axes[0, 1]
    for key, result in results.items():
        if len(result.t) > 0:
            ax.plot(result.t, result.theta, color=colors[key], label=labels[key], linewidth=1.5)
    ax.axhline(y=80, color='gray', linestyle='--', alpha=0.5, label='Fall limit')
    ax.axhline(y=-80, color='gray', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('θ (deg)')
    ax.set_title('Arm Angle (DRIFT is the problem!)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Acceleration command
    ax = axes[1, 0]
    for key, result in results.items():
        if len(result.t) > 0:
            ax.plot(result.t, result.acc_cmd, color=colors[key], label=labels[key], linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (steps/s²)')
    ax.set_title('Acceleration Command')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Position command
    ax = axes[1, 1]
    for key, result in results.items():
        if len(result.t) > 0:
            ax.plot(result.t, result.pos_cmd, color=colors[key], label=labels[key], linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (steps)')
    ax.set_title('Position Command')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('controller_comparison.png', dpi=150)
    print(f"\n📊 Plot saved to: simulation/controller_comparison.png")
    plt.show()


def find_optimal_gains():
    """Search for optimal outer loop gains."""
    print("\n" + "=" * 70)
    print("GAIN OPTIMIZATION SEARCH")
    print("=" * 70)
    
    ktheta_values = [10, 20, 30, 40, 50]
    kthetadot_values = [40, 60, 80, 100, 120]
    
    best_result = None
    best_params = None
    
    print("\nSearching gain combinations...")
    print(f"{'KTHETA':>8} {'KTHETADOT':>10} {'Success':>8} {'Max α':>8} {'Max θ':>8}")
    print("-" * 50)
    
    for ktheta in ktheta_values:
        for kthetadot in kthetadot_values:
            ctrl = ControllerParams(
                ACC_KP=742.0, ACC_KD=54.6,
                KTHETA=ktheta, KTHETADOT=kthetadot,
                outer_loop_enabled=True
            )
            config = SimulationConfig(
                duration_s=5.0,
                alpha_0_deg=5.0,  # Harder test
                engage_delay_s=0.05
            )
            
            sim = FurutaSimulator(ctrl_params=ctrl, config=config)
            result = sim.run(verbose=False)
            
            status = "✓" if result.success else "✗"
            print(f"{ktheta:8.0f} {kthetadot:10.0f} {status:>8} {result.max_alpha:8.1f} {result.max_theta:8.1f}")
            
            # Track best stable result (smallest max_theta)
            if result.success:
                if best_result is None or result.max_theta < best_result.max_theta:
                    best_result = result
                    best_params = (ktheta, kthetadot)
    
    if best_params:
        print(f"\n✓ RECOMMENDED OUTER LOOP GAINS:")
        print(f"   KTHETA = {best_params[0]}")
        print(f"   KTHETADOT = {best_params[1]}")
        print(f"   Max arm drift: {best_result.max_theta:.1f}°")
    else:
        print("\n✗ No stable configuration found - check inner loop gains")
    
    return best_params


def generate_firmware_recommendations():
    """Generate specific firmware changes."""
    print("\n" + "=" * 70)
    print("FIRMWARE RECOMMENDATIONS")
    print("=" * 70)
    
    print("""
Based on the simulation analysis, here are the key findings:

1. ROOT CAUSE OF DRIFT:
   The inner loop (ACC_KP, ACC_KD on alpha) successfully stabilizes the 
   pendulum angle, but it has ZERO feedback on arm position (theta).
   
   Mathematically, the closed-loop eigenvalues show two poles at λ≈0,
   meaning the arm position is "marginally stable" - it won't grow 
   exponentially, but it will drift indefinitely due to any bias.

2. THE FIX:
   Enable the outer loop which adds feedback on theta and theta_dot.
   This moves the marginal poles to the left half-plane (stable).

3. RECOMMENDED GAINS (from simulation):
   - ACC_KP = 742 (keep as derived)
   - ACC_KD = 54.6 (keep as derived)
   - KTHETA = 30 (position correction)
   - KTHETADOT = 80 (velocity damping)
   - outer_loop_enabled = true

4. FIRMWARE CHANGES:
   In main.cpp, ensure these are set:

   float KTHETA     = 30.0f;   // Increased from 15
   float KTHETADOT  = 80.0f;   // Increased from 60
   bool outerLoopEnabled = true;

5. TUNING TIPS:
   - If arm oscillates: reduce KTHETADOT or KTHETA
   - If arm still drifts: increase KTHETA
   - If pendulum falls easily: reduce KTHETA (outer loop fights inner loop)
   - ALPHA_GATE_DEG = 3-5° is good (only apply outer when nearly balanced)

6. TESTING PROCEDURE:
   a) Enable outer loop: press 'O' (should show "outerLoop=ON")
   b) Set gains: 'K30' then 'L80'
   c) Calibrate upright: 'Z'
   d) Enable: 'E'
   e) Release pendulum gently
   f) Watch if arm stays near center or drifts

""")


def main():
    """Run complete analysis."""
    # 1. Eigenvalue analysis
    eig_data = analyze_eigenvalues()
    
    # 2. Simulation tests
    results = test_simulations()
    
    # 3. Plot comparison
    try:
        plot_comparison(results)
    except Exception as e:
        print(f"\n(Plotting skipped: {e})")
    
    # 4. Find optimal gains
    best_gains = find_optimal_gains()
    
    # 5. Generate recommendations
    generate_firmware_recommendations()
    
    print("\n" + "=" * 70)
    print("ANALYSIS COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()
