#!/usr/bin/env python3
"""
Gain analysis for Furuta pendulum controller.
Uses linearized dynamics to find stability regions.
"""

import numpy as np
from furuta_model import FurutaDynamics, FurutaParams
from controller import FurutaController, ControllerParams
from simulator import FurutaSimulator, SimulationConfig

def main():
    # Create components
    model_params = FurutaParams()
    model = FurutaDynamics(model_params)
    ctrl_params = ControllerParams()
    controller = FurutaController(ctrl_params, model_params)

    print("="*60)
    print("GAIN ANALYSIS")
    print("="*60)

    # Show current gains
    print(f"\nCurrent gains:")
    print(f"  KP = {ctrl_params.ACC_KP}")
    print(f"  KD = {ctrl_params.ACC_KD}")
    print(f"  KTHETA = {ctrl_params.KTHETA}")
    print(f"  KTHETADOT = {ctrl_params.KTHETADOT}")

    # Using linearized matrices (α small)
    print("\n" + "-"*60)
    print("LINEARIZED ANALYSIS")
    print("-"*60)

    p = model_params
    J0 = p.J_hat_0  # arm inertia  
    J1 = p.J_1      # pendulum inertia
    K = p.K         # coupling
    G = p.G         # gravity term

    # Effective inertia denominator
    det = J0 * J1 - K**2
    print(f"\nInertia matrix:")
    print(f"  J₀ = {J0:.6e}")
    print(f"  J₁ = {J1:.6e}")  
    print(f"  K = {K:.6e}")
    print(f"  det = {det:.6e}")

    # Linearized A matrix at upright (α=0)
    a_theta_alpha = K * G / det
    a_alpha_alpha = J0 * G / det
    b_theta = J1 / det  
    b_alpha = -K / det

    print(f"\nLinearized coefficients:")
    print(f"  θ̈ = {a_theta_alpha:.2f}*α + {b_theta:.2f}*τ")
    print(f"  α̈ = {a_alpha_alpha:.2f}*α + {b_alpha:.2f}*τ")

    # A matrix (4x4)
    A = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, a_theta_alpha, 0, 0],
        [0, a_alpha_alpha, 0, 0]
    ])

    B = np.array([[0], [0], [b_theta], [b_alpha]])

    # Eigenvalues
    eigs = np.linalg.eigvals(A)
    print(f"\nOpen-loop eigenvalues: {[f'{e:.2f}' for e in eigs]}")

    # Motor conversion
    STEPS_PER_RAD = model_params.steps_per_rad
    print(f"\nMotor conversion: STEPS_PER_RAD = {STEPS_PER_RAD:.2f}")

    # Current gains in "torque equivalent":
    k_alpha_eff = ctrl_params.ACC_KP / STEPS_PER_RAD * J0
    k_alphadot_eff = ctrl_params.ACC_KD / STEPS_PER_RAD * J0

    print(f"\nEffective torque gains:")
    print(f"  k_α = {k_alpha_eff:.6f} N·m/rad")
    print(f"  k_α̇ = {k_alphadot_eff:.6f} N·m/(rad/s)")

    # Closed-loop eigenvalues - BUT the controller uses DEGREES not radians!
    # Controller: acc = KP * alpha_deg + KD * alpha_dot_deg
    # So effective gain per radian = KP * 180/π
    DEG_TO_RAD = np.pi / 180
    k_alpha_rad = ctrl_params.ACC_KP / DEG_TO_RAD / STEPS_PER_RAD * J0
    k_alphadot_rad = ctrl_params.ACC_KD / DEG_TO_RAD / STEPS_PER_RAD * J0
    
    print(f"\nEffective torque gains (accounting for degrees):")
    print(f"  k_α = {k_alpha_rad:.6f} N·m/rad")
    print(f"  k_α̇ = {k_alphadot_rad:.6f} N·m/(rad/s)")

    # Closed-loop eigenvalues
    K_ctrl = np.array([[0, k_alpha_rad, 0, k_alphadot_rad]])
    A_cl = A - B @ K_ctrl

    eigs_cl = np.linalg.eigvals(A_cl)
    print(f"\nClosed-loop eigenvalues:")
    for e in eigs_cl:
        status = "stable" if e.real < 0 else "UNSTABLE"
        print(f"  {e.real:+.4f} {e.imag:+.4f}j  [{status}]")

    # Minimum gain for stability
    k_min = a_alpha_alpha / abs(b_alpha)
    print(f"\nMinimum k_α for stability: {k_min:.4f} N·m/rad")
    print(f"Current k_α: {k_alpha_rad:.4f} N·m/rad")

    if k_alpha_rad > k_min:
        print("✓ Gain is sufficient for LINEAR stability")
    else:
        print("✗ GAIN TOO LOW - need to increase!")
        
    # Gain sweep
    print(f"\n" + "="*60)
    print("GAIN SWEEP (checking linear stability)")
    print("="*60)

    test_gains = [
        (500, 40, "Lower"),
        (742, 54.6, "Current"),
        (1000, 70, "Higher"),
        (1500, 100, "Much Higher"),
        (2000, 120, "Very High"),
    ]

    for kp, kd, name in test_gains:
        k_a = kp / DEG_TO_RAD / STEPS_PER_RAD * J0
        k_ad = kd / DEG_TO_RAD / STEPS_PER_RAD * J0
        K_test = np.array([[0, k_a, 0, k_ad]])
        A_test = A - B @ K_test
        eigs_test = np.linalg.eigvals(A_test)
        max_real = max(e.real for e in eigs_test)
        status = "STABLE" if max_real < 0 else "UNSTABLE"
        print(f"  {name:15s} KP={kp:5.0f}, KD={kd:5.1f}: max_real={max_real:+.3f} [{status}]")

    # Now check what's happening in simulation
    print(f"\n" + "="*60)
    print("NONLINEAR SIMULATION TEST")
    print("="*60)
    
    # Try different gains
    for kp, kd in [(742, 54.6), (1200, 80), (1800, 110), (2500, 150)]:
        ctrl = ControllerParams(ACC_KP=kp, ACC_KD=kd)
        cfg = SimulationConfig(
            duration_s=5.0,
            alpha_0_deg=5.0,  
            engage_delay_s=0.05
        )
        sim = FurutaSimulator(ctrl_params=ctrl, config=cfg)
        result = sim.run(verbose=False)
        
        status = "✓ STABLE" if result.success else f"✗ fell at t={result.fall_time:.2f}s"
        print(f"  KP={kp:5.0f}, KD={kd:5.1f}: {status}")

if __name__ == "__main__":
    main()
