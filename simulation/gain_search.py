#!/usr/bin/env python3
"""
Comprehensive gain search to find stable configurations.
Tests many combinations of inner and outer loop gains.
"""

import numpy as np
from furuta_model import FurutaParams, FurutaDynamics


def full_gain_search():
    """Search for stable gain combinations using eigenvalue analysis."""
    print("=" * 70)
    print("FULL GAIN SEARCH")
    print("=" * 70)
    
    model = FurutaDynamics()
    p = model.p
    A, B = model.linearize()
    
    DEG_TO_RAD = np.pi / 180
    steps_per_rad = p.steps_per_rad
    acc_to_tau = p.J_hat_0 / steps_per_rad
    
    stable_configs = []
    
    # Search over a wide range
    kp_values = [500, 742, 1000, 1500, 2000, 3000, 4000, 5000]
    kd_values = [30, 54.6, 80, 100, 150, 200, 300]
    ktheta_values = [0, 5, 10, 15, 20, 30]  # Include 0 = outer off
    kthetadot_values = [0, 20, 40, 60, 80, 100]
    
    print(f"\nSearching {len(kp_values)*len(kd_values)*len(ktheta_values)*len(kthetadot_values)} combinations...")
    print("\nStable configurations found:")
    print(f"{'KP':>6} {'KD':>6} {'KTHETA':>8} {'KTHETADOT':>10} {'Max λ':>8}")
    print("-" * 50)
    
    for kp in kp_values:
        for kd in kd_values:
            for ktheta in ktheta_values:
                for kthetadot in kthetadot_values:
                    k_a = (kp / DEG_TO_RAD) * acc_to_tau
                    k_ad = (kd / DEG_TO_RAD) * acc_to_tau
                    k_t = (ktheta / DEG_TO_RAD) * acc_to_tau
                    k_td = (kthetadot / DEG_TO_RAD) * acc_to_tau
                    
                    K = np.array([[-k_t, k_a, -k_td, k_ad]])
                    A_cl = A + B @ K
                    eigs = np.linalg.eigvals(A_cl)
                    max_real = max(e.real for e in eigs)
                    
                    if max_real < -0.1:  # Require some stability margin
                        stable_configs.append({
                            'kp': kp, 'kd': kd, 
                            'ktheta': ktheta, 'kthetadot': kthetadot,
                            'max_eig': max_real
                        })
                        print(f"{kp:6.0f} {kd:6.1f} {ktheta:8.0f} {kthetadot:10.0f} {max_real:+8.2f}")
    
    if not stable_configs:
        print("\nNo stable configurations found in search range!")
        print("\nLet's try VERY high gains...")
        
        # Try much higher
        for kp in [6000, 8000, 10000]:
            for kd in [400, 500, 600]:
                for ktheta in [0, 5, 10]:
                    for kthetadot in [0, 20, 40]:
                        k_a = (kp / DEG_TO_RAD) * acc_to_tau
                        k_ad = (kd / DEG_TO_RAD) * acc_to_tau
                        k_t = (ktheta / DEG_TO_RAD) * acc_to_tau
                        k_td = (kthetadot / DEG_TO_RAD) * acc_to_tau
                        
                        K = np.array([[-k_t, k_a, -k_td, k_ad]])
                        A_cl = A + B @ K
                        eigs = np.linalg.eigvals(A_cl)
                        max_real = max(e.real for e in eigs)
                        
                        if max_real < 0:
                            stable_configs.append({
                                'kp': kp, 'kd': kd, 
                                'ktheta': ktheta, 'kthetadot': kthetadot,
                                'max_eig': max_real
                            })
                            print(f"{kp:6.0f} {kd:6.1f} {ktheta:8.0f} {kthetadot:10.0f} {max_real:+8.2f}")
    
    return stable_configs


def check_controller_structure():
    """
    Check if the controller structure itself is the problem.
    A standard PD controller with acceleration output may have issues.
    """
    print("\n" + "=" * 70)
    print("CHECKING CONTROLLER STRUCTURE")
    print("=" * 70)
    
    model = FurutaDynamics()
    p = model.p
    A, B = model.linearize()
    
    print("""
The current controller structure:
  accCmd = KP * α + KD * α̇ - KTHETA * θ - KTHETADOT * θ̇
  
This is then INTEGRATED TWICE to get position:
  velCmd += accCmd * dt
  posCmd += velCmd * dt

This means the controller has TWO INTEGRATORS in the forward path!

For a type-2 system (position control with acceleration input),
this is actually correct. But let's verify...
""")

    # The transfer function from α to accCmd is just a PD controller
    # But the transfer function from accCmd to θ is 1/s²
    # And from θ to α there's coupling through the physics
    
    # Let's check the open-loop transfer function structure
    print("The linearized plant has the structure:")
    print(f"  [θ̈]   [  0   {A[2,1]:.2f}] [θ]   [{B[2,0]:.2f}]")
    print(f"  [α̈] = [{A[3,0]:.2f} {A[3,1]:.2f}] [α] + [{B[3,0]:.2f}] τ")
    
    print("""
For the inner loop (α stabilization):
  The characteristic equation is:
  s² - (A42 + B4*k_α)*α - B4*k_αd*s*α = 0
  
  For stability: (A42 + B4*k_α) < 0
  
  A42 = 153.93, B4 = -2646.43
  Need: 153.93 - 2646.43*k_α < 0
  k_α > 0.058 N·m/rad
  
  With KP=742: k_α = 0.188 N·m/rad > 0.058 ✓
  
The problem is the COUPLING through A_cl[3,0]!
""")
    
    print(f"""
When outer loop is added:
  A_cl[3,0] = B[3,0] * (-k_θ) = -2646.43 * (-k_θ) = +2646.43 * k_θ
  
This term is ALWAYS POSITIVE when k_θ > 0!
It represents: "when arm is displaced, pendulum experiences destabilizing torque"

This coupling means that ANY positive KTHETA will destabilize α to some degree.
The inner loop must work harder to compensate.

For the system to be stable, the inner loop's stabilizing effect 
(A_cl[3,1] = -343.8) must overcome the outer loop's destabilizing 
effect (A_cl[3,0] * θ).

The effective condition is roughly:
  |A_cl[3,1] * α| > |A_cl[3,0] * θ|
  
During transients, both α and θ are changing. The ratio of their
amplitudes depends on the initial conditions and the dynamics.
""")

    # Alternative: COLLOCATED CONTROL
    print("\n" + "-" * 70)
    print("ALTERNATIVE: CASCADED CONTROL STRUCTURE")
    print("-" * 70)
    print("""
A more robust structure is CASCADED control:

1. INNER LOOP (fast): Stabilize α using arm acceleration
   - This is what we have now
   - Output: desired θ acceleration
   
2. OUTER LOOP (slow): Use DESIRED α to control θ
   - Generate a small α setpoint that will produce desired θ force
   - α_setpoint = -k_θ * θ / K_coupling
   - This uses the physics coupling instead of fighting it!

This is called "reaction wheel" or "indirect" control.
The arm moves by TILTING THE PENDULUM SLIGHTLY, which the 
inner loop then stabilizes, causing the arm to move.

But this requires careful tuning and is more complex...
""")

    # LQR approach
    print("\n" + "-" * 70)
    print("OPTIMAL APPROACH: LQR DESIGN")
    print("-" * 70)
    
    try:
        from scipy.linalg import solve_continuous_are
        
        # State cost
        Q = np.diag([100, 1000, 10, 100])  # Weight θ, α (α most important)
        R = np.array([[1]])  # Control cost
        
        # Solve ARE
        P = solve_continuous_are(A, B, Q, R)
        K_lqr = np.linalg.inv(R) @ B.T @ P
        
        A_cl_lqr = A - B @ K_lqr
        eigs_lqr = np.linalg.eigvals(A_cl_lqr)
        
        print(f"LQR Gain: K = {K_lqr.flatten()}")
        print(f"Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_lqr, key=lambda x: -x.real)]}")
        
        # Convert to firmware units
        DEG_TO_RAD = np.pi / 180
        acc_to_tau = model.p.J_hat_0 / model.p.steps_per_rad
        
        k_lqr = K_lqr.flatten()
        kp_equiv = -k_lqr[1] / acc_to_tau * DEG_TO_RAD
        kd_equiv = -k_lqr[3] / acc_to_tau * DEG_TO_RAD
        ktheta_equiv = k_lqr[0] / acc_to_tau * DEG_TO_RAD
        kthetadot_equiv = k_lqr[2] / acc_to_tau * DEG_TO_RAD
        
        print(f"\nEquivalent firmware gains (step units):")
        print(f"  ACC_KP ≈ {kp_equiv:.0f}")
        print(f"  ACC_KD ≈ {kd_equiv:.1f}")
        print(f"  KTHETA ≈ {-ktheta_equiv:.1f} (NOTE: opposite sign!)")
        print(f"  KTHETADOT ≈ {-kthetadot_equiv:.1f}")
        
        if max(e.real for e in eigs_lqr) < 0:
            print("\n✓ LQR solution is STABLE!")
            print("\nNOTICE: LQR gives POSITIVE KTHETA feedback!")
            print("This means: outer = +KTHETA * theta (NOT minus!)")
            print("The firmware has: outer = -KTHETA * theta")
            print("\n>>> TRY FLIPPING THE OUTER LOOP SIGN! <<<")
        
    except Exception as e:
        print(f"LQR computation failed: {e}")


def verify_sign_fix():
    """Test if flipping the outer loop sign fixes the problem."""
    print("\n" + "=" * 70)
    print("TESTING OUTER LOOP SIGN FIX")
    print("=" * 70)
    
    model = FurutaDynamics()
    p = model.p
    A, B = model.linearize()
    
    DEG_TO_RAD = np.pi / 180
    acc_to_tau = p.J_hat_0 / p.steps_per_rad
    
    # Current firmware (outer = -KTHETA * theta)
    kp, kd = 742, 54.6
    ktheta, kthetadot = 15, 60
    
    k_a = (kp / DEG_TO_RAD) * acc_to_tau
    k_ad = (kd / DEG_TO_RAD) * acc_to_tau
    k_t = (ktheta / DEG_TO_RAD) * acc_to_tau
    k_td = (kthetadot / DEG_TO_RAD) * acc_to_tau
    
    # Current: K = [-k_t, +k_a, -k_td, +k_ad]  (outer has minus)
    K_current = np.array([[-k_t, k_a, -k_td, k_ad]])
    A_cl_current = A + B @ K_current
    eigs_current = np.linalg.eigvals(A_cl_current)
    
    print(f"\nCURRENT (outer = -KTHETA*θ):")
    print(f"  K = [{-k_t:.5f}, {k_a:.5f}, {-k_td:.5f}, {k_ad:.5f}]")
    print(f"  Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_current, key=lambda x: -x.real)]}")
    print(f"  Max real: {max(e.real for e in eigs_current):+.2f}")
    
    # Flipped: K = [+k_t, +k_a, +k_td, +k_ad]  (outer has plus)
    K_flipped = np.array([[+k_t, k_a, +k_td, k_ad]])
    A_cl_flipped = A + B @ K_flipped
    eigs_flipped = np.linalg.eigvals(A_cl_flipped)
    
    print(f"\nFLIPPED (outer = +KTHETA*θ):")
    print(f"  K = [{+k_t:.5f}, {k_a:.5f}, {+k_td:.5f}, {k_ad:.5f}]")
    print(f"  Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_flipped, key=lambda x: -x.real)]}")
    print(f"  Max real: {max(e.real for e in eigs_flipped):+.2f}")
    
    if max(e.real for e in eigs_flipped) < 0:
        print("\n>>> FLIPPING THE OUTER LOOP SIGN MAKES IT STABLE! <<<")
        print("\nFIRMWARE FIX:")
        print("  Change: outer = -(KTHETA * thetaDeg + KTHETADOT * thetaDotFilt);")
        print("  To:     outer = +(KTHETA * thetaDeg + KTHETADOT * thetaDotFilt);")
    else:
        # Try with smaller outer gains
        print("\nStill unstable. Trying smaller outer gains with flipped sign...")
        
        for kt in [5, 10, 15]:
            for ktd in [20, 40, 60]:
                k_t = (kt / DEG_TO_RAD) * acc_to_tau
                k_td = (ktd / DEG_TO_RAD) * acc_to_tau
                
                K_test = np.array([[+k_t, k_a, +k_td, k_ad]])
                A_cl_test = A + B @ K_test
                eigs_test = np.linalg.eigvals(A_cl_test)
                max_real = max(e.real for e in eigs_test)
                
                if max_real < 0:
                    print(f"  KTHETA={kt}, KTHETADOT={ktd}: Max λ = {max_real:+.2f} STABLE!")


def main():
    check_controller_structure()
    verify_sign_fix()
    
    stable_configs = full_gain_search()
    
    print("\n" + "=" * 70)
    print("FINAL RECOMMENDATION")
    print("=" * 70)


if __name__ == "__main__":
    main()
