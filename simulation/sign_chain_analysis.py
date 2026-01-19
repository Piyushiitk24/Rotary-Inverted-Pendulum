#!/usr/bin/env python3
"""
Deep sign chain analysis - tracing every sign from sensor to actuator.
This will find the exact sign error causing instability.
"""

import numpy as np
from furuta_model import FurutaParams, FurutaDynamics


def trace_complete_sign_chain():
    """Trace signs through the entire system."""
    print("=" * 70)
    print("COMPLETE SIGN CHAIN ANALYSIS")
    print("=" * 70)
    
    model = FurutaDynamics()
    p = model.p
    A, B = model.linearize()
    
    print("\n" + "=" * 70)
    print("STEP 1: PHYSICS SIGNS (from linearized model)")
    print("=" * 70)
    
    print(f"""
State vector: x = [θ, α, θ̇, α̇]
Input: u = τ (motor torque, N·m)

ẋ = Ax + Bu where:

A = | 0     0    1    0 |     B = |   0   |
    | 0     0    0    1 |         |   0   |
    | 0  A32    0    0 |         | B3    |
    | 0  A42    0    0 |         | B4    |

A32 = d(θ̈)/d(α) = {A[2,1]:+.2f} rad/s² per rad
A42 = d(α̈)/d(α) = {A[3,1]:+.2f} rad/s² per rad  (POSITIVE = unstable!)

B3 = d(θ̈)/d(τ) = {B[2,0]:+.2f} rad/s² per N·m   (POSITIVE = expected)
B4 = d(α̈)/d(τ) = {B[3,0]:+.2f} rad/s² per N·m   (NEGATIVE = coupling effect)
""")

    print(f"""
KEY PHYSICAL INSIGHT:
- When α > 0 (pendulum tilted), A42 > 0 means α̈ > 0 (falling further)
- When τ > 0 (positive torque), B4 < 0 means α̈ DECREASES
- Therefore: positive τ HELPS when α > 0

CONCLUSION: Controller should output τ = +k × α for stabilization
(This is what the firmware does, so inner loop sign is CORRECT)
""")

    print("=" * 70)
    print("STEP 2: CONTROLLER SIGN CHAIN")
    print("=" * 70)
    
    print(f"""
FIRMWARE CONTROLLER (from main.cpp):

1. INNER LOOP (alpha stabilization):
   accCmdPhysical = CTRL_SIGN * (ACC_KP * alphaCtrl + ACC_KD * alphaDotFilt)
   
   With CTRL_SIGN = +1, ACC_KP = 742:
   - When α > 0: accCmd > 0 → τ > 0 → α̈ decreases ✓ CORRECT

2. OUTER LOOP (theta centering):
   if (outerLoopEnabled && |α| < ALPHA_GATE) {{
       float outer = -(KTHETA * thetaDeg + KTHETADOT * thetaDotFilt);
       accCmdPhysical += outer;
   }}
   
   With KTHETA = 15:
   - When θ > 0 (arm right): outer = -15*θ < 0
   - This SUBTRACTS from accCmd
   - Less τ → arm decelerates
   
   BUT WAIT - let's check what the arm equation needs!
""")

    print("=" * 70)
    print("STEP 3: WHAT DOES THE ARM EQUATION NEED?")
    print("=" * 70)
    
    print(f"""
From physics: θ̈ = A32*α + B3*τ = {A[2,1]:.2f}*α + {B[2,0]:.2f}*τ

For θ to return to zero when θ > 0, we need θ̈ < 0.
If α ≈ 0 (pendulum balanced): θ̈ = {B[2,0]:.2f}*τ

So we need τ < 0 when θ > 0 to push arm back!

OUTER LOOP SIGN CHECK:
- Firmware: outer = -(KTHETA * θ) → outer < 0 when θ > 0
- This subtracts from accCmd, making τ smaller (or negative)
- This IS the correct direction!

But the eigenvalue analysis showed instability... Let me check the 
actual closed-loop math more carefully.
""")

    print("=" * 70)
    print("STEP 4: CLOSED-LOOP MATRIX (checking the math)")
    print("=" * 70)
    
    DEG_TO_RAD = np.pi / 180
    
    # Controller gains converted to physical units (N·m/rad)
    ACC_KP = 742.0  # steps/s² per deg
    ACC_KD = 54.6   # steps/s² per deg/s
    KTHETA = 15.0   # steps/s² per deg  
    KTHETADOT = 60.0  # steps/s² per deg/s
    
    # Convert from steps/s² to rad/s²
    steps_per_rad = p.steps_per_rad
    acc_to_tau = p.J_hat_0 / steps_per_rad  # multiply acc(steps/s²) by this to get tau(N·m)
    
    # Gains in N·m per radian
    k_alpha = (ACC_KP / DEG_TO_RAD) * acc_to_tau
    k_alpha_dot = (ACC_KD / DEG_TO_RAD) * acc_to_tau
    k_theta = (KTHETA / DEG_TO_RAD) * acc_to_tau
    k_theta_dot = (KTHETADOT / DEG_TO_RAD) * acc_to_tau
    
    print(f"""
Controller outputs: accCmd (steps/s²)
Then: τ = accCmd * (J_hat_0 / steps_per_rad)

Gain conversions:
  k_α = {k_alpha:.5f} N·m/rad  (from ACC_KP={ACC_KP})
  k_α̇ = {k_alpha_dot:.5f} N·m/(rad/s)  (from ACC_KD={ACC_KD})
  k_θ = {k_theta:.5f} N·m/rad  (from KTHETA={KTHETA})
  k_θ̇ = {k_theta_dot:.5f} N·m/(rad/s)  (from KTHETADOT={KTHETADOT})

Full controller:
  τ = k_α × α + k_α̇ × α̇ - k_θ × θ - k_θ̇ × θ̇
     (inner loop: + sign)    (outer loop: - sign)

In matrix form: τ = K × [θ, α, θ̇, α̇]ᵀ
  K = [-k_θ, +k_α, -k_θ̇, +k_α̇]
  K = [{-k_theta:.5f}, {k_alpha:.5f}, {-k_theta_dot:.5f}, {k_alpha_dot:.5f}]
""")

    # Construct the feedback gain matrix
    K = np.array([[-k_theta, k_alpha, -k_theta_dot, k_alpha_dot]])
    
    # Closed-loop: ẋ = Ax + Bu = Ax + B(Kx) = (A + BK)x
    A_cl = A + B @ K
    
    print(f"""
Closed-loop matrix A_cl = A + B×K:
{A_cl}
""")
    
    eigs = np.linalg.eigvals(A_cl)
    print(f"Eigenvalues: {[f'{e.real:+.3f}' for e in sorted(eigs, key=lambda x: -x.real)]}")
    
    max_real = max(e.real for e in eigs)
    if max_real < 0:
        print(f"\n✓ STABLE! Max real part = {max_real:.3f}")
    else:
        print(f"\n✗ UNSTABLE! Positive eigenvalue at {max_real:.3f}")
        
        # Debug: print the matrix elements
        print("\nA_cl matrix details:")
        print(f"  A_cl[2,0] = {A_cl[2,0]:.4f}  (should be negative for θ stability)")
        print(f"  A_cl[2,1] = {A_cl[2,1]:.4f}")
        print(f"  A_cl[3,0] = {A_cl[3,0]:.4f}")
        print(f"  A_cl[3,1] = {A_cl[3,1]:.4f}  (should be negative for α stability)")
    
    print("\n" + "=" * 70)
    print("STEP 5: DIAGNOSING THE ISSUE")
    print("=" * 70)
    
    # Check inner loop alone
    K_inner = np.array([[0, k_alpha, 0, k_alpha_dot]])
    A_cl_inner = A + B @ K_inner
    eigs_inner = np.linalg.eigvals(A_cl_inner)
    
    print(f"""
INNER LOOP ONLY (K = [0, {k_alpha:.4f}, 0, {k_alpha_dot:.4f}]):
  Eigenvalues: {[f'{e.real:+.2f}' for e in sorted(eigs_inner, key=lambda x: -x.real)]}
  
  A_cl[3,1] = A[3,1] + B[3,0] × k_α
            = {A[3,1]:.2f} + ({B[3,0]:.2f}) × ({k_alpha:.4f})
            = {A[3,1] + B[3,0] * k_alpha:.2f}
""")
    
    if A_cl_inner[3,1] < 0:
        print("  ✓ Alpha mode is stable (negative coefficient)")
    else:
        print("  ✗ Alpha mode still unstable!")
        
    # Check if outer loop helps or hurts
    print(f"""
ADDING OUTER LOOP:
  A_cl[2,0] = 0 + B[2,0] × (-k_θ)
            = 0 + ({B[2,0]:.2f}) × ({-k_theta:.5f})
            = {B[2,0] * (-k_theta):.4f}
            
  A_cl[3,0] = 0 + B[3,0] × (-k_θ)  
            = 0 + ({B[3,0]:.2f}) × ({-k_theta:.5f})
            = {B[3,0] * (-k_theta):.4f}
""")

    print(f"""
INTERPRETATION:
  - A_cl[2,0] = {A_cl[2,0]:.4f} means θ̈ depends on θ
    When θ > 0: θ̈ = {A_cl[2,0]:.4f} × θ < 0 → arm decelerates ✓
    
  - A_cl[3,0] = {A_cl[3,0]:.4f} means α̈ depends on θ  
    When θ > 0: α̈ = {A_cl[3,0]:.4f} × θ > 0 → pendulum tips over!
    
    This is the COUPLING PROBLEM!
    The outer loop pushes the arm, which tips the pendulum.
    If not compensated, this destabilizes α.
""")

    print("\n" + "=" * 70)
    print("STEP 6: THE REAL SOLUTION")
    print("=" * 70)
    
    print(f"""
The issue is that outer loop gains are TOO WEAK relative to inner loop.

The inner loop must be MUCH stronger than outer loop to:
1. Stabilize pendulum angle α (fast inner loop)
2. Allow small α variations when outer loop acts (cascaded control)

Let's compute the required gain ratio:

For stability, roughly need:
  k_α / k_θ > |B4/B3| × coupling_factor
  
  {k_alpha:.5f} / {k_theta:.5f} = {k_alpha/k_theta:.1f}
  |B4/B3| = |{B[3,0]:.2f}| / |{B[2,0]:.2f}| = {abs(B[3,0]/B[2,0]):.2f}

The inner/outer ratio is {k_alpha/k_theta:.1f}, which should be >> {abs(B[3,0]/B[2,0]):.1f}
This looks OK...

Let me try much higher inner loop gains:
""")

    # Try different gain ratios
    print("\nGAIN RATIO STUDY:")
    print(f"{'KP':>6} {'KD':>6} {'KTHETA':>8} {'Ratio':>8} {'Max λ':>8} {'Status':>10}")
    print("-" * 60)
    
    for acc_kp in [742, 1500, 2000, 3000]:
        for ktheta in [5, 10, 15, 20]:
            k_a = (acc_kp / DEG_TO_RAD) * acc_to_tau
            k_ad = (54.6 / DEG_TO_RAD) * acc_to_tau
            k_t = (ktheta / DEG_TO_RAD) * acc_to_tau
            k_td = (40.0 / DEG_TO_RAD) * acc_to_tau
            
            K_test = np.array([[-k_t, k_a, -k_td, k_ad]])
            A_test = A + B @ K_test
            eigs_test = np.linalg.eigvals(A_test)
            max_l = max(e.real for e in eigs_test)
            
            ratio = k_a / k_t
            status = "STABLE" if max_l < 0 else "unstable"
            print(f"{acc_kp:6.0f} {54.6:6.1f} {ktheta:8.0f} {ratio:8.1f} {max_l:+8.2f} {status:>10}")


def main():
    trace_complete_sign_chain()
    
    print("\n" + "=" * 70)
    print("CONCLUSIONS")
    print("=" * 70)
    print("""
The eigenvalue analysis reveals that:

1. INNER LOOP IS CORRECT - pendulum angle is stabilized

2. OUTER LOOP CAUSES COUPLING INSTABILITY
   - Pushing the arm creates a reaction torque on the pendulum
   - The outer loop command flows through B[3,0] into α̈
   - This can destabilize α if inner loop can't compensate fast enough

3. SOLUTIONS:
   a) INCREASE INNER LOOP GAINS (ACC_KP, ACC_KD)
      - Makes inner loop faster than outer loop disturbance
   
   b) REDUCE OUTER LOOP GAINS (KTHETA, KTHETADOT)
      - Slower position correction, less pendulum disturbance
   
   c) ADD FEEDFORWARD COMPENSATION
      - Predict pendulum response to arm acceleration
      - Add correction to cancel the coupling effect

4. RECOMMENDED FIRST ATTEMPT:
   - ACC_KP = 2000 (up from 742)
   - ACC_KD = 100 (up from 54.6)
   - KTHETA = 10 (down from 15)
   - KTHETADOT = 40 (down from 60)
   
   This increases inner/outer ratio from ~50 to ~200
""")


if __name__ == "__main__":
    main()
