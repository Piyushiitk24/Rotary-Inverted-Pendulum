#!/usr/bin/env python3
"""
Deep sign analysis for Furuta pendulum controller.
Traces through the entire sign chain to find the root cause of instability.
"""

import numpy as np
from furuta_model import FurutaParams, FurutaDynamics
from controller import FurutaController, ControllerParams
from simulator import FurutaSimulator, SimulationConfig

def main():
    model = FurutaDynamics()
    p = model.p

    print("="*60)
    print("SIGN ANALYSIS - Finding the Root Cause")
    print("="*60)

    # Test: pendulum tilted at alpha = +5 deg from upright
    # What acceleration does it experience with tau = 0?
    alpha_test = np.radians(5)
    state = np.array([0, alpha_test, 0, 0])
    theta_ddot, alpha_ddot = model.compute_accelerations(state, 0)

    print(f"\nWith alpha = +5 deg (tilted), tau = 0:")
    print(f"  alpha_ddot = {np.degrees(alpha_ddot):+.1f} deg/s^2")
    print(f"  (Pendulum accelerates AWAY from upright - this is unstable behavior)")

    print("\n" + "-"*60)
    print("TRACING THE CONTROLLER-TO-PHYSICS CHAIN")
    print("-"*60)

    # The controller outputs accCmd in steps/s^2
    # This is fed to the physics as TORQUE
    # The conversion is: tau = J0 * (accCmd / steps_per_rad)

    KP = 742  # steps/s^2/deg
    alpha_deg = 5
    accCmd = KP * alpha_deg
    print(f"\nController output: accCmd = KP * alpha = {KP} * {alpha_deg} = {accCmd} steps/s^2")

    # Convert to rad/s^2
    acc_rad = accCmd / p.steps_per_rad
    print(f"  theta_ddot_cmd = {acc_rad:.2f} rad/s^2")

    # Convert to torque
    tau = p.J_hat_0 * acc_rad
    print(f"  tau = J0 * theta_ddot_cmd = {tau:.6f} N*m")

    # Now compute what happens with this torque
    state_with_tau = np.array([0, alpha_test, 0, 0])
    theta_ddot_actual, alpha_ddot_actual = model.compute_accelerations(state_with_tau, tau)

    print(f"\nWith tau = {tau:.6f} N*m applied:")
    print(f"  theta_ddot = {np.degrees(theta_ddot_actual):.1f} deg/s^2")
    print(f"  alpha_ddot = {np.degrees(alpha_ddot_actual):+.1f} deg/s^2")

    if alpha_ddot_actual > 0:
        print("\n  >>> PROBLEM: alpha_ddot is STILL POSITIVE!")
        print("  >>> The controller is making things WORSE, not better!")
        print("  >>> We need NEGATIVE alpha_ddot to return to upright.")
    else:
        print("\n  >>> Good: alpha_ddot is negative, pendulum returns to upright")

    print("\n" + "-"*60)
    print("CHECKING THE COUPLING SIGN")
    print("-"*60)

    # The linearized pendulum equation at small alpha:
    # K*theta_ddot + J1*alpha_ddot - G*alpha = 0
    # => alpha_ddot = (G*alpha - K*theta_ddot) / J1
    #
    # For positive alpha (tilted), G*alpha is positive (destabilizing)
    # For the controller to help, K*theta_ddot should be positive too
    # This means theta_ddot should be positive when alpha is positive

    # But wait - does positive torque give positive theta_ddot?
    print(f"\nLet's check: with positive tau, what is theta_ddot?")
    state_zero = np.array([0, 0, 0, 0])  # pendulum at upright
    theta_ddot_from_tau, _ = model.compute_accelerations(state_zero, +0.01)
    print(f"  With tau = +0.01 N*m (positive torque):")
    print(f"  theta_ddot = {np.degrees(theta_ddot_from_tau):.1f} deg/s^2")

    if theta_ddot_from_tau > 0:
        print("  => Positive torque gives positive theta_ddot (as expected)")
    else:
        print("  => WARNING: Positive torque gives NEGATIVE theta_ddot!")

    print("\n" + "-"*60)
    print("THE KEY INSIGHT")
    print("-"*60)

    # Looking at the pendulum equation more carefully:
    # From the nonlinear model, at small alpha:
    # alpha_ddot = (J0*G*alpha - K*tau) / det
    #
    # where det = J0*J1 - K^2 > 0
    #
    # For stability: we need alpha_ddot < 0 when alpha > 0
    # => J0*G*alpha - K*tau < 0
    # => tau > (J0*G/K) * alpha
    #
    # The SIGN of tau matters! We need POSITIVE tau when alpha > 0.

    det = p.J_hat_0 * p.J_1 - p.K**2
    min_tau_per_rad = p.J_hat_0 * p.G / p.K
    print(f"\nFor stability, when alpha > 0, we need:")
    print(f"  tau > {min_tau_per_rad:.4f} * alpha (in radians)")
    print(f"  At alpha = 5 deg = {alpha_test:.4f} rad:")
    print(f"  tau > {min_tau_per_rad * alpha_test:.6f} N*m")
    print(f"\n  Controller produces: tau = {tau:.6f} N*m")

    if tau > min_tau_per_rad * alpha_test:
        print("  => Controller torque is SUFFICIENT")
    else:
        print("  => Controller torque is INSUFFICIENT")

    # But the eigenvalue analysis showed instability...
    # Let me recalculate more carefully

    print("\n" + "-"*60)
    print("RECOMPUTING CLOSED-LOOP EIGENVALUES")
    print("-"*60)

    # State: x = [theta, alpha, theta_dot, alpha_dot]
    # Linearized open-loop:
    # theta_ddot = 0 (no restoring force on arm)
    # alpha_ddot = (J0*G/det)*alpha (unstable)

    # With controller: tau = k_alpha * alpha + k_alpha_dot * alpha_dot
    # (where k values are in N*m/rad)

    a_alpha_alpha = p.J_hat_0 * p.G / det  # coefficient of alpha in alpha_ddot
    b_theta_tau = p.J_1 / det  # coefficient of tau in theta_ddot
    b_alpha_tau = -p.K / det   # coefficient of tau in alpha_ddot

    print(f"\nOpen-loop coefficients:")
    print(f"  d(alpha_ddot)/d(alpha) = {a_alpha_alpha:.2f} rad/s^2 per rad")
    print(f"  d(theta_ddot)/d(tau) = {b_theta_tau:.2f} rad/s^2 per N*m")
    print(f"  d(alpha_ddot)/d(tau) = {b_alpha_tau:.2f} rad/s^2 per N*m")

    # Controller gains in physical units (N*m/rad)
    DEG_TO_RAD = np.pi / 180
    k_alpha = (KP / DEG_TO_RAD) / p.steps_per_rad * p.J_hat_0
    KD = 54.6
    k_alpha_dot = (KD / DEG_TO_RAD) / p.steps_per_rad * p.J_hat_0

    print(f"\nController gains (physical units):")
    print(f"  k_alpha = {k_alpha:.4f} N*m/rad")
    print(f"  k_alpha_dot = {k_alpha_dot:.4f} N*m/(rad/s)")

    # Closed-loop alpha dynamics:
    # alpha_ddot = a_alpha_alpha * alpha + b_alpha_tau * tau
    #            = a_alpha_alpha * alpha + b_alpha_tau * (k_alpha * alpha + k_alpha_dot * alpha_dot)
    #            = (a_alpha_alpha + b_alpha_tau * k_alpha) * alpha + b_alpha_tau * k_alpha_dot * alpha_dot

    closed_loop_coeff = a_alpha_alpha + b_alpha_tau * k_alpha
    print(f"\nClosed-loop alpha coefficient:")
    print(f"  open-loop: {a_alpha_alpha:.2f}")
    print(f"  control contribution: {b_alpha_tau * k_alpha:.2f}")
    print(f"  closed-loop: {closed_loop_coeff:.2f}")

    if closed_loop_coeff < 0:
        print("  => STABLE (negative coefficient)")
    else:
        print("  => UNSTABLE (positive coefficient)")

    # WAIT - b_alpha_tau is NEGATIVE!
    # So when we apply positive tau, alpha_ddot DECREASES (which is good!)
    # Let me verify...

    print("\n" + "="*60)
    print("DEBUGGING THE LINEARIZATION")
    print("="*60)

    # Get linearized matrices from model
    A, B = model.linearize()
    print(f"\nA matrix (from model):")
    print(A)
    print(f"\nB matrix (from model):")
    print(B.flatten())

    # The signs in the matrix should tell us what's happening
    print(f"\n  A[3,1] = d(alpha_ddot)/d(alpha) = {A[3,1]:.2f}")
    print(f"  B[3,0] = d(alpha_ddot)/d(tau) = {B[3,0]:.2f}")

    # For controller tau = k * alpha, closed loop:
    # A_cl[3,1] = A[3,1] - B[3,0] * k
    #
    # Wait, the sign depends on whether it's A - BK or A + BK!
    # In standard control: x_dot = Ax + Bu, u = -Kx
    # So x_dot = (A - BK)x
    #
    # Our controller: tau = +k * alpha (positive gain)
    # So it's A + BK, not A - BK!

    print("\n" + "-"*60)
    print("THE SIGN CONVENTION ISSUE")
    print("-"*60)
    print("\nIn the simulation, the controller outputs:")
    print("  tau = k * alpha (POSITIVE feedback)")
    print("\nFor NEGATIVE feedback, we need:")
    print("  tau = -k * alpha when alpha > 0 (push pendulum back)")
    print("\nBut wait - let's check what direction is 'back'...")

    # The pendulum equation: alpha_ddot = ... - (K/det) * tau
    # The NEGATIVE sign means: positive tau -> DECREASE in alpha_ddot
    # So positive tau DOES help stabilize!

    # Let me compute A + BK (positive feedback) vs A - BK (negative feedback)
    K_gain = np.array([[0, k_alpha, 0, k_alpha_dot]])

    A_cl_pos = A + B @ K_gain  # tau = +K*x (positive feedback)
    A_cl_neg = A - B @ K_gain  # tau = -K*x (negative feedback)

    eigs_pos = np.linalg.eigvals(A_cl_pos)
    eigs_neg = np.linalg.eigvals(A_cl_neg)

    print(f"\nWith tau = +K*x (controller as written):")
    print(f"  Eigenvalues: {[f'{e.real:.2f}' for e in eigs_pos]}")
    print(f"  Max real part: {max(e.real for e in eigs_pos):.2f}")

    print(f"\nWith tau = -K*x (negated):")
    print(f"  Eigenvalues: {[f'{e.real:.2f}' for e in eigs_neg]}")
    print(f"  Max real part: {max(e.real for e in eigs_neg):.2f}")

    if max(e.real for e in eigs_pos) < 0:
        print("\n=> Current controller (positive feedback) is STABLE")
    elif max(e.real for e in eigs_neg) < 0:
        print("\n=> NEED TO NEGATE THE CONTROLLER OUTPUT!")
    else:
        print("\n=> Neither works - check the model!")


if __name__ == "__main__":
    main()
