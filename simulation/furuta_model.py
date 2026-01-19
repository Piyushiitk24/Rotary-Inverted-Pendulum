"""
Furuta Pendulum (Rotary Inverted Pendulum) - Nonlinear Dynamics Model
======================================================================
This implements the EXACT nonlinear equations of motion derived from 
Lagrangian mechanics. No linearization - full coupled dynamics.

Equations from: tools/modelling_complete.md

State vector: x = [theta, alpha, theta_dot, alpha_dot]
  - theta: arm angle (rad), 0 = initial position
  - alpha: pendulum angle (rad), 0 = UPRIGHT (unstable equilibrium)
  - theta_dot: arm angular velocity (rad/s)
  - alpha_dot: pendulum angular velocity (rad/s)

Input: tau = motor torque (N·m)

Convention: alpha = 0 is UPRIGHT (unstable), alpha = π is DOWN (stable)
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional

@dataclass
class FurutaParams:
    """
    Physical parameters of the Furuta pendulum.
    Default values match the actual hardware from FreeCAD macros.
    
    Geometry from FreeCAD:
    - Rotary arm: 210mm plate, hub at 30mm from end, L_r=190mm to pendulum axis
    - Pendulum: L-shaped rod, 7.95mm diameter
      - Horizontal segment: 170mm (acts as hinge axle, passes through 688RS bearings)
      - Vertical segment: 120mm (swings with pendulum)
    - Sphere: 34mm diameter with through hole, mounted 86-120mm from hinge
    """
    # === ROTARY ARM (BASE) GEOMETRY ===
    # From RotaryArm_Base_M3_v4_SingleFrameHole.FCMacro
    plate_L: float = 0.210       # Plate length (m)
    plate_W: float = 0.032       # Plate width (m)
    plate_T: float = 0.006       # Plate thickness (m)
    hub_setback: float = 0.014   # Hub center from rounded end (m)
    end_R: float = 0.016         # End radius (m)
    hub_OD: float = 0.022        # Hub outer diameter (m)
    hub_H: float = 0.018         # Hub height (m)
    motor_shaft_d: float = 0.00515  # Motor shaft diameter (m)
    
    # Arm length: distance from motor axis to pendulum hinge axis
    L_r: float = 0.19            # Arm length (m) - effective pivot to pivot
    m_r: float = 0.051           # Arm mass (kg) - approximate for PLA/PETG print
    
    # === PENDULUM (L-ROD) GEOMETRY ===
    # From Pendulum_L_M4.FCMacro
    rod_d: float = 0.00795       # Pendulum rod diameter (m) - fits 688RS ID=8mm
    rod_len_x: float = 0.170     # Horizontal segment length (m) - hinge axle
    rod_len_z: float = 0.120     # Vertical segment length (m) - swinging part
    rod_total_len: float = 0.290 # Total rod length (m) = 170 + 120
    rod_mass: float = 0.0103     # Total L-rod mass (kg) = 10.3g
    
    # Magnet seat at horizontal end (for AS5600)
    mag_d: float = 0.00496       # Magnet diameter (m)
    mag_h: float = 0.00196       # Magnet height (m)
    
    # === SPHERE GEOMETRY ===
    sphere_d: float = 0.034      # Sphere diameter (m) = 34mm
    sphere_mass: float = 0.0077  # Sphere mass (kg) = 7.7g
    sphere_pos_min: float = 0.086  # Sphere covers rod from this point (m)
    sphere_pos_max: float = 0.120  # Sphere covers rod to this point (m)
    
    # === DERIVED PENDULUM PARAMETERS ===
    # These are calculated from geometry in __post_init__
    L_p: float = 0.120           # Vertical pendulum length (m) = rod_len_z
    m_p: float = 0.01196         # Total swinging mass (vertical rod + sphere) (kg)
    l_p: float = 0.0877          # Distance to pendulum COM from hinge (m)
    
    # === INERTIAS (computed from geometry) ===
    J_arm: float = 6.137e-4      # Arm inertia about motor axis (kg·m²)
    J_axle: float = 8.11e-5      # Horizontal axle contribution (kg·m²)
    J_1: float = 1.021e-4        # Pendulum inertia about hinge (kg·m²)
    
    # === MOTOR/DRIVER PARAMETERS ===
    steps_per_rev: int = 1600    # Microstepping (1/8 with 200 step motor)
    max_acc_steps: float = 200000  # Max acceleration (steps/s²)
    max_speed_steps: float = 8000  # Max speed (steps/s)
    
    # === FRICTION (tune experimentally) ===
    b_theta: float = 0.0         # Arm viscous friction (N·m·s/rad)
    b_alpha: float = 0.0         # Pendulum viscous friction (N·m·s/rad)
    
    # === PHYSICAL CONSTANTS ===
    g: float = 9.81              # Gravity (m/s²)
    
    def __post_init__(self):
        """
        Compute all derived quantities from the raw geometry.
        This recalculates everything from first principles using the
        actual dimensions from the FreeCAD macros.
        """
        # === STEP CONVERSIONS ===
        self.steps_per_deg = self.steps_per_rev / 360.0
        self.steps_per_rad = self.steps_per_rev / (2 * np.pi)
        
        # === PENDULUM MASS DISTRIBUTION ===
        # Linear density of the L-rod
        rod_lambda = self.rod_mass / self.rod_total_len  # kg/m
        
        # Horizontal segment mass (hinge axle - doesn't swing)
        self.m_h = rod_lambda * self.rod_len_x  # ~6.04g
        
        # Vertical segment mass (swings with pendulum)
        self.m_rod = rod_lambda * self.rod_len_z  # ~4.26g
        
        # Total swinging mass (vertical rod + sphere)
        self.m_p = self.m_rod + self.sphere_mass  # ~11.96g
        
        # === PENDULUM CENTER OF MASS ===
        # Distances from hinge axis along the vertical rod
        x_rod = self.rod_len_z / 2  # Rod COM at midpoint: 60mm
        x_sphere = (self.sphere_pos_min + self.sphere_pos_max) / 2  # Sphere COM: 103mm
        
        # Weighted average for pendulum COM
        self.l_p = (self.m_rod * x_rod + self.sphere_mass * x_sphere) / self.m_p
        
        # === PENDULUM INERTIA ABOUT HINGE ===
        # Vertical rod as uniform rod about one end
        I_rod_piv = (1/3) * self.m_rod * self.rod_len_z**2
        
        # Sphere as point mass at x_sphere (dominant term)
        I_sphere_piv = self.sphere_mass * x_sphere**2
        
        # Sphere's own rotational inertia about its COM (small correction)
        r_sphere = self.sphere_d / 2
        I_sphere_cm = (2/5) * self.sphere_mass * r_sphere**2
        
        # Total pendulum inertia about hinge (Ĵ₂ in the model)
        self.J_1 = I_rod_piv + I_sphere_piv + I_sphere_cm
        
        # === ARM INERTIA ===
        # Arm as slender rod about motor axis
        self.J_arm = (1/3) * self.m_r * self.L_r**2
        
        # Horizontal axle inertia about motor axis
        # The axle spans from r_a to r_b radially
        # Assuming axle goes from (L_r - rod_len_x) to L_r
        r_a = self.L_r - self.rod_len_x  # 0.02 m
        r_b = self.L_r  # 0.19 m
        self.J_axle = (self.m_h / self.rod_len_x) * (r_b**3 - r_a**3) / 3
        
        # Total arm-side yaw inertia (Ĵ₁ in the model)
        self.J_0 = self.J_arm + self.J_axle
        
        # === COMPOSITE INERTIAS AND COUPLING ===
        # Effective arm inertia including pendulum mass at arm tip
        self.J_hat_0 = self.J_0 + self.m_p * self.L_r**2
        
        # Coupling constant (K in the model)
        self.K = self.m_p * self.L_r * self.l_p
        
        # Gravity torque constant (G in the model)
        self.G = self.m_p * self.g * self.l_p
        
        # === MOTOR LIMITS ===
        max_acc_rad = self.max_acc_steps / self.steps_per_rad
        self.tau_max = self.J_hat_0 * max_acc_rad
        
        # === DYNAMICS CONSTANTS (for quick reference) ===
        # Natural frequency of pendulum (unstable pole)
        self.omega_n = np.sqrt(self.G / self.J_1)  # rad/s
        self.f_n = self.omega_n / (2 * np.pi)  # Hz
        self.tau_fall = 1 / self.omega_n  # Time constant for falling (s)


class FurutaDynamics:
    """
    Nonlinear dynamics of the Furuta pendulum.
    
    Implements the full coupled equations of motion:
    
    ARM EQUATION:
    (Ĵ₀ + Ĵ₂sin²α)θ̈ + K·cos(α)·α̈ + Ĵ₂·sin(2α)·θ̇·α̇ - K·sin(α)·α̇² = τ
    
    PENDULUM EQUATION:
    K·cos(α)·θ̈ + Ĵ₂·α̈ - ½Ĵ₂·sin(2α)·θ̇² - G·sin(α) = 0
    """
    
    def __init__(self, params: Optional[FurutaParams] = None):
        self.p = params if params else FurutaParams()
        
    def compute_accelerations(self, state: np.ndarray, tau: float) -> Tuple[float, float]:
        """
        Compute angular accelerations given current state and torque.
        
        Args:
            state: [theta, alpha, theta_dot, alpha_dot] in radians
            tau: Motor torque (N·m)
            
        Returns:
            (theta_ddot, alpha_ddot) in rad/s²
        """
        theta, alpha, theta_dot, alpha_dot = state
        p = self.p
        
        # Shorthand for trig functions
        sa = np.sin(alpha)
        ca = np.cos(alpha)
        s2a = np.sin(2 * alpha)
        
        # Mass matrix elements
        # M = [M11  M12]
        #     [M21  M22]
        M11 = p.J_hat_0 + p.J_1 * sa**2
        M12 = p.K * ca
        M21 = p.K * ca
        M22 = p.J_1
        
        # Coriolis/centrifugal and gravity terms
        # C1 = Ĵ₂·sin(2α)·θ̇·α̇ - K·sin(α)·α̇²
        C1 = p.J_1 * s2a * theta_dot * alpha_dot - p.K * sa * alpha_dot**2
        
        # C2 = -½Ĵ₂·sin(2α)·θ̇² - G·sin(α)
        C2 = -0.5 * p.J_1 * s2a * theta_dot**2 - p.G * sa
        
        # Add friction if present
        F1 = p.b_theta * theta_dot
        F2 = p.b_alpha * alpha_dot
        
        # Right-hand side
        # [τ - C1 - F1]
        # [  - C2 - F2]
        rhs1 = tau - C1 - F1
        rhs2 = -C2 - F2
        
        # Solve M * [θ̈; α̈] = rhs using 2x2 inverse
        det = M11 * M22 - M12 * M21
        
        # Prevent division by zero (shouldn't happen physically)
        if abs(det) < 1e-12:
            det = 1e-12 * np.sign(det) if det != 0 else 1e-12
        
        theta_ddot = (M22 * rhs1 - M12 * rhs2) / det
        alpha_ddot = (-M21 * rhs1 + M11 * rhs2) / det
        
        return theta_ddot, alpha_ddot
    
    def state_derivative(self, t: float, state: np.ndarray, tau_func) -> np.ndarray:
        """
        Compute dx/dt for ODE integration.
        
        Args:
            t: Time (s)
            state: [theta, alpha, theta_dot, alpha_dot]
            tau_func: Function that returns torque given (t, state)
            
        Returns:
            [theta_dot, alpha_dot, theta_ddot, alpha_ddot]
        """
        theta_dot = state[2]
        alpha_dot = state[3]
        
        tau = tau_func(t, state)
        theta_ddot, alpha_ddot = self.compute_accelerations(state, tau)
        
        return np.array([theta_dot, alpha_dot, theta_ddot, alpha_ddot])
    
    def linearize(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get linearized A, B matrices around upright equilibrium (α=0).
        
        Returns:
            A: 4x4 state matrix
            B: 4x1 input matrix
        
        Linearized model: ẋ = Ax + Bu
        where x = [θ, α, θ̇, α̇], u = τ
        """
        p = self.p
        
        # From the linearized equations:
        # Ĵ₀·θ̈ + K·α̈ = τ
        # K·θ̈ + Ĵ₂·α̈ - G·α = 0
        
        # Solve for accelerations:
        # [Ĵ₀  K ] [θ̈]   [τ  ]
        # [K   Ĵ₂] [α̈] = [G·α]
        
        det = p.J_hat_0 * p.J_1 - p.K**2
        
        # θ̈ = (Ĵ₂·τ - K·G·α) / det
        # α̈ = (-K·τ + Ĵ₀·G·α) / det
        
        # A matrix coefficients
        a_theta_ddot_alpha = -p.K * p.G / det
        a_alpha_ddot_alpha = p.J_hat_0 * p.G / det
        
        # B matrix coefficients  
        b_theta_ddot = p.J_1 / det
        b_alpha_ddot = -p.K / det
        
        A = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, a_theta_ddot_alpha, -p.b_theta/p.J_hat_0, 0],
            [0, a_alpha_ddot_alpha, 0, -p.b_alpha/p.J_1]
        ])
        
        B = np.array([
            [0],
            [0],
            [b_theta_ddot],
            [b_alpha_ddot]
        ])
        
        return A, B
    
    def get_open_loop_poles(self) -> np.ndarray:
        """Get eigenvalues of the linearized open-loop system."""
        A, _ = self.linearize()
        return np.linalg.eigvals(A)
    
    def natural_frequency(self) -> float:
        """
        Get the natural (unstable) frequency of the pendulum.
        ω = √(G/J₁) in rad/s
        """
        return np.sqrt(self.p.G / self.p.J_1)
    
    def fall_time_constant(self) -> float:
        """
        Time constant for pendulum to fall (1/λ where λ is unstable pole).
        """
        omega = self.natural_frequency()
        return 1.0 / omega


def test_model():
    """Quick sanity check of the model with all parameters from FreeCAD geometry."""
    params = FurutaParams()
    model = FurutaDynamics(params)
    
    print("=" * 70)
    print("FURUTA PENDULUM MODEL - HARDWARE PARAMETER VERIFICATION")
    print("=" * 70)
    
    print("\n" + "-" * 70)
    print("GEOMETRY FROM FREECAD MACROS")
    print("-" * 70)
    print(f"Rotary Arm Plate: {params.plate_L*1000:.1f} × {params.plate_W*1000:.1f} × {params.plate_T*1000:.1f} mm")
    print(f"Hub: OD={params.hub_OD*1000:.1f}mm, H={params.hub_H*1000:.1f}mm, setback={params.hub_setback*1000:.1f}mm")
    print(f"Motor shaft: d={params.motor_shaft_d*1000:.2f}mm")
    print()
    print(f"Pendulum L-Rod: d={params.rod_d*1000:.2f}mm (fits 688RS bearing ID=8mm)")
    print(f"  Horizontal (axle): {params.rod_len_x*1000:.1f}mm")
    print(f"  Vertical (swing):  {params.rod_len_z*1000:.1f}mm")
    print(f"  Total length:      {params.rod_total_len*1000:.1f}mm")
    print(f"  Total mass:        {params.rod_mass*1000:.1f}g")
    print()
    print(f"Sphere: d={params.sphere_d*1000:.1f}mm, mass={params.sphere_mass*1000:.1f}g")
    print(f"  Mounted at {params.sphere_pos_min*1000:.0f}-{params.sphere_pos_max*1000:.0f}mm from hinge")
    
    print("\n" + "-" * 70)
    print("COMPUTED MASS DISTRIBUTION")
    print("-" * 70)
    rod_lambda = params.rod_mass / params.rod_total_len * 1000  # g/mm
    print(f"Rod linear density: λ = {rod_lambda:.4f} g/mm")
    print(f"Horizontal segment mass: m_h = {params.m_h*1000:.2f} g")
    print(f"Vertical segment mass: m_rod = {params.m_rod*1000:.2f} g")
    print(f"Sphere mass: m_s = {params.sphere_mass*1000:.2f} g")
    print(f"Total swinging mass: m_p = {params.m_p*1000:.2f} g")
    print()
    print(f"Vertical rod COM: {params.rod_len_z/2*1000:.1f} mm from hinge")
    print(f"Sphere COM: {(params.sphere_pos_min+params.sphere_pos_max)/2*1000:.1f} mm from hinge")
    print(f"Pendulum COM: l_p = {params.l_p*1000:.2f} mm from hinge")
    
    print("\n" + "-" * 70)
    print("COMPUTED INERTIAS")
    print("-" * 70)
    print(f"Arm inertia (rod about end): J_arm = {params.J_arm:.6e} kg·m²")
    print(f"Horizontal axle inertia:     J_axle = {params.J_axle:.6e} kg·m²")
    print(f"Total arm-side yaw inertia:  J_0 = {params.J_0:.6e} kg·m²")
    print(f"Effective arm inertia:       Ĵ_0 = {params.J_hat_0:.6e} kg·m²")
    print()
    print(f"Pendulum inertia about hinge: Ĵ_1 = {params.J_1:.6e} kg·m²")
    print(f"Coupling constant:            K = {params.K:.6e} kg·m²")
    print(f"Gravity constant:             G = {params.G:.6e} N·m")
    
    print("\n" + "-" * 70)
    print("DYNAMICS CHARACTERISTICS")
    print("-" * 70)
    print(f"Natural frequency: ω_n = {params.omega_n:.2f} rad/s = {params.f_n:.2f} Hz")
    print(f"Fall time constant: τ_fall = {params.tau_fall*1000:.1f} ms")
    print(f"Open-loop poles: {model.get_open_loop_poles()}")
    
    print("\n" + "-" * 70)
    print("MOTOR PARAMETERS")
    print("-" * 70)
    print(f"Steps/rev: {params.steps_per_rev} (1/8 microstepping)")
    print(f"Steps/deg: {params.steps_per_deg:.4f}")
    print(f"Steps/rad: {params.steps_per_rad:.2f}")
    print(f"Max acceleration: {params.max_acc_steps:.0f} steps/s²")
    print(f"Max speed: {params.max_speed_steps:.0f} steps/s")
    print(f"Max torque (approx): {params.tau_max*1000:.2f} mN·m")
    
    # Test acceleration at α = 5° with no torque
    print("\n" + "-" * 70)
    print("SANITY CHECK: Free pendulum response")
    print("-" * 70)
    alpha_test = np.radians(5)
    state = np.array([0, alpha_test, 0, 0])
    theta_ddot, alpha_ddot = model.compute_accelerations(state, 0)
    
    print(f"At α = 5°, τ = 0:")
    print(f"  θ̈ = {np.degrees(theta_ddot):+.1f} °/s²")
    print(f"  α̈ = {np.degrees(alpha_ddot):+.1f} °/s²")
    if alpha_ddot > 0:
        print("  ✓ Pendulum accelerates AWAY from upright (unstable equilibrium)")
    else:
        print("  ✗ ERROR: Pendulum should accelerate away from upright!")
    
    print("\n" + "=" * 70)
    print("Model verification complete.")
    print("=" * 70)
    
    return model


if __name__ == "__main__":
    test_model()
