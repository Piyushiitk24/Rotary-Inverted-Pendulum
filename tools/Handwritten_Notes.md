# Rotary Inverted Pendulum: Complete Mathematical Derivation

## 1. Physical Parameters & Component Analysis

### 1.1 Geometry & Material Properties
**System Description:** L-shaped Rod with a Sphere at the tip.
* **Total Length ($L_{total}$):** $290 \text{ mm}$ ($170 \text{ mm} \text{ horizontal} + 120 \text{ mm} \text{ vertical}$)
* **Total Mass ($m_{total}$):** $10.3 \text{ g}$
* **Linear Density ($\lambda$):**
    $$\lambda = \frac{10.3}{290} \approx 0.03552 \text{ g/mm}$$

### 1.2 Component Breakdown
**Horizontal Segment (Rotary Arm):**
* **Length ($L_H$):** $170 \text{ mm}$
* **Mass ($m_H$):** $170 \times 0.03552 = 6.04 \text{ g}$

**Vertical Segment (Pendulum Rod):**
* **Length ($L_V$):** $120 \text{ mm}$
* **Mass ($m_V$):** $120 \times 0.03552 = 4.26 \text{ g} = 0.00426 \text{ kg}$

**Sphere (Tip Mass):**
* **Mass ($m_{sphere}$):** $7.7 \text{ g} = 0.0077 \text{ kg}$
* **Distance from Pivot ($r_{sphere}$):** $120 \text{ mm} - 17 \text{ mm} = 103 \text{ mm} = 0.103 \text{ m}$

### 1.3 Swinging Assembly Properties (The Pendulum)
**Total Swinging Mass ($m_p$):**
$$m_p = m_{rod} + m_{sphere} = 0.00426 + 0.0077 = 0.01196 \text{ kg}$$

**Center of Mass ($l_p$):**
* Rod CoM: $0.120 / 2 = 0.06 \text{ m}$
* Sphere CoM: $0.103 \text{ m}$
$$l_p = \frac{(m_{rod} \cdot 0.06) + (m_{sphere} \cdot 0.103)}{m_p}$$
$$l_p = \frac{0.0002556 + 0.0007931}{0.01196} \approx 0.0877 \text{ m}$$

**Moment of Inertia about Pivot ($J_1$):**
1.  **Rod Inertia ($I_{rod}$):**
    $$I_{rod} = \frac{1}{3} m L^2 = \frac{1}{3}(0.00426)(0.120)^2 = 2.045 \times 10^{-5} \text{ kg}\cdot\text{m}^2$$
2.  **Sphere Inertia ($I_{sphere}$):**
    $$I_{sphere} = m r^2 = (0.0077)(0.103)^2 = 8.169 \times 10^{-5} \text{ kg}\cdot\text{m}^2$$
3.  **Total ($J_1$):**
    $$J_1 = I_{rod} + I_{sphere} = 1.021 \times 10^{-4} \text{ kg}\cdot\text{m}^2$$

**Rotary Arm Inertia ($J_0$):**
Calculated as the sum of arm base inertia, horizontal rod inertia, and the pendulum mass acting as a point mass at radius $L_r$.
$$J_0 = 0.001104 \text{ kg}\cdot\text{m}^2$$

---

## 2. Lagrangian Dynamics

### 2.1 Coordinate System
* $\theta(t)$: Rotary Arm Angle (Actuated)
* $\alpha(t)$: Pendulum Angle (Unactuated, $0 = \text{Upright}$)

### 2.2 Kinetic Energy ($T$)
The kinetic energy ($T$) comprises the arm's rotation, the pendulum's rotation, and the translation velocity due to the arm.
$$T = \frac{1}{2}(J_0 + J_1 \sin^2\alpha)\dot{\theta}^2 + \frac{1}{2}J_1 \dot{\alpha}^2 + m_p L_r l_p \cos\alpha \dot{\theta} \dot{\alpha}$$

### 2.3 Potential Energy ($V$)
Depends on the vertical height $z$ of the pendulum's center of mass ($l_p$).
$$V = m_p g l_p \cos\alpha$$

### 2.4 Lagrangian ($L$)
$$L = T - V$$
$$L = \frac{1}{2}(J_0 + J_1 \sin^2\alpha)\dot{\theta}^2 + \frac{1}{2}J_1 \dot{\alpha}^2 + m_p L_r l_p \cos\alpha \dot{\theta} \dot{\alpha} - m_p g l_p \cos\alpha$$

---

## 3. Derivation of Equations of Motion (Euler-Lagrange)

General form:
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}}\right) - \frac{\partial L}{\partial q} = Q$$

### 3.1 Pendulum Equation ($q = \alpha$)
1.  **Partial w.r.t $\dot{\alpha}$:**
    $$\frac{\partial L}{\partial \dot{\alpha}} = J_1 \dot{\alpha} + m_p L_r l_p \cos\alpha \dot{\theta}$$
2.  **Time Derivative:**
    $$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\alpha}}\right) = J_1 \ddot{\alpha} + m_p L_r l_p (\cos\alpha \ddot{\theta} - \sin\alpha \dot{\alpha} \dot{\theta})$$
3.  **Partial w.r.t $\alpha$:**
    $$\frac{\partial L}{\partial \alpha} = \frac{1}{2} J_1 (2 \sin\alpha \cos\alpha) \dot{\theta}^2 - m_p L_r l_p \sin\alpha \dot{\theta} \dot{\alpha} + m_p g l_p \sin\alpha$$
4.  **Equation of Motion:**
    $$J_1 \ddot{\alpha} + m_p L_r l_p \cos\alpha \ddot{\theta} - \frac{1}{2} J_1 \sin(2\alpha)\dot{\theta}^2 - m_p g l_p \sin\alpha = 0$$

### 3.2 Rotary Arm Equation ($q = \theta$)
1.  **Partial w.r.t $\dot{\theta}$:**
    $$\frac{\partial L}{\partial \dot{\theta}} = (J_0 + J_1 \sin^2\alpha)\dot{\theta} + m_p L_r l_p \cos\alpha \dot{\alpha}$$
2.  **Time Derivative:**
    $$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}}\right) = (J_0 + J_1 \sin^2\alpha)\ddot{\theta} + J_1 \sin(2\alpha)\dot{\alpha}\dot{\theta} + m_p L_r l_p \cos\alpha \ddot{\alpha} - m_p L_r l_p \sin\alpha \dot{\alpha}^2$$
3.  **Partial w.r.t $\theta$:**
    $$\frac{\partial L}{\partial \theta} = 0$$
4.  **Equation of Motion:**
    $$(J_0 + J_1 \sin^2\alpha)\ddot{\theta} + m_p L_r l_p \cos\alpha \ddot{\alpha} + J_1 \sin(2\alpha)\dot{\alpha}\dot{\theta} - m_p L_r l_p \sin\alpha \dot{\alpha}^2 = \tau$$

---

## 4. Linearization & State Space

### 4.1 Small Angle Approximations ($\alpha \approx 0$)
* $\cos\alpha \approx 1$
* $\sin\alpha \approx \alpha$
* $\sin(2\alpha) \approx 2\alpha$
* $\dot{\theta}^2 \approx 0$, $\dot{\alpha}^2 \approx 0$ (Small velocities)

### 4.2 Linearized Equations
1.  **Arm:** $J_0 \ddot{\theta} + (m_p L_r l_p) \ddot{\alpha} = \tau$
2.  **Pendulum:** $(m_p L_r l_p) \ddot{\theta} + J_1 \ddot{\alpha} - (m_p g l_p) \alpha = 0$

**Defined Coefficients:**
* $K = m_p L_r l_p$ (Coupling Inertia)
* $G = m_p g l_p$ (Gravity Stiffness)

### 4.3 Matrix Representation
$$\begin{bmatrix} J_0 & K \\ K & J_1 \end{bmatrix} \begin{bmatrix} \ddot{\theta} \\ \ddot{\alpha} \end{bmatrix} + \begin{bmatrix} 0 & 0 \\ 0 & -G \end{bmatrix} \begin{bmatrix} \theta \\ \alpha \end{bmatrix} = \begin{bmatrix} \tau \\ 0 \end{bmatrix}$$

**Numerical Values:**
$$\begin{bmatrix} 0.001104 & 0.0001993 \\ 0.0001993 & 0.0001021 \end{bmatrix} \begin{bmatrix} \ddot{\theta} \\ \ddot{\alpha} \end{bmatrix} + \begin{bmatrix} 0 & 0 \\ 0 & -0.01029 \end{bmatrix} \begin{bmatrix} \theta \\ \alpha \end{bmatrix} = \begin{bmatrix} \tau \\ 0 \end{bmatrix}$$

---

## 5. System Analysis

### 5.1 Open Loop Stability (Fall Dynamics)
Setting $\tau = 0$ to find natural dynamics.
From Arm Equation:
$$\ddot{\theta} = -\frac{K}{J_0}\ddot{\alpha}$$
Substitute into Pendulum Equation:
$$K\left(-\frac{K}{J_0}\ddot{\alpha}\right) + J_1 \ddot{\alpha} - G\alpha = 0$$
$$\left( J_1 - \frac{K^2}{J_0} \right) \ddot{\alpha} - G\alpha = 0$$

**Equivalent Inertia ($J_{eq}$):**
$$J_{eq} = J_1 - \frac{K^2}{J_0} = 0.0001021 - \frac{(0.0001993)^2}{0.001104}$$
$$J_{eq} = 6.612 \times 10^{-5} \text{ kg}\cdot\text{m}^2$$

**Eigenvalues ($\lambda$):**
$$\ddot{\alpha} = \frac{G}{J_{eq}}\alpha \quad \Rightarrow \quad \lambda^2 = \frac{G}{J_{eq}}$$
$$\lambda = \pm \sqrt{\frac{0.01029}{6.612 \times 10^{-5}}} = \pm \sqrt{155.62}$$
$$\lambda = \pm 12.47 \text{ rad/s}$$

* **Time Constant:** $\tau_{fall} \approx 0.080 \text{ s}$
* **Natural Frequency:** $f \approx 2.0 \text{ Hz}$

### 5.2 Input Effectiveness Calculation (Matrix Inverse)
To find accelerations explicitly:
$$\begin{bmatrix} \ddot{\theta} \\ \ddot{\alpha} \end{bmatrix} = M^{-1} \begin{bmatrix} \tau \\ G\alpha \end{bmatrix}$$
Determinant $\Delta = J_0 J_1 - K^2 = 7.2998 \times 10^{-8}$.
$$M^{-1} = \frac{1}{\Delta} \begin{bmatrix} J_1 & -K \\ -K & J_0 \end{bmatrix} = \begin{bmatrix} 1398.7 & -2730.2 \\ -2730.2 & 15123.7 \end{bmatrix}$$
This shows significant control authority ($15123.7$ term).

### 5.3 Torque Requirements
Approximation for coupling effectiveness ($\ddot{\alpha} \approx 0$):
$$\ddot{\theta} \approx \frac{G}{K} \alpha = 51.63 \alpha$$
Torque required for max recovery ($20,000 \text{ steps/s}^2$):
$$\tau_{max} \approx J_{eff} \ddot{\theta}_{max} \approx 0.0562 \text{ Nm}$$
**Result:** NEMA 17 is sufficient (Rated 0.2 - 0.4 Nm).

---

## 6. Control Law Design

### 6.1 Plant Model
Isolate $\ddot{\alpha}$:
$$\ddot{\alpha} = \frac{G}{J_1}\alpha - \frac{K}{J_1}\ddot{\theta}$$
Let $A = \frac{G}{J_1} = 100.8 \text{ s}^{-2}$ and $B = \frac{K}{J_1} = 1.952$.
$$\ddot{\alpha} = A\alpha - B\ddot{\theta}$$

### 6.2 PD Control Structure
Command arm acceleration ($\ddot{\theta}_{cmd}$) to stabilize $\alpha$:
$$\ddot{\theta}_{cmd} = -k_p \alpha - k_d \dot{\alpha}$$
Substituting into plant:
$$\ddot{\alpha} = A\alpha - B(-k_p \alpha - k_d \dot{\alpha})$$
$$\ddot{\alpha} - (B k_d)\dot{\alpha} - (A + B k_p)\alpha = 0$$

### 6.3 Pole Placement Tuning
Target 2nd order system: $\ddot{\alpha} + 2\zeta\omega_c \dot{\alpha} + \omega_c^2 \alpha = 0$

**Coefficients Matching:**
1.  **Damping Term:**
    $$-B k_d = 2\zeta\omega_c \quad \Rightarrow \quad k_d = -\frac{2\zeta\omega_c}{B}$$
2.  **Stiffness Term:**
    $$-(A + B k_p) = \omega_c^2 \quad \Rightarrow \quad k_p = -\frac{A + \omega_c^2}{B}$$

**Design Choices:**
* Response Speed ($\omega_c$): $15 \text{ rad/s}$
* Damping Ratio ($\zeta$): $0.8$

**Calculated Gains (Physical):**
$$k_p = -\frac{100.8 + 15^2}{1.952} = -166.9 \text{ rad/s}^2/\text{rad}$$
$$k_d = -\frac{2(0.8)(15)}{1.952} = -12.3 \text{ rad/s}^2/(\text{rad/s})$$

---

## 7. Implementation Gains (Step Units)

### 7.1 Unit Conversions
* **Steps per Revolution:** 1600 (1/8 microstepping)
* **Steps per Degree:** $1600 / 360 = 4.444$
* **Steps per Radian:** $1600 / 2\pi \approx 254.65$

### 7.2 Final Firmware Equation
Convert physical gains to step-based gains for the microcontroller.
$$\ddot{\theta}_{steps} = -(k_p \cdot \text{steps\_per\_rad} \cdot \alpha_{rad} + k_d \cdot \text{steps\_per\_rad} \cdot \dot{\alpha}_{rad})$$
Or using degrees directly as input:
$$\ddot{\theta}_{steps} = -(742 \cdot \alpha_{deg} + 54.6 \cdot \dot{\alpha}_{deg})$$