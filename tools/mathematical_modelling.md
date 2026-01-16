# Mathematical Modelling of the Rotary Inverted Pendulum (Furuta)
*Complete Derivation from First Principles (Lagrangian Mechanics)*

## 1. System Definition & Coordinates

We define the system using two generalized coordinates:
1.  **$\theta(t)$**: The angle of the Rotary Arm (Horizontal Plane). $\theta = 0$ corresponds to the X-axis.
2.  **$\alpha(t)$**: The angle of the Pendulum (Vertical Plane). $\alpha = 0$ corresponds to the perfectly upright (unstable) position.

### Frames of Reference
* **Inertial Frame ($O$):** Fixed at the base motor shaft. Z-axis is vertical (gravity).
* **Arm Frame ($A$):** Rotates with angle $\theta$ about the Z-axis.
* **Pendulum Frame ($P$):** Attached to the end of the arm, rotating with angle $\alpha$.

---

## 2. Parameter Calculation (Step-by-Step)

We calculate the exact physical properties of your hardware: **L-Rod (120mm Vertical) + Sphere (7.7g)**.

### 2.1 Component Analysis

**A. The L-Rod (Split Calculation)**
* **Total Mass:** 10.3 g
* **Total Length:** $170\text{ mm} + 120\text{ mm} = 290\text{ mm}$
* **Linear Density ($\lambda$):**
    $$\lambda = \frac{10.3}{290} \approx 0.03552 \text{ g/mm}$$

We must split the rod because the horizontal part acts as part of the *Arm*, while the vertical part acts as the *Pendulum*.

1.  **Horizontal Segment ($L_h = 170$ mm):**
    * Mass ($m_h$) = $170 \times 0.03552 = 6.04 \text{ g} = \mathbf{0.00604 \text{ kg}}$.
    * *Role:* Increases the inertia of the Rotary Arm ($J_0$). Does not swing.
2.  **Vertical Segment ($L_v = 120$ mm):**
    * Mass ($m_{rod}$) = $120 \times 0.03552 = 4.26 \text{ g} = \mathbf{0.00426 \text{ kg}}$.
    * *Role:* This is the pendulum body.

**B. The Sphere**
* **Mass ($m_{sphere}$):** 7.7 g = $\mathbf{0.0077 \text{ kg}}$.
* **Geometry:**
    * Rod Length = 120 mm.
    * Sphere Radius = 17 mm.
    * Center of Mass Position ($r_{sphere}$): Since the sphere is attached at the end, its center is at $120 - 17 = 103 \text{ mm}$.
    * $r_{sphere} = \mathbf{0.103 \text{ m}}$.

---

### 2.2 Combined Swinging Mass Properties ($m_p$, $l_p$, $J_1$)

We treat the Vertical Rod + Sphere as one rigid body.

**A. Total Swinging Mass ($m_p$)**
$$m_p = m_{rod} + m_{sphere}$$
$$m_p = 0.00426 + 0.0077 = \mathbf{0.01196 \text{ kg}}$$

**B. Center of Mass Distance ($l_p$)**
This is the weighted average distance from the pivot.
* Rod CoM: Midpoint of 120mm $\rightarrow$ 60mm ($0.06$ m).
* Sphere CoM: 103mm ($0.103$ m).

$$l_p = \frac{(m_{rod} \times 0.06) + (m_{sphere} \times 0.103)}{m_p}$$
$$l_p = \frac{(0.00426 \times 0.06) + (0.0077 \times 0.103)}{0.01196}$$
$$l_p = \frac{0.0002556 + 0.0007931}{0.01196}$$
$$l_p = \frac{0.0010487}{0.01196} \approx \mathbf{0.0877 \text{ m}}$$

**C. Moment of Inertia about Pivot ($J_1$)**
Resistance to swinging.
* **Rod Part ($I = \frac{1}{3}mL^2$):**
    $$I_{rod} = \frac{1}{3} \times 0.00426 \times (0.120)^2 = 2.045 \times 10^{-5}$$
* **Sphere Part ($I = mr^2$):**
    $$I_{sphere} = 0.0077 \times (0.103)^2 = 8.169 \times 10^{-5}$$

$$J_1 = 2.045 \times 10^{-5} + 8.169 \times 10^{-5} = \mathbf{1.021 \times 10^{-4} \text{ kg}\cdot\text{m}^2}$$

---

### 2.3 Rotary Arm Inertia ($J_0$)
Resistance to spinning the base motor.
$J_0 = J_{arm\_base} + J_{horiz\_rod} + J_{pendulum\_point}$

1.  **Base Arm:** Calculated previously as $\mathbf{0.0006137}$.
2.  **Horizontal Rod ($L=170mm$):**
    $$I_{h\_rod} = \frac{1}{3} m L^2 = \frac{1}{3} \times 0.00604 \times (0.17)^2 = 0.0000582$$
3.  **Pendulum Mass at Arm Tip:**
    The entire swinging mass ($m_p$) sits at the end of the arm ($L_r = 0.19m$).
    $$I_{tip} = m_p L_r^2 = 0.01196 \times (0.19)^2 = 0.0004318$$

$$J_0 = 0.0006137 + 0.0000582 + 0.0004318 = \mathbf{0.001104 \text{ kg}\cdot\text{m}^2}$$

---

## 3. The Lagrangian Function

We calculate the total Energy of the system: $L = T - V$.

**Kinetic Energy ($T$):**
$T$ is composed of three terms:
1.  **Arm Rotation:** $\frac{1}{2} J_0 \dot{\theta}^2$
2.  **Pendulum Swing:** $\frac{1}{2} J_1 \dot{\alpha}^2$
3.  **Interaction (Coupling):** When the pendulum is not vertical ($\alpha \neq 0$), its inertia contributes to the arm's load. The term $J_1 \sin^2\alpha \dot{\theta}^2$ represents the "flywheel effect" of the pendulum swinging out. The term $m_p L_r l_p \cos\alpha \dot{\theta} \dot{\alpha}$ represents the velocity coupling.

**Full Kinetic Energy Equation:**
$$T = \frac{1}{2} (J_0 + J_1 \sin^2\alpha) \dot{\theta}^2 + \frac{1}{2} J_1 \dot{\alpha}^2 + (m_p L_r l_p \cos\alpha) \dot{\theta} \dot{\alpha}$$

**Potential Energy ($V$):**
Only gravity acting on the pendulum height.
$$V = m_p g l_p \cos\alpha$$

**The Lagrangian ($L = T - V$):**
$$L = \frac{1}{2} (J_0 + J_1 \sin^2\alpha) \dot{\theta}^2 + \frac{1}{2} J_1 \dot{\alpha}^2 + (K \cos\alpha) \dot{\theta} \dot{\alpha} - G_{grav} \cos\alpha$$

*Definitions for simplification:*
* $K = m_p L_r l_p$ (Coupling Constant)
* $G_{grav} = m_p g l_p$ (Gravity Constant)

---

## 4. Derivation of Equation 1: Pendulum Dynamics ($\alpha$)

We apply the Euler-Lagrange formula for coordinate $\alpha$:
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\alpha}} \right) - \frac{\partial L}{\partial \alpha} = 0$$

**Step 1: Partial Derivative w.r.t Velocity ($\dot{\alpha}$)**
Looking at $L$, only the 2nd and 3rd terms have $\dot{\alpha}$.
$$\frac{\partial L}{\partial \dot{\alpha}} = J_1 \dot{\alpha} + K \cos\alpha \dot{\theta}$$

**Step 2: Time Derivative ($\frac{d}{dt}$)**
We differentiate the result of Step 1 with respect to time.
* $\frac{d}{dt}(J_1 \dot{\alpha}) = J_1 \ddot{\alpha}$
* $\frac{d}{dt}(K \cos\alpha \dot{\theta})$ requires the **Product Rule** ($uv' + u'v$).
    * $u = \cos\alpha$, $v = \dot{\theta}$
    * $\frac{d}{dt} = K (\cos\alpha \ddot{\theta} - \sin\alpha \dot{\alpha} \dot{\theta})$

$$Result_A = J_1 \ddot{\alpha} + K \cos\alpha \ddot{\theta} - K \sin\alpha \dot{\alpha} \dot{\theta}$$

**Step 3: Partial Derivative w.r.t Position ($\alpha$)**
We look at $L$ again for terms containing $\alpha$.
* Term 1: $\frac{1}{2} J_1 \sin^2\alpha \dot{\theta}^2 \rightarrow \frac{1}{2} J_1 (2\sin\alpha\cos\alpha)\dot{\theta}^2 = \frac{1}{2} J_1 \sin(2\alpha)\dot{\theta}^2$
* Term 3: $K \cos\alpha \dot{\theta} \dot{\alpha} \rightarrow -K \sin\alpha \dot{\theta} \dot{\alpha}$
* Potential: $-G_{grav} \cos\alpha \rightarrow G_{grav} \sin\alpha$

$$Result_B = \frac{1}{2} J_1 \sin(2\alpha) \dot{\theta}^2 - K \sin\alpha \dot{\theta} \dot{\alpha} + G_{grav} \sin\alpha$$

**Step 4: Assemble Eq 1 ($Result_A - Result_B = 0$)**
$$J_1 \ddot{\alpha} + K \cos\alpha \ddot{\theta} - K \sin\alpha \dot{\alpha} \dot{\theta} - [\frac{1}{2} J_1 \sin(2\alpha) \dot{\theta}^2 - K \sin\alpha \dot{\theta} \dot{\alpha} + G_{grav} \sin\alpha] = 0$$

*Notice:* The term $-K \sin\alpha \dot{\alpha} \dot{\theta}$ cancels out exactly.

**Final Non-Linear Pendulum Equation:**
$$(K \cos\alpha) \ddot{\theta} + J_1 \ddot{\alpha} - \frac{1}{2} J_1 \sin(2\alpha) \dot{\theta}^2 - G_{grav} \sin\alpha = 0$$

---

## 5. Derivation of Equation 2: Arm Dynamics ($\theta$)

We apply Euler-Lagrange for coordinate $\theta$:
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) - \frac{\partial L}{\partial \theta} = \tau$$

**Step 1: Partial Derivative w.r.t Velocity ($\dot{\theta}$)**
Looking at $L$:
* Term 1: $\frac{1}{2}(J_0 + J_1 \sin^2\alpha) \dot{\theta}^2 \rightarrow (J_0 + J_1 \sin^2\alpha) \dot{\theta}$
* Term 3: $(K \cos\alpha \dot{\alpha}) \dot{\theta} \rightarrow K \cos\alpha \dot{\alpha}$

$$\frac{\partial L}{\partial \dot{\theta}} = (J_0 + J_1 \sin^2\alpha) \dot{\theta} + K \cos\alpha \dot{\alpha}$$

**Step 2: Time Derivative ($\frac{d}{dt}$)**
This is complex because $\alpha$ changes with time.
* **Part A:** $\frac{d}{dt} [ (J_0 + J_1 \sin^2\alpha) \dot{\theta} ]$
    * Using Product Rule: $(J_0 + J_1 \sin^2\alpha) \ddot{\theta} + \dot{\theta} \cdot \frac{d}{dt}(J_1 \sin^2\alpha)$
    * Chain Rule for $\sin^2\alpha$: $2 \sin\alpha \cos\alpha \dot{\alpha} = \sin(2\alpha) \dot{\alpha}$
    * Result A: $(J_0 + J_1 \sin^2\alpha) \ddot{\theta} + J_1 \sin(2\alpha) \dot{\alpha} \dot{\theta}$

* **Part B:** $\frac{d}{dt} [ K \cos\alpha \dot{\alpha} ]$
    * Using Product Rule: $K \cos\alpha \ddot{\alpha} + K \dot{\alpha} (-\sin\alpha \dot{\alpha})$
    * Result B: $K \cos\alpha \ddot{\alpha} - K \sin\alpha \dot{\alpha}^2$

**Step 3: Partial Derivative w.r.t Position ($\theta$)**
$L$ does not contain $\theta$ directly.
$$\frac{\partial L}{\partial \theta} = 0$$

**Final Non-Linear Arm Equation:**
$$(J_0 + J_1 \sin^2\alpha) \ddot{\theta} + (K \cos\alpha) \ddot{\alpha} + J_1 \sin(2\alpha) \dot{\alpha} \dot{\theta} - K \sin\alpha \dot{\alpha}^2 = \tau$$

---

## 6. Linearization (Small Angle Approximation)

We assume the pendulum is near the top ($\alpha \approx 0$) and velocities are small ($\dot{\theta}^2 \approx 0, \dot{\alpha}^2 \approx 0$).

**Approximations:**
* $\cos\alpha \approx 1$
* $\sin\alpha \approx \alpha$
* $\sin^2\alpha \approx 0$
* $\sin(2\alpha) \approx 2\alpha$
* Non-linear velocity terms (Coriolis/Centripetal) $\approx 0$

**Simplified Equations:**
1.  **Arm:** $J_0 \ddot{\theta} + K \ddot{\alpha} = \tau$
2.  **Pendulum:** $K \ddot{\theta} + J_1 \ddot{\alpha} - G_{grav} \alpha = 0$

**Matrix Form:**
$$
\begin{bmatrix}
J_0 & K \\
K & J_1
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta} \\
\ddot{\alpha}
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 \\
0 & -G_{grav}
\end{bmatrix}
\begin{bmatrix}
\theta \\
\alpha
\end{bmatrix}
=
\begin{bmatrix}
\tau \\
0
\end{bmatrix}
$$

---

## 7. Final Numerical Model (Substituted Values)

We insert the calculated hardware constants into the Linear Matrix.

**Constants Recap:**
* $J_0 = 0.001104$
* $J_1 = 0.0001021$
* $K = m_p L_r l_p = 0.01196 \times 0.19 \times 0.0877 = \mathbf{0.0001993}$
* $G_{grav} = m_p g l_p = 0.01196 \times 9.81 \times 0.0877 = \mathbf{0.01029}$

**The Final System Matrices:**

$$
\begin{bmatrix}
0.001104 & 0.0001993 \\
0.0001993 & 0.0001021
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta} \\
\ddot{\alpha}
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 \\
0 & -0.01029
\end{bmatrix}
\begin{bmatrix}
\theta \\
\alpha
\end{bmatrix}
=
\begin{bmatrix}
\tau \\
0
\end{bmatrix}
$$

**Interpretation:**
This is the complete coupled model. It tells us:
1.  **Arm Equation (Row 1):** Applying motor torque $\tau$ accelerates the arm ($0.0011\ddot{\theta}$) *and* the pendulum ($0.0002\ddot{\alpha}$).
2.  **Pendulum Equation (Row 2):** Accelerating the arm ($0.0002\ddot{\theta}$) causes the pendulum to accelerate ($0.0001\ddot{\alpha}$) and fight gravity ($-0.0103\alpha$).