# Mathematical Modelling of the Rotary Inverted Pendulum (Furuta)
*Derivation from First Principles (Lagrangian Mechanics) — **complete** nonlinear model (arm + pendulum), using your updated L-rod + sphere geometry.*

---

## 0. What this document gives you (and what it assumes)

This file derives the **full coupled equations of motion** for a Furuta (rotary inverted) pendulum:

- **Arm equation** (actuated): relates motor torque $\tau$ to $\ddot{\theta}$
- **Pendulum equation** (unactuated): gives $\ddot{\alpha}$ and how it couples to $\theta$

We derive from first principles using **Lagrange's equations**. The final result is a pair of nonlinear ODEs in $\theta(t)$ and $\alpha(t)$, then we linearize about the upright equilibrium $\alpha\approx 0$.

### Explicit modelling assumptions
1. The swinging part is a **rigid body** consisting of:
   - a **vertical rod segment** of length $L_v=120$ mm
   - plus a **sphere** mounted on the rod (does not increase length)
2. The **170 mm horizontal rod segment** acts as the *hinge axle* running in 688RS bearings:
   - it **does not swing** with $\alpha$ → excluded from pendulum swing inertia $\hat{J}_2$
   - it **does yaw** with the arm about the motor axis → included in arm-side yaw inertia $\hat{J}_1$
3. Angle convention:
   - $\theta(t)$: arm yaw in the horizontal plane about the vertical $Z$-axis
   - $\alpha(t)$: pendulum angle in a vertical plane with **$\alpha=0$ upright** (unstable)
4. Sphere coverage: sphere covers **86–120 mm** of the vertical rod, so sphere COM is at:
   $$x_s=\frac{86+120}{2}\text{ mm}=103\text{ mm}=0.103\,\text{m}$$
5. Main derivation ignores friction; an optional friction term is shown later.

---

## 1. System Definition & Coordinates

### 1.1 Generalized coordinates
1. **$\theta(t)$** — arm angle (yaw). $\theta=0$ along inertial $X$-axis.
2. **$\alpha(t)$** — pendulum angle. $\alpha=0$ is upright (unstable).

### 1.2 Frames
- **Inertial frame $\mathcal{F}_O$** fixed at motor shaft, axes $(X,Y,Z)$, $Z$ upward.
- **Arm frame $\mathcal{F}_A$** rotates about $Z$ by angle $\theta$.
- Pendulum swings in a vertical plane attached to the arm tip.

---

## 2. Parameter Calculation (your specific hardware)

### 2.1 Given geometry and masses
**Arm (rotary link):**
- Arm length: $L_r = 0.19$ m
- Arm mass: $m_r = 0.051$ kg

**L-shaped rod (one rigid piece):**
- Total mass: $10.3$ g
- Total length: $170$ mm + $120$ mm = $290$ mm

**Sphere:**
- Mass: $m_s = 7.7$ g = $0.0077$ kg
- Sphere COM distance along rod: $x_s=0.103$ m

---

### 2.2 Split the L-rod into horizontal and vertical segments (mass)
Rod linear density:
$$\lambda=\frac{10.3\,\text{g}}{290\,\text{mm}}=0.03552\,\text{g/mm}$$

**Horizontal segment (hinge axle):** $L_h=170$ mm
$$m_h = L_h\lambda = 170\cdot 0.03552=6.04\,\text{g}=0.006038\,\text{kg}$$

**Vertical segment (swinging):** $L_v=120$ mm
$$m_{rod} = L_v\lambda = 120\cdot 0.03552=4.26\,\text{g}=0.004262\,\text{kg}$$

---

### 2.3 Swinging pendulum mass $m_p$
Swinging assembly = vertical rod + sphere:
$$m_p=m_{rod}+m_s=0.004262+0.0077=0.011962069\,\text{kg}$$

**Result:**
```
m_p = 0.011962069 kg
```

---

### 2.4 Pendulum COM distance $l_p$ from hinge pivot
Distances measured from the hinge axis along the vertical rod:

- Rod COM: $x_{rod}=L_v/2=0.12/2=0.06$ m
- Sphere COM: $x_s=0.103$ m

Weighted average:
$$l_p=\frac{m_{rod}x_{rod}+m_s x_s}{m_p}$$

Substitute:
$$l_p=\frac{(0.004262)(0.06)+(0.0077)(0.103)}{0.011962069}$$

Compute numerator:
$$(0.004262)(0.06)=0.06\,m_{rod},\qquad (0.0077)(0.103)=7.931\times 10^{-4}$$

**Result:**
```
l_p = 0.087679158 m ≈ 0.0877 m
```

---

### 2.5 Pendulum inertia about the hinge axis $\hat{J}_2$
This is the inertia that appears in the standard Furuta "compact" equations.

#### 2.5.1 Vertical rod inertia about one end (hinge)
For a uniform rod of length $L_v$ rotating about one end:
$$I_{rod,piv}=\frac{1}{3}m_{rod}L_v^2$$

Substitute:
```
I_rod,piv = (1/3)(0.004262)(0.12²) = 2.045793103e-05 kg·m²
```

#### 2.5.2 Sphere inertia about hinge (point-mass approximation)
Sphere modeled as a point mass at $x_s$:
$$I_{s,piv}=m_s x_s^2=(0.0077)(0.103^2)$$
```
I_s,piv = 8.168930000e-05 kg·m²
```

#### 2.5.3 (Optional) Sphere's own inertia about its COM
If you want a more faithful rigid-body sphere (radius $r=0.017$ m):
$$I_{s,cm}=\frac{2}{5}m_s r^2=\frac{2}{5}(0.0077)(0.017^2)$$
```
I_s,cm = 8.901200000e-07 kg·m²
```
This is <1% of the pivot inertia and is usually negligible for control.

#### 2.5.4 Total pendulum hinge inertia
Point-mass sphere:
```
Ĵ_2 = I_rod,piv + I_s,piv = 1.021472310e-04 kg·m²
```

With sphere self-inertia:
```
Ĵ_2 = I_rod,piv + I_s,piv + I_s,cm = 1.030373510e-04 kg·m²
```

---

### 2.6 Arm-side yaw inertia (for the $\theta$ equation)
The arm equation needs the yaw inertia about the motor axis. We define:
$$\hat{J}_1 = J_{arm} + J_{axle} + J_{motor}$$

where $J_{motor}$ is optional (stepper rotor + coupler etc). If unknown, set $J_{motor}=0$ and treat it as a tuning parameter later.

#### 2.6.1 Arm inertia about motor axis
Treat the arm as a slender rod of length $L_r$ about one end:
$$J_{arm}=\frac{1}{3}m_r L_r^2 = \frac{1}{3}(0.051)(0.19^2)$$
```
J_arm = 6.137000000e-04 kg·m²
```

#### 2.6.2 Horizontal axle inertia about motor axis
Because the axle rotates with the arm, it contributes yaw inertia. The exact value depends on its radial position.

General formula for a uniform rod of mass $m_h$ spanning radii $r\in[r_a,r_b]$:
$$J_{axle}=\int r^2\,dm=\int_{r_a}^{r_b} r^2\left(\frac{m_h}{L_h}\right)dr
=\frac{m_h}{L_h}\cdot\frac{r_b^3-r_a^3}{3}$$

**Working assumption** (common if the bend is at the arm tip and axle extends inward):
$$r_b=L_r=0.19,\qquad r_a=L_r-L_h=0.02$$

Then:
```
J_axle = 8.110954023e-05 kg·m²
```

So (with $J_{motor}=0$):
```
Ĵ_1 = J_arm + J_axle = 6.948095402e-04 kg·m²
```

---

## 3. Kinematics (position & velocity) — detailed steps

### 3.1 Arm tip position
The arm tip traces a circle in the $XY$-plane:
$$\mathbf{r}_{tip}=
\begin{bmatrix}
L_r\cos\theta\\
L_r\sin\theta\\
0
\end{bmatrix}$$

### 3.2 Pendulum COM position
The pendulum COM is at distance $l_p$ from the hinge. When upright ($\alpha=0$), the COM is at height $z=l_p$. When tilted, horizontal offset magnitude is $l_p\sin\alpha$.

A standard Furuta parametrization consistent with $\alpha=0$ upright is:
$$\mathbf{r}_p=
\begin{bmatrix}
x\\y\\z
\end{bmatrix}
=
\begin{bmatrix}
L_r\cos\theta - l_p\sin\alpha\,\sin\theta\\
L_r\sin\theta + l_p\sin\alpha\,\cos\theta\\
l_p\cos\alpha
\end{bmatrix}$$

### 3.3 Differentiate to obtain velocity components
Start with:
$$x = L_r\cos\theta - l_p\sin\alpha\sin\theta$$

Differentiate (chain + product rule):
- $d(\cos\theta)/dt = -\sin\theta\,\dot\theta$
- $d(\sin\alpha)/dt = \cos\alpha\,\dot\alpha$
- $d(\sin\theta)/dt = \cos\theta\,\dot\theta$

So:
$$\dot x = -L_r\sin\theta\,\dot\theta
- l_p\left[(\cos\alpha\,\dot\alpha)\sin\theta + \sin\alpha(\cos\theta\,\dot\theta)\right]$$

Rewriting:
$$\dot x = -L_r\sin\theta\,\dot\theta
- l_p\cos\alpha\sin\theta\,\dot\alpha
- l_p\sin\alpha\cos\theta\,\dot\theta$$

Similarly:
$$y = L_r\sin\theta + l_p\sin\alpha\cos\theta$$

Differentiate:
- $d(\sin\theta)/dt = \cos\theta\,\dot\theta$
- $d(\cos\theta)/dt = -\sin\theta\,\dot\theta$

So:
$$\dot y = L_r\cos\theta\,\dot\theta
+ l_p\left[(\cos\alpha\,\dot\alpha)\cos\theta + \sin\alpha(-\sin\theta\,\dot\theta)\right]$$

$$\dot y = L_r\cos\theta\,\dot\theta
+ l_p\cos\alpha\cos\theta\,\dot\alpha
- l_p\sin\alpha\sin\theta\,\dot\theta$$

And:
$$z=l_p\cos\alpha \quad\Rightarrow\quad \dot z = -l_p\sin\alpha\,\dot\alpha$$

### 3.4 Speed squared $v^2$
Compute:
$$v^2=\dot x^2+\dot y^2+\dot z^2$$

If you expand $\dot x^2$ and $\dot y^2$ fully, then group terms by $\dot\theta^2$, $\dot\alpha^2$, and $\dot\theta\dot\alpha$, and use $\sin^2\theta+\cos^2\theta=1$, all mixed trig cross-terms cancel. The simplified result is:

$$v^2=
\left(L_r^2+l_p^2\sin^2\alpha\right)\dot\theta^2
+ l_p^2\dot\alpha^2
+2L_r l_p\cos\alpha\,\dot\theta\dot\alpha$$

---

## 4. Energies and Lagrangian — detailed build-up

### 4.1 Kinetic energy

#### 4.1.1 Arm-side yaw KE
$$T_{arm}=\frac{1}{2}\hat{J}_1\dot\theta^2$$

#### 4.1.2 Pendulum translational KE
$$T_{p,trans}=\frac{1}{2} m_p v^2$$

Substitute $v^2$:
$$T_{p,trans}=\frac{1}{2} m_p\left[
(L_r^2+l_p^2\sin^2\alpha)\dot\theta^2
+l_p^2\dot\alpha^2
+2L_r l_p\cos\alpha\,\dot\theta\dot\alpha
\right]$$

#### 4.1.3 Pendulum rotational KE (why $\sin^2\alpha\dot\theta^2$ appears)
The pendulum experiences:
- swing angular speed $\dot\alpha$
- plus a yaw-induced component perpendicular to the pendulum of magnitude $\dot\theta\sin\alpha$ when $\alpha\neq0$

So:
$$\omega_\perp^2=\dot\alpha^2+(\dot\theta\sin\alpha)^2$$

If $J_p$ is pendulum inertia about its COM (perpendicular to swing plane):
$$T_{p,rot}=\frac{1}{2} J_p(\dot\alpha^2+\dot\theta^2\sin^2\alpha)$$

Now use the parallel-axis theorem:
$$\hat{J}_2 = J_p + m_p l_p^2$$
so we can write the pendulum total KE cleanly in terms of $\hat{J}_2$.

#### 4.1.4 Collect terms into standard Furuta constants
Define:
$$\hat{J}_0=\hat{J}_1+m_pL_r^2,\qquad K=m_pL_r l_p$$

After collecting coefficients of $\dot\theta^2$, $\dot\alpha^2$, and $\dot\theta\dot\alpha$, the **total kinetic energy** becomes:
$$T=
\frac{1}{2}(\hat{J}_0+\hat{J}_2\sin^2\alpha)\dot\theta^2
+\frac{1}{2}\hat{J}_2\dot\alpha^2
+K\cos\alpha\,\dot\theta\dot\alpha$$

### 4.2 Potential energy
Using $z=l_p\cos\alpha$:
$$V=m_p g l_p\cos\alpha$$

Define:
$$G=m_p g l_p$$

Then $V=G\cos\alpha$.

### 4.3 Lagrangian
$$\mathcal{L}=T-V
=\frac{1}{2}(\hat{J}_0+\hat{J}_2\sin^2\alpha)\dot\theta^2
+\frac{1}{2}\hat{J}_2\dot\alpha^2
+K\cos\alpha\,\dot\theta\dot\alpha
-G\cos\alpha$$

---

## 5. Euler–Lagrange Equations — full derivation for both DOF

Euler–Lagrange:
$$\frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot q}\right)
-\frac{\partial\mathcal{L}}{\partial q}
=Q$$

Generalized forces:
$$Q_\theta=\tau,\qquad Q_\alpha=0$$

---

### 5.1 Arm (θ) equation

#### Step 1: $\partial\mathcal{L}/\partial\dot\theta$
$$\frac{\partial\mathcal{L}}{\partial\dot\theta}
=(\hat{J}_0+\hat{J}_2\sin^2\alpha)\dot\theta + K\cos\alpha\,\dot\alpha$$

#### Step 2: $d/dt$ of that
First term:
$$\frac{d}{dt}\left[(\hat{J}_0+\hat{J}_2\sin^2\alpha)\dot\theta\right]
=(\hat{J}_0+\hat{J}_2\sin^2\alpha)\ddot\theta
+\frac{d}{dt}(\hat{J}_2\sin^2\alpha)\dot\theta$$

$$\frac{d}{dt}(\sin^2\alpha)=2\sin\alpha\cos\alpha\,\dot\alpha=\sin(2\alpha)\dot\alpha$$

So:
$$\frac{d}{dt}(\hat{J}_2\sin^2\alpha)\dot\theta
=\hat{J}_2\sin(2\alpha)\dot\alpha\dot\theta$$

Second term:
$$\frac{d}{dt}\left[K\cos\alpha\,\dot\alpha\right]
=K\left(\cos\alpha\,\ddot\alpha-\sin\alpha\,\dot\alpha^2\right)$$

Combine:
$$\frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot\theta}\right)
=(\hat{J}_0+\hat{J}_2\sin^2\alpha)\ddot\theta
+\hat{J}_2\sin(2\alpha)\dot\theta\dot\alpha
+K\cos\alpha\,\ddot\alpha
-K\sin\alpha\,\dot\alpha^2$$

#### Step 3: $\partial\mathcal{L}/\partial\theta$
No explicit $\theta$ in $\mathcal{L}$, so:
$$\frac{\partial\mathcal{L}}{\partial\theta}=0$$

#### Step 4: Apply Euler–Lagrange with $Q_\theta=\tau$
**Final arm equation:**
$$(\hat{J}_0+\hat{J}_2\sin^2\alpha)\ddot\theta
+K\cos\alpha\,\ddot\alpha
+\hat{J}_2\sin(2\alpha)\dot\theta\dot\alpha
-K\sin\alpha\,\dot\alpha^2
=\tau$$

---

### 5.2 Pendulum (α) equation

#### Step 1: $\partial\mathcal{L}/\partial\dot\alpha$
$$\frac{\partial\mathcal{L}}{\partial\dot\alpha}
=\hat{J}_2\dot\alpha + K\cos\alpha\,\dot\theta$$

#### Step 2: $d/dt$ of that
$$\frac{d}{dt}\left(\hat{J}_2\dot\alpha\right)=\hat{J}_2\ddot\alpha$$

$$\frac{d}{dt}\left(K\cos\alpha\,\dot\theta\right)
=K\left(\cos\alpha\,\ddot\theta-\sin\alpha\,\dot\alpha\dot\theta\right)$$

So:
$$\frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot\alpha}\right)
=\hat{J}_2\ddot\alpha
+K\cos\alpha\,\ddot\theta
-K\sin\alpha\,\dot\alpha\dot\theta$$

#### Step 3: $\partial\mathcal{L}/\partial\alpha$
Compute term-by-term:

1) From $\frac{1}{2}(\hat{J}_0+\hat{J}_2\sin^2\alpha)\dot\theta^2$:
$$\frac{\partial}{\partial\alpha}\left[\frac{1}{2}\hat{J}_2\sin^2\alpha\,\dot\theta^2\right]
=\frac{1}{2}\hat{J}_2\sin(2\alpha)\dot\theta^2$$

2) From $K\cos\alpha\,\dot\theta\dot\alpha$:
$$\frac{\partial}{\partial\alpha}[K\cos\alpha\,\dot\theta\dot\alpha]
=-K\sin\alpha\,\dot\theta\dot\alpha$$

3) From $-G\cos\alpha$:
$$\frac{\partial}{\partial\alpha}[-G\cos\alpha]=+G\sin\alpha$$

Combine:
$$\frac{\partial\mathcal{L}}{\partial\alpha}
=\frac{1}{2}\hat{J}_2\sin(2\alpha)\dot\theta^2
-K\sin\alpha\,\dot\theta\dot\alpha
+G\sin\alpha$$

#### Step 4: Apply Euler–Lagrange with $Q_\alpha=0$
$$\hat{J}_2\ddot\alpha+K\cos\alpha\,\ddot\theta-K\sin\alpha\,\dot\alpha\dot\theta
-\left(\frac{1}{2}\hat{J}_2\sin(2\alpha)\dot\theta^2
-K\sin\alpha\,\dot\theta\dot\alpha+G\sin\alpha\right)=0$$

The $\pm K\sin\alpha\,\dot\theta\dot\alpha$ terms cancel, leaving:

**Final pendulum equation:**
$$K\cos\alpha\,\ddot\theta
+\hat{J}_2\ddot\alpha
-\frac{1}{2}\hat{J}_2\sin(2\alpha)\dot\theta^2
-G\sin\alpha
=0$$

---

## 6. Final nonlinear Furuta model (complete)

**Arm equation:**
$$(\hat{J}_0+\hat{J}_2\sin^2\alpha)\ddot\theta
+K\cos\alpha\,\ddot\alpha
+\hat{J}_2\sin(2\alpha)\dot\theta\dot\alpha
-K\sin\alpha\,\dot\alpha^2
=\tau$$

**Pendulum equation:**
$$K\cos\alpha\,\ddot\theta
+\hat{J}_2\ddot\alpha
-\frac{1}{2}\hat{J}_2\sin(2\alpha)\dot\theta^2
-G\sin\alpha
=0$$

### Optional viscous friction
Add:
- $+b_\theta\dot\theta$ to the left of the arm equation
- $+b_\alpha\dot\alpha$ to the left of the pendulum equation

---

## 7. Linearization about upright ($\alpha\approx 0$)

Use:
$$\sin\alpha\approx \alpha,\quad \cos\alpha\approx 1,\quad \sin(2\alpha)\approx 2\alpha$$

and drop products of small quantities $(\dot\theta\dot\alpha,\,\dot\alpha^2,\,\alpha\dot\theta^2)$.

### Linear model
$$\hat{J}_0\ddot\theta+K\ddot\alpha=\tau$$
$$K\ddot\theta+\hat{J}_2\ddot\alpha-G\alpha=0$$

Matrix form:
$$\begin{bmatrix}
\hat{J}_0 & K\\
K & \hat{J}_2
\end{bmatrix}
\begin{bmatrix}
\ddot\theta\\
\ddot\alpha
\end{bmatrix}
+
\begin{bmatrix}
0&0\\
0&-G
\end{bmatrix}
\begin{bmatrix}
\theta\\
\alpha
\end{bmatrix}
=
\begin{bmatrix}
\tau\\
0
\end{bmatrix}$$

### Reduced pendulum relation
From $K\ddot\theta+\hat{J}_2\ddot\alpha-G\alpha=0$:
$$\ddot\alpha=\frac{G}{\hat{J}_2}\alpha-\frac{K}{\hat{J}_2}\ddot\theta$$

Numerically (using point-mass sphere $\hat{J}_2=1.021472310e-04$):
```
ḍα = 100.727α - 1.951ḍθ
```

### Unstable “natural” rate near upright
$$\omega=\sqrt{\frac{G}{\hat{J}_2}}=\sqrt{100.727}=10.036\ \text{rad/s}$$
$$f=\frac{\omega}{2\pi}=1.597\ \text{Hz}$$

---

## 8. Numerical summary (all key constants)

Using your current geometry and the working axle placement assumption:

```
m_p     = 0.011962069 kg
l_p     = 0.087679158 m
Ĵ_2     = 1.021472310e-04 kg·m²  (or 1.030373510e-04 with sphere self-inertia)
K       = 1.992765862e-04 kg·m²
G       = 1.028896479e-02 N·m
J_arm   = 6.137000000e-04 kg·m²
J_axle  = 8.110954023e-05 kg·m²  (depends on axle radial placement)
Ĵ_1     = 6.948095402e-04 kg·m²  (assuming J_motor=0)
Ĵ_0     = 1.126640230e-03 kg·m²
```

---

## 9. Notes on correctness
- The equation pair in §6 is the standard Furuta model structure (no friction), specialized to your $\alpha=0$ upright convention.
- Differences between "published versions" usually come from:
  - different angle conventions (upright vs downward reference)
  - whether authors include rotor inertia and friction
  - whether they use $\hat{J}_2$ (hinge inertia) vs $J_p$ (COM inertia) and how they convert between them

*End of file.*
