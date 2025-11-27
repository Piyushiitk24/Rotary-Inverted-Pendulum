# Rotary Inverted Pendulum – Modelling & Gain Design Notes

## 0. Goal

Model your **specific** rotary inverted pendulum + stepper hardware, and use that model to:

- Derive a **continuous-time plant** from **base velocity (Hz)** to **pendulum angle (deg)**.
- Use that plant to design a **PD controller by Bode/loop-shaping**.
- Map the resulting continuous-time PD gains into the firmware as `Kp_balance`, `Kd_balance`.

This document records all the assumptions, parameter calculations, and algebra so we don’t have to re-derive anything later.

---

## 1. Hardware description

### 1.1 Pendulum (L-shaped rod)

- Material: PLA, ~20% infill (approx. uniform density).
- Geometry:
  - Cylindrical rod, diameter: **9 mm**.
  - **Horizontal segment**:
    - Length \(L_h = 170\) mm.
    - Lies along the rotary arm (x-axis), supported in bearings.
    - From these 170 mm, 162 mm overlap the rotary arm; ~8 mm extend beyond the edge.
  - **Vertical segment**:
    - Length \(L_v = 280\) mm.
    - Bends downwards from the far end of the horizontal segment (z-axis).
- Total pendulum mass:
  - \(m = 22\) g \(= 0.022\) kg.

For modelling:

- Pivot is at the elbow where the vertical piece begins.
- Horizontal part sits roughly through the pivot axis, so its influence on gravity torque is small; the vertical part dominates gravity and inertia.

### 1.2 Rotary arm

- Shape: uniform “capsule” (rounded rectangle).
- Dimensions:
  - Length: 190 mm
  - Width: 32 mm
  - Thickness: 6 mm
- Features (from one edge along the length):
  - 20 mm: motor shaft hole.
  - 28 mm: AS5600 motor sensor.
  - 46 mm: bearing 1 (688RS).
  - 94 mm: bearing 2.
  - 142 mm: bearing 3.
- Total arm + bearings + motor sensor mass:
  - **51 g**. (Used conceptually; not needed in reduced model.)

Distance from base shaft (vertical axis) to pendulum pivot:

- Approximately \(L = 190\) mm \(= 0.19\) m.

### 1.3 Actuation and sensing

- MCU: Arduino Mega 2560.
- Actuator: stepper motor driven by FastAccelStepper.
  - Command: `setSpeedInHz(f)` → attempts to rotate at **f steps/sec**.
  - Each step produces ~**0.225°** of base rotation.
- Sensors (via TCA9548A I²C mux):
  - Pendulum angle (AS5600, channel 0).
  - Base angle (AS5600, channel 1).

The firmware control loop:

- Runs at **125 Hz** (Timer 3, 8 ms tick).
- Each tick:
  - Reads pendulum and base angles, estimates velocities.
  - Computes a motor speed command in **Hz**.
  - Calls `setSpeedInHz()` and chooses forward/backward direction.

---

## 2. Coordinate definitions

We use the standard **Furuta pendulum** convention.

Generalised coordinates:

- \( \theta(t) \): base angle about the vertical axis (what the stepper rotates).
- \( \alpha(t) \): pendulum angle **from upright** in the vertical plane:
  - \( \alpha = 0 \): pendulum exactly upright.
  - Positive α: lean in some fixed direction (just pick a sign and stay consistent).

Your code uses:

- `motorAngle` ≈ θ (deg).
- `pendAngle` ≈ α (deg).

We explicitly assume **small angles** around upright for modelling and gain design:

- |α| small (say <10–15°) for linearisation to be valid.

---

## 3. Nonlinear model (conceptual form)

We won’t re-derive the full Lagrange equations line-by-line; we just need the standard structure.

For a rotary inverted pendulum (Furuta) with:

- Base inertia \(J_a\),
- Pendulum inertia about pivot \(J_p\),
- Pendulum mass \(m\),
- Horizontal arm length \(L\) (base shaft → pendulum pivot),
- Distance from pendulum pivot to its centre of mass \(l\),

the nonlinear equations can be written in joint space as:

\[
M(q)
\begin{bmatrix}
\ddot\theta \\ \ddot\alpha
\end{bmatrix}
+
C(q,\dot q)
\begin{bmatrix}
\dot\theta \\ \dot\alpha
\end{bmatrix}
+
G(q) =
\begin{bmatrix}
\tau \\ 0
\end{bmatrix}
\]

where:

- \(q = [\theta,\,\alpha]^T\),
- \(M(q)\) is a 2×2 symmetric inertia matrix,
- \(C(q,\dot q)\dot q\) includes Coriolis/centrifugal terms,
- \(G(q)\) is the gravity vector,
- τ is motor torque at the base.

We only need the **linearisation around upright and small velocities**, where:

- \(M(q)\) can be taken as constant \(M\),
- Coriolis/centrifugal terms vanish to first order,
- \(G(q)\) becomes linear in α.

---

## 4. Linearisation around upright

The linearised equations around \( \alpha \approx 0,\, \dot\theta \approx 0,\,\dot\alpha \approx 0 \) are:

\[
M
\begin{bmatrix}
\ddot\theta \\ \ddot\alpha
\end{bmatrix}
+
\begin{bmatrix}
0 \\ -m g l\,\alpha
\end{bmatrix}
=
\begin{bmatrix}
\tau \\ 0
\end{bmatrix}
\]

with:

\[
M =
\begin{bmatrix}
M_{11} & M_{12}\\
M_{21} & M_{22}
\end{bmatrix}
=
\begin{bmatrix}
J_a + mL^2 & mLl \\
mLl & J_p + ml^2
\end{bmatrix}
\]

Explicitly:

- Row 1 (base dynamics):
  \[
  M_{11}\ddot\theta + M_{12}\ddot\alpha = \tau
  \]
- Row 2 (pendulum dynamics):
  \[
  M_{21}\ddot\theta + M_{22}\ddot\alpha - mgl\,\alpha = 0
  \]

Crucial observation:

- Base inertia \(J_a\) only appears in \(M_{11}\).
- The pendulum equation (row 2) depends on **\(\ddot\theta\)** via \(M_{21}\) but **not on \(J_a\)** directly.

---

## 5. Reduction to “pendulum driven by base motion”

We do **not** command torque τ directly (stepper driver hides it). Instead:

- We think of **θ(t) as an imposed motion** created by the actuator.
- We only care how that motion affects α.

With that mindset, treat θ(t), \(\dot\theta(t)\), \(\ddot\theta(t)\) as **inputs**, not states we solve from τ. So we use only **row 2**:

\[
M_{21}\ddot\theta + M_{22}\ddot\alpha - mgl\,\alpha = 0
\]

Solve for \(\ddot\alpha\):

\[
\ddot\alpha
= \frac{mgl}{M_{22}}\,\alpha - \frac{M_{21}}{M_{22}}\,\ddot\theta
= a\,\alpha + b\,\ddot\theta
\]

where we define:

- \( M_{21} = mLl\)
- \( M_{22} = J_p + ml^2\)

and:

\[
a = \frac{mgl}{J_p + ml^2},\qquad
b = -\frac{mLl}{J_p + ml^2}
\]

So the linearised **pendulum equation** is:

> \[
> \ddot\alpha = a\,\alpha + b\,\ddot\theta
> \]

This is the core continuous-time model.

- The **unstable gravity mode** is encoded in \(a > 0\).
- The **coupling from base acceleration to pendulum angle** is a constant \(b < 0\).

---

## 6. Estimating parameters from your geometry

### 6.1 Mass distribution

Total pendulum mass:  
\(m = 0.022\) kg.

Total length:  
\(L_{\text{tot}} = L_h + L_v = 0.17 + 0.28 = 0.45\) m.

Assume **uniform linear density**:

\[
\lambda = \frac{m}{L_{\text{tot}}} = \frac{0.022}{0.45} \approx 0.0489\ \text{kg/m}
\]

Mass of horizontal and vertical segments:

- Horizontal:
  \[
  m_h = \lambda L_h \approx 0.0489 \cdot 0.17 \approx 0.00831\ \text{kg}
  \]
- Vertical:
  \[
  m_v = \lambda L_v \approx 0.0489 \cdot 0.28 \approx 0.01369\ \text{kg}
  \]

Check:  
\(m_h + m_v \approx 0.022\) kg → consistent.

### 6.2 Centre of mass distance \(l\)

Take pivot at the elbow where the vertical rod starts.

- Horizontal segment is essentially at \(z = 0\).
- Vertical segment extends from \(z = 0\) down to \(z = -L_v\).
  - Its COM is at \(z = -L_v/2 = -0.14\) m.

Total COM z-coordinate:

\[
z_{\text{COM}} =
\frac{m_h \cdot 0 + m_v \cdot (-0.14)}{m_h + m_v}
=
\frac{-m_v \cdot 0.14}{m}
\approx \frac{-0.01369 \cdot 0.14}{0.022}
\approx -0.087\ \text{m}
\]

So:

- Distance from pivot to COM (downwards):  
  \[
  l \approx 0.087\ \text{m}
  \]

### 6.3 Pendulum inertia \(J_p\)

We approximate most of the inertia as coming from the vertical rod. Inertia of a slender rod of length \(L_v\) about its centre, for rotation perpendicular to its length, is:

\[
J_{v,\text{COM}} = \frac{1}{12} m_v L_v^2
\]

Compute:

- \(L_v^2 = 0.28^2 = 0.0784\)
- \(m_v L_v^2 \approx 0.01369 \cdot 0.0784 \approx 0.001073\)
- So
  \[
  J_p \approx J_{v,\text{COM}} = \frac{1}{12} \cdot 0.001073 \approx 8.94\times10^{-5}\ \text{kg·m}^2
  \]

We’re slightly inconsis­tent here: in the original derivation \(J_p\) is about the **pendulum’s COM**, and the matrix uses \(J_p + m l^2\) to shift to the pivot. We’ll keep that interpretation.

### 6.4 Effective inertia term \(J_p + m l^2\)

Compute:

- \(l^2 \approx 0.087^2 \approx 0.00757\)
- \(m l^2 \approx 0.022 \cdot 0.00757 \approx 0.0001665\)
- So:

  \[
  J_p + m l^2 \approx 8.94\times10^{-5} + 1.665\times10^{-4}
                      \approx 2.56\times10^{-4}\ \text{kg·m}^2
  \]

We’ll denote this as:

\[
I_{\text{eff}} = J_p + m l^2 \approx 2.56\times10^{-4}
\]

### 6.5 Horizontal arm length \(L\)

Given mechanical description:

- Distance from base shaft to pendulum pivot ≈ 190 mm:

\[
L = 0.19\ \text{m}
\]

### 6.6 Final a, b, and natural frequency

Now compute:

\[
a = \frac{m g l}{I_{\text{eff}}},\quad
b = -\frac{m L l}{I_{\text{eff}}}
\]

Use:

- \(m = 0.022\) kg,
- \(g = 9.81\) m/s²,
- \(l = 0.087\) m,
- \(L = 0.19\) m,
- \(I_{\text{eff}} \approx 2.56\times10^{-4}\).

First \(m g l\):

\[
m g l \approx 0.022 \cdot 9.81 \cdot 0.087
        \approx 0.022 \cdot 0.853 \approx 0.0188
\]

Then:

\[
a = \frac{0.0188}{2.56\times10^{-4}} \approx 73.3\ \text{s}^{-2}
\]

Base acceleration coupling:

\[
m L l \approx 0.022 \cdot 0.19 \cdot 0.087
        \approx 0.022 \cdot 0.01653 \approx 0.0003636
\]

\[
b = -\frac{0.0003636}{2.56\times10^{-4}} \approx -1.42
\]

So:

> \[
> \ddot\alpha = 73.3\,\alpha - 1.42\,\ddot\theta
> \]

Natural (unstable) frequency:

\[
\omega_0 = \sqrt{a} \approx \sqrt{73.3} \approx 8.57\ \text{rad/s}
\]

\[
f_0 = \frac{\omega_0}{2\pi} \approx \frac{8.57}{6.283} \approx 1.36\ \text{Hz}
\]

That’s the ~1.2–1.4 Hz unstable pole we’ve been talking about.

---

## 7. Plant with base **velocity** input and stepper units

### 7.1 Base velocity input

Define base angular velocity \(v(t) = \dot\theta(t)\). In Laplace domain:

- \(\ddot\theta(s) = s \cdot v(s)\)

Substitute into the pendulum equation:

\[
s^2 \alpha(s) = a \alpha(s) + b \cdot s v(s)
\]

Rearrange:

\[
(s^2 - a) \alpha(s) = b s v(s)
\]

So:

\[
G_{\alpha v}(s) = \frac{\alpha(s)}{v(s)} = \frac{b s}{s^2 - a}
\]

This is the plant with **input = base angular velocity (rad/s)** and **output = pendulum angle (rad)**.

### 7.2 Stepper command (Hz) to base velocity

Each step of the stepper rotates the base by:

- \(Δ\theta = 0.225° = 0.225 \cdot \frac{\pi}{180} = \frac{\pi}{800} \approx 0.003927\ \text{rad}\)

If the driver produces **u steps per second** (Hz):

- Base angular velocity:
  \[
  v = \dot\theta = k_v u
  \]
  where
  \[
  k_v = \frac{\pi}{800} \approx 0.003927\ \text{rad·s}^{-1}\text{/Hz}
  \]

### 7.3 Output in degrees instead of radians

Control software works with pendulum angle in **degrees**. Convert:

\[
\alpha_{\text{deg}} = \frac{180}{\pi} \alpha
\]

Combine everything:

\[
\frac{\alpha_{\text{deg}}(s)}{u(s)}
=
\frac{180}{\pi} \cdot G_{\alpha v}(s) \cdot k_v
=
\frac{180}{\pi} \cdot \frac{b s}{s^2 - a} \cdot \frac{\pi}{800}
=
\frac{b \cdot 180/800}{s^2 - a} s
\]

But \(180/800 = 0.225\). So the net gain is simply:

\[
k = b \cdot 0.225 \approx -1.42 \cdot 0.225 \approx -0.3196
\]

So the **final scalar plant**, mapping **step frequency u (Hz)** to **pendulum angle α (deg)**, is:

> \[
> G(s) = \frac{\alpha_{\text{deg}}(s)}{u(s)}
>      = \frac{-0.3196\,s}{s^2 - 73.33}
> \]

That’s the model we use for Bode and controller design.

---

## 8. Bode analysis and PD design

### 8.1 Target closed-loop bandwidth

We want the closed-loop to be significantly faster than the unstable mode (~1.36 Hz) but not so fast that:

- sensor noise dominates,
- stepper torque/saturation destroy the behaviour.

A standard rule of thumb is:

- closed-loop bandwidth \(f_c \approx 3\text{–}4 \times f_0\).

With \(f_0 \approx 1.36\) Hz, choose:

- \(f_c \approx 4\) Hz.

This corresponds to:

- \(\omega_c = 2\pi f_c = 2\pi \cdot 4 \approx 25.13\ \text{rad/s}\).

### 8.2 Magnitude of the plant at crossover

For our plant:

\[
G(j\omega) = \frac{-0.3196 \cdot j\omega}{(j\omega)^2 - a}
           = \frac{-0.3196 \cdot j\omega}{- \omega^2 - a}
           = 0.3196 \frac{j\omega}{\omega^2 + a}
\]

Denominator is **real and negative**, numerator is purely imaginary. Magnitude is:

\[
|G(j\omega)| = \frac{0.3196 \cdot \omega}{\omega^2 + a}
\]

At ω = ω_c = 25.13 rad/s:

- \(\omega_c^2 \approx 25.13^2 \approx 631.6\)
- \(\omega_c^2 + a \approx 631.6 + 73.3 \approx 704.9\)

So:

\[
|G(j\omega_c)| \approx \frac{0.3196 \cdot 25.13}{704.9}
                 \approx \frac{8.03}{704.9}
                 \approx 0.0114
\]

That’s about −39 dB.

### 8.3 Controller structure

We choose a **continuous-time PD**:

\[
C(s) = K_p + K_d s
\]

To gain some phase lead around crossover, we put the controller zero at **1 Hz**:

- Zero at \(f_z = 1\) Hz → \(\omega_z = 2\pi f_z \approx 6.283\) rad/s.

We can equivalently write:

\[
C(s) = K_p \left(1 + \frac{s}{\omega_z}\right)
\]

so:

\[
K_d = \frac{K_p}{\omega_z}
\]

### 8.4 Solve for Kp to hit |L(jωc)| = 1

Open-loop:

\[
L(s) = C(s) G(s)
\]

At ω = ω_c:

\[
C(j\omega_c) = K_p \left(1 + j\frac{\omega_c}{\omega_z}\right)
\]

Magnitude:

\[
|C(j\omega_c)| = K_p \left|1 + j\frac{\omega_c}{\omega_z}\right|
\]

Compute:

- \(\omega_c / \omega_z = (2\pi \cdot 4)/(2\pi \cdot 1) = 4\)
- So:
  \[
  \left|1 + j4\right| = \sqrt{1^2 + 4^2} = \sqrt{17} \approx 4.123
  \]

So:

\[
|L(j\omega_c)| = |C(j\omega_c) G(j\omega_c)|
               = K_p \cdot 4.123 \cdot |G(j\omega_c)|
\]

We want |L(jω_c)| = 1 at crossover:

\[
1 = K_p \cdot 4.123 \cdot 0.0114
\]

So:

\[
K_p = \frac{1}{4.123 \cdot 0.0114}
    \approx \frac{1}{0.0470}
    \approx 21.3\ \text{Hz/deg}
\]

Then:

\[
K_d = \frac{K_p}{\omega_z}
    = \frac{21.3}{6.283}
    \approx 3.4\ \text{Hz/(deg/s)}
\]

These are **continuous-time PD gains** that give:

- open-loop crossover ~4 Hz,
- some phase lead from the zero at 1 Hz,
- reasonable expectation of stabilisation (subject to actuator limits).

Phase behaviour:

- G(jω) has net phase ~±90° (imaginary over negative real).
- The PD adds phase lead near crossover.
- With the sign choice fixed by physical direction mapping (stepper direction vs α response), the resulting loop has a usable phase margin (we’re not sitting at −180° at crossover).

---

## 9. Mapping continuous PD to firmware gains

### 9.1 Firmware control law

The new firmware uses a **direct PD in Hz units**:

```cpp
// error in degrees, pendulum velocity in deg/s
float error   = pendAngle - pendulumSetpoint;
float velForD = constrain(pendVelocity, -D_VEL_CLAMP, D_VEL_CLAMP);

float rawHz =
    (Kp_balance * error) +
    (Ki_balance * integralError) -
    (Kd_balance * velForD);
```

Then:

```cpp
targetHz = rawHz (with saturation and soft limits);
setMotorVelocity(targetHz);
```

So, ignoring I for now (Ki=0), the implemented control law is:

- Input: α (deg), \(\dot\alpha\) (deg/s)
- Output: \(u = \text{targetHz}\) (Hz)
- Controller:
  \[
  u = K_p^{\text{eff}} \cdot \alpha - K_d^{\text{eff}} \cdot \dot\alpha
  \]
  with:
  \[
  K_p^{\text{eff}} = Kp\_balance,\quad K_d^{\text{eff}} = Kd\_balance
  \]

This matches the **continuous-time PD** we designed:

\[
C(s) = K_p + K_d s
\]

with:

- \(K_p \leftrightarrow Kp\_balance\),
- \(K_d \leftrightarrow Kd\_balance\).

### 9.2 Chosen starting gains

From the Bode design:

- \(K_p \approx 21.3\) Hz/deg.
- \(K_d \approx 3.4\) Hz/(deg/s).

So we set:

```cpp
float Kp_balance = 21.3f;
float Ki_balance = 0.0f;
float Kd_balance = 3.4f;
float Kp_center  = 0.0f;   // centering off for initial tuning
```

That’s exactly what the current script uses.

---

## 10. Summary and caveats

**Model structure:**

1. Start from rotary inverted pendulum dynamics.
2. Linearise around upright:  
   \(M_{21}\ddot\theta + M_{22}\ddot\alpha - mgl\,\alpha = 0\).
3. Reduce to pendulum-only equation:  
   \(\ddot\alpha = a\alpha + b\ddot\theta\) with  
   \(a = mg l / (J_p + ml^2)\),  
   \(b = -mLl / (J_p + ml^2)\).
4. Estimate parameters (m, l, L, \(J_p\)) from your L-rod geometry.
5. Compute \(a \approx 73.3\), \(b \approx -1.42\).
6. Derive natural unstable mode \(f_0 ≈ 1.36\) Hz.
7. Include stepper scaling (Hz → rad/s) and rad→deg:
   \[
   G(s) = \frac{\alpha_{\text{deg}}(s)}{u(s)}
        = \frac{-0.3196\,s}{s^2 - 73.33}
   \]

**Controller design:**

1. Choose target crossover \(f_c ≈ 4\) Hz (~3× unstable frequency).
2. Use PD with zero at 1 Hz:
   \[
   C(s) = K_p\left(1 + \frac{s}{2\pi}\right)
   \]
3. Solve |C(jω_c)G(jω_c)| = 1 → \(K_p ≈ 21.3\) Hz/deg.
4. Compute \(K_d = K_p / (2\pi) ≈ 3.4\) Hz/(deg/s).
5. Implement in code as:
   ```cpp
   targetHz = Kp_balance * error - Kd_balance * pendVelocity;
   ```
   with `Kp_balance = 21.3`, `Kd_balance = 3.4`.

**Approximations and limitations:**

- Pendulum model ignores:
  - Horizontal rod inertia (small near pivot),
  - Base inertia \(J_a\) (dropped when θ(t) is treated as input),
  - Nonlinear Coriolis terms (small near upright),
  - Friction and stiction (left for empirical tuning).
- Stepper is treated as an ideal velocity source up to `MAX_MOTOR_HZ`, with no lost steps.
- Controller is designed in continuous time; implementation is discrete at 125 Hz. For these frequencies the error is modest but not zero.
- Soft and hard base limits, saturation and deadzones will modify the effective loop at large motions.

This is still a **good first-order model** and gives a consistent, physics-based starting point for tuning. Further refinement (e.g. including stepper torque limits, adding centering gain, or introducing an I-term) should be done on top of this, not from scratch.
