---
title: "Mathematical Modelling of a Rotary Inverted Pendulum (Furuta)"
date: "January 21, 2026"
geometry: margin=1in
fontsize: 11pt
header-includes:
  - \usepackage{amsmath}
  - \usepackage{amssymb}
---

 
## 1. Definition

Generalized coordinates:

$\theta(t)$: arm yaw angle.

$\alpha(t)$: pendulum angle, with $\alpha=0$ upright.

Constants:

$L_r$: arm length.

$m_p$: swinging mass.

$l_p$: distance from hinge to pendulum COM.

$g$: gravity.

$\tau$: actuation torque about $\theta$.

Define

$$K \equiv m_p L_r l_p,\qquad G \equiv m_p g l_p$$

Let

$\hat J_1$: yaw inertia about motor axis (arm-side).

$\hat J_2$: pendulum hinge inertia.

$J_p$: pendulum inertia about its center of mass, about an axis perpendicular to the swing plane.
$$\hat J_0 \triangleq \hat J_1 + m_p L_r^2$$

Rigid links; frictionless joints.

---

## 2. Parameter evaluation

Given:

$$L_r=0.19\ \mathrm{m},\quad m_r=0.051\ \mathrm{kg}$$
$$L_h=0.17\ \mathrm{m},\quad L_v=0.12\ \mathrm{m}$$
$$m_{\mathrm{rod,total}}=10.3\ \mathrm{g}=0.0103\ \mathrm{kg}$$
$$m_s=7.7\ \mathrm{g}=0.0077\ \mathrm{kg},\quad x_s=0.103\ \mathrm{m}$$

Linear density:

$$\lambda = \frac{10.3\ \mathrm{g}}{290\ \mathrm{mm}}=0.03552\ \mathrm{g/mm}$$

Mass split:

$$m_h = 170\lambda = 6.04\ \mathrm{g}=0.006038\ \mathrm{kg}$$
$$m_{\mathrm{rod}} = 120\lambda = 4.26\ \mathrm{g}=0.004262\ \mathrm{kg}$$

Swinging mass:

$$m_p = m_{\mathrm{rod}}+m_s = 0.011962069\ \mathrm{kg}$$

COM distance:

$$x_{\mathrm{rod}}=\frac{L_v}{2}=0.06\ \mathrm{m}$$
$$l_p = \frac{m_{\mathrm{rod}}x_{\mathrm{rod}}+m_s x_s}{m_p}$$
$$l_p = \frac{(0.004262)(0.06) + (0.0077)(0.103)}{0.011962069}$$
$$l_p = \frac{2.5572\times 10^{-4} + 7.931\times 10^{-4}}{0.011962069} = 0.087679158\ \mathrm{m}$$

Pendulum hinge inertia (rod about end + sphere point mass):

$$I_{\mathrm{rod,piv}} = \frac{1}{3}m_{\mathrm{rod}}L_v^2 = \frac{1}{3}(0.004262)(0.12^2)=2.045793103\times 10^{-5}$$
$$I_{\mathrm{s,piv}} = m_s x_s^2 = (0.0077)(0.103^2)=8.16893\times 10^{-5}$$
$$\hat J_2 = I_{\mathrm{rod,piv}}+I_{\mathrm{s,piv}} = 1.021472310\times 10^{-4}\ \mathrm{kg\,m^2}$$

Gravity constant:

$$G=m_p g l_p=(0.011962069)(9.81)(0.087679158)=1.028896479\times 10^{-2}\ \mathrm{N\,m}$$

Coupling constant:

$$K=m_p L_r l_p=(0.011962069)(0.19)(0.087679158)=1.992765862\times 10^{-4}\ \mathrm{kg\,m^2}$$

Arm-side yaw inertia:

$$\hat J_0=\hat J_1+m_pL_r^2$$

---

## 3. Kinematics

Let $\{\mathbf i,\mathbf j,\mathbf k\}$ be an inertial orthonormal basis. Define the planar arm basis

$$\mathbf e_r \equiv \cos\theta\,\mathbf i + \sin\theta\,\mathbf j,\qquad \mathbf e_t \equiv -\sin\theta\,\mathbf i + \cos\theta\,\mathbf j$$

Arm hinge position:

$$\mathbf r_h = L_r\mathbf e_r$$

Pendulum COM relative to hinge:

$$\mathbf r_{p/h}=l_p\bigl(\sin\alpha\,\mathbf e_t + \cos\alpha\,\mathbf k\bigr)$$

Pendulum COM position:

$$\mathbf r_p = \mathbf r_h + \mathbf r_{p/h}$$

Components $\mathbf r_p = x\mathbf i+y\mathbf j+z\mathbf k$:

$$x = L_r\cos\theta - l_p\sin\alpha\,\sin\theta$$
$$y = L_r\sin\theta + l_p\sin\alpha\,\cos\theta$$
$$z = l_p\cos\alpha$$

Derivatives:

$$\frac{d}{dt}(\cos\theta)=-\sin\theta\,\dot\theta,\quad \frac{d}{dt}(\sin\theta)=\cos\theta\,\dot\theta$$
$$\frac{d}{dt}(\sin\alpha)=\cos\alpha\,\dot\alpha,\quad \frac{d}{dt}(\cos\alpha)=-\sin\alpha\,\dot\alpha$$

### 3.1 Velocity components

For $x$:

$$\dot x = \frac{d}{dt}(L_r\cos\theta) - l_p\frac{d}{dt}(\sin\alpha\,\sin\theta)$$
$$\frac{d}{dt}(L_r\cos\theta)=-L_r\sin\theta\,\dot\theta$$
$$\frac{d}{dt}(\sin\alpha\,\sin\theta)=(\cos\alpha\,\dot\alpha)\sin\theta + \sin\alpha(\cos\theta\,\dot\theta)$$
$$\dot x = -L_r\sin\theta\,\dot\theta - l_p\cos\alpha\sin\theta\,\dot\alpha - l_p\sin\alpha\cos\theta\,\dot\theta$$

For $y$:

$$\dot y = \frac{d}{dt}(L_r\sin\theta) + l_p\frac{d}{dt}(\sin\alpha\,\cos\theta)$$
$$\frac{d}{dt}(L_r\sin\theta)=L_r\cos\theta\,\dot\theta$$
$$\frac{d}{dt}(\sin\alpha\,\cos\theta)=(\cos\alpha\,\dot\alpha)\cos\theta + \sin\alpha(-\sin\theta\,\dot\theta)$$
$$\dot y = L_r\cos\theta\,\dot\theta + l_p\cos\alpha\cos\theta\,\dot\alpha - l_p\sin\alpha\sin\theta\,\dot\theta$$

For $z$:

$$\dot z = \frac{d}{dt}(l_p\cos\alpha)=-l_p\sin\alpha\,\dot\alpha$$

### 3.2 Speed squared

Compute $v^2=\dot x^2+\dot y^2+\dot z^2$.

Define

$$A=L_r\sin\theta + l_p\sin\alpha\cos\theta,\quad B=l_p\cos\alpha\sin\theta$$
$$C=L_r\cos\theta - l_p\sin\alpha\sin\theta,\quad D=l_p\cos\alpha\cos\theta$$

Then

$$\dot x = -A\dot\theta - B\dot\alpha,\qquad \dot y = C\dot\theta + D\dot\alpha,\qquad \dot z = -l_p\sin\alpha\,\dot\alpha$$

Square:

$$\dot x^2 = A^2\dot\theta^2 + B^2\dot\alpha^2 + 2AB\dot\theta\dot\alpha$$
$$\dot y^2 = C^2\dot\theta^2 + D^2\dot\alpha^2 + 2CD\dot\theta\dot\alpha$$
$$\dot z^2 = l_p^2\sin^2\alpha\,\dot\alpha^2$$

Collect $\dot\theta^2$:

$$A^2+C^2 = (L_r\sin\theta + l_p\sin\alpha\cos\theta)^2 + (L_r\cos\theta - l_p\sin\alpha\sin\theta)^2$$

Expand:

$$\begin{aligned}
A^2 &= L_r^2\sin^2\theta + l_p^2\sin^2\alpha\cos^2\theta + 2L_r l_p\sin\alpha\sin\theta\cos\theta,\\
C^2 &= L_r^2\cos^2\theta + l_p^2\sin^2\alpha\sin^2\theta - 2L_r l_p\sin\alpha\sin\theta\cos\theta.
\end{aligned}$$

Add:

$$A^2+C^2 = L_r^2(\sin^2\theta+\cos^2\theta) + l_p^2\sin^2\alpha(\sin^2\theta+\cos^2\theta)=L_r^2+l_p^2\sin^2\alpha$$

Collect $\dot\alpha^2$:

$$B^2+D^2+ l_p^2\sin^2\alpha = l_p^2\cos^2\alpha(\sin^2\theta+\cos^2\theta)+l_p^2\sin^2\alpha=l_p^2$$

Collect $\dot\theta\dot\alpha$:

$$AB+CD = l_p\cos\alpha\Big[(L_r\sin\theta + l_p\sin\alpha\cos\theta)\sin\theta + (L_r\cos\theta - l_p\sin\alpha\sin\theta)\cos\theta\Big]$$

Expand bracket:

$$\begin{aligned}
&(L_r\sin\theta + l_p\sin\alpha\cos\theta)\sin\theta + (L_r\cos\theta - l_p\sin\alpha\sin\theta)\cos\theta\\
&=L_r(\sin^2\theta+\cos^2\theta)+l_p\sin\alpha(\cos\theta\sin\theta-\sin\theta\cos\theta)=L_r.
\end{aligned}$$

Thus

$$AB+CD = L_r l_p\cos\alpha$$

Therefore

$$v^2=(L_r^2+l_p^2\sin^2\alpha)\dot\theta^2 + l_p^2\dot\alpha^2 + 2L_r l_p\cos\alpha\,\dot\theta\dot\alpha$$

---

## 4. Energies

Translational kinetic energy:

$$T_{\mathrm{trans}}=\frac{1}{2}m_p v^2$$

Rotational kinetic energy of the pendulum about its COM (perpendicular axis):

$$T_{\mathrm{rot}}=\frac{1}{2}J_p(\dot\alpha^2+\sin^2\alpha\,\dot\theta^2)$$

Parallel axis:

$$\hat J_2 = J_p + m_p l_p^2$$

Arm yaw kinetic energy:

$$T_{\mathrm{arm}}=\frac{1}{2}\hat J_1\dot\theta^2$$

Total kinetic energy:

$$T=T_{\mathrm{arm}}+T_{\mathrm{trans}}+T_{\mathrm{rot}}$$

Collect terms.

Coefficient of $\dot\theta^2$:

$$\frac{1}{2}\Big[\hat J_1+m_pL_r^2+(m_pl_p^2+J_p)\sin^2\alpha\Big]\dot\theta^2
=\frac{1}{2}(\hat J_0+\hat J_2\sin^2\alpha)\dot\theta^2$$

Coefficient of $\dot\alpha^2$:

$$\frac{1}{2}(m_pl_p^2+J_p)\dot\alpha^2=\frac{1}{2}\hat J_2\dot\alpha^2$$

Cross term:

$$m_pL_r l_p\cos\alpha\,\dot\theta\dot\alpha = K\cos\alpha\,\dot\theta\dot\alpha$$

Thus

$$T=\frac{1}{2}(\hat J_0+\hat J_2\sin^2\alpha)\dot\theta^2 + \frac{1}{2}\hat J_2\dot\alpha^2 + K\cos\alpha\,\dot\theta\dot\alpha$$

Potential energy:

$$V=m_p g z = m_p g l_p\cos\alpha = G\cos\alpha$$


Lagrangian:

$$\mathcal L=T-V$$

---

## 5. Euler–Lagrange equations

$$\frac{d}{dt}\left(\frac{\partial\mathcal L}{\partial\dot q}\right)-\frac{\partial\mathcal L}{\partial q}=Q$$

$$Q_\theta=\tau,\qquad Q_\alpha=0$$

### 5.1 Equation in $\theta$

$$\frac{\partial\mathcal L}{\partial\dot\theta}=(\hat J_0+\hat J_2\sin^2\alpha)\dot\theta + K\cos\alpha\,\dot\alpha$$

Differentiate:

$$\frac{d}{dt}\left[(\hat J_0+\hat J_2\sin^2\alpha)\dot\theta\right]=(\hat J_0+\hat J_2\sin^2\alpha)\ddot\theta + \hat J_2\frac{d}{dt}(\sin^2\alpha)\dot\theta$$

$$\frac{d}{dt}(\sin^2\alpha)=2\sin\alpha\cos\alpha\,\dot\alpha=\sin(2\alpha)\dot\alpha$$

$$\frac{d}{dt}(K\cos\alpha\,\dot\alpha)=K\left(-\sin\alpha\,\dot\alpha^2+\cos\alpha\,\ddot\alpha\right)$$

Also $\partial\mathcal L/\partial\theta=0$.

Thus

$$ (\hat J_0+\hat J_2\sin^2\alpha)\ddot\theta + K\cos\alpha\,\ddot\alpha + \hat J_2\sin(2\alpha)\dot\theta\dot\alpha - K\sin\alpha\,\dot\alpha^2 = \tau$$

### 5.2 Equation in $\alpha$

$$\frac{\partial\mathcal L}{\partial\dot\alpha}=\hat J_2\dot\alpha + K\cos\alpha\,\dot\theta$$

Differentiate:

$$\frac{d}{dt}\left(\hat J_2\dot\alpha\right)=\hat J_2\ddot\alpha$$
$$\frac{d}{dt}(K\cos\alpha\,\dot\theta)=K\left(-\sin\alpha\,\dot\alpha\dot\theta+\cos\alpha\,\ddot\theta\right)$$

Partial derivative:

$$\frac{\partial\mathcal L}{\partial\alpha}=\frac{1}{2}\hat J_2\frac{\partial}{\partial\alpha}(\sin^2\alpha)\dot\theta^2 - K\sin\alpha\,\dot\theta\dot\alpha + G\sin\alpha$$
$$\frac{\partial}{\partial\alpha}(\sin^2\alpha)=\sin(2\alpha)$$

Thus

$$\frac{\partial\mathcal L}{\partial\alpha}=\frac{1}{2}\hat J_2\sin(2\alpha)\dot\theta^2 - K\sin\alpha\,\dot\theta\dot\alpha + G\sin\alpha$$

Euler–Lagrange ($Q_\alpha=0$):

$$\hat J_2\ddot\alpha + K\cos\alpha\,\ddot\theta - K\sin\alpha\,\dot\alpha\dot\theta - \left(\frac{1}{2}\hat J_2\sin(2\alpha)\dot\theta^2 - K\sin\alpha\,\dot\theta\dot\alpha + G\sin\alpha\right)=0$$

Cancel the $\pm K\sin\alpha\,\dot\theta\dot\alpha$ terms:

$$K\cos\alpha\,\ddot\theta + \hat J_2\ddot\alpha - \frac{1}{2}\hat J_2\sin(2\alpha)\dot\theta^2 - G\sin\alpha = 0$$

---

## 6. Nonlinear model

$$ (\hat J_0+\hat J_2\sin^2\alpha)\ddot\theta + K\cos\alpha\,\ddot\alpha + \hat J_2\sin(2\alpha)\dot\theta\dot\alpha - K\sin\alpha\,\dot\alpha^2 = \tau$$

$$K\cos\alpha\,\ddot\theta + \hat J_2\ddot\alpha - \frac{1}{2}\hat J_2\sin(2\alpha)\dot\theta^2 - G\sin\alpha = 0$$

The derived equations are structurally consistent with classical Furuta pendulum models, with the distinction that actuator dynamics are treated at the acceleration level rather than torque level.

---

## 7. Linearization about upright ($\alpha\approx 0$)

$$\sin\alpha\approx \alpha,\qquad \cos\alpha\approx 1,\qquad \sin(2\alpha)\approx 2\alpha$$

Neglect products of small quantities.

Linear model:

$$\hat J_0\ddot\theta + K\ddot\alpha = \tau$$
$$K\ddot\theta + \hat J_2\ddot\alpha - G\alpha = 0$$

Solve for $\ddot\alpha$:

$$\ddot\alpha = \frac{G}{\hat J_2}\alpha - \frac{K}{\hat J_2}\ddot\theta$$

---

## 8. Numerical specialization

Let $J_0\equiv \hat J_0$ and $J_1\equiv \hat J_2$. Numerical values:

$$J_0=0.001104\ \mathrm{kg\,m^2},\quad J_1=1.021\times 10^{-4}\ \mathrm{kg\,m^2},\quad K=1.993\times 10^{-4}\ \mathrm{kg\,m^2},\quad G=0.01029\ \mathrm{N\,m}$$

Linear equations:

$$J_0\ddot\theta + K\ddot\alpha = \tau$$
$$K\ddot\theta + J_1\ddot\alpha - G\alpha = 0$$

Reduced relation:

$$\ddot\alpha = \frac{G}{J_1}\alpha - \frac{K}{J_1}\ddot\theta$$

Numerically:

$$\ddot\alpha = 100.8\,\alpha - 1.952\,\ddot\theta$$

Open-loop fall rate ($\tau=0$). From $\ddot\theta=-(K/J_0)\ddot\alpha$:

$$\left(J_1-\frac{K^2}{J_0}\right)\ddot\alpha - G\alpha = 0$$

$$J_{eq}=J_1-\frac{K^2}{J_0}=0.0001021-\frac{(0.0001993)^2}{0.001104}=6.612\times 10^{-5}$$

$$\ddot\alpha = \frac{G}{J_{eq}}\alpha,\qquad \lambda=\pm\sqrt{\frac{G}{J_{eq}}}=\pm 12.47\ \mathrm{rad/s}$$

Mass matrix inverse:

$$M=\begin{bmatrix}J_0 & K\\K & J_1\end{bmatrix},\qquad M^{-1}=\frac{1}{J_0J_1-K^2}\begin{bmatrix}J_1 & -K\\-K & J_0\end{bmatrix}$$

$$J_0J_1-K^2=7.2998\times 10^{-8}$$

$$M^{-1}=\begin{bmatrix}1398.7 & -2730.2\\-2730.2 & 15123.7\end{bmatrix}$$

Substituted form:

$$0.001104\,\ddot\theta + 0.0001993\,\ddot\alpha = \tau$$
$$0.0001993\,\ddot\theta + 0.0001021\,\ddot\alpha - 0.01029\,\alpha = 0$$

Nonlinear model with numerical constants:

$$ (J_0+J_1\sin^2\alpha)\ddot\theta + K\cos\alpha\,\ddot\alpha + J_1\sin(2\alpha)\dot\theta\dot\alpha - K\sin\alpha\,\dot\alpha^2 = \tau$$

$$K\cos\alpha\,\ddot\theta + J_1\ddot\alpha - \frac{1}{2}J_1\sin(2\alpha)\dot\theta^2 - G\sin\alpha = 0$$

---

## 9. PD design on commanded arm acceleration

Plant:

$$\ddot\alpha = A\alpha - B\ddot\theta,\qquad A\equiv \frac{G}{J_1},\quad B\equiv \frac{K}{J_1}$$

Controller:

$$\ddot\theta_{cmd}=-k_p\alpha-k_d\dot\alpha$$

Closed-loop characteristic equation:

$$\ddot\alpha - (Bk_d)\dot\alpha - (A+Bk_p)\alpha=0$$

Match to

$$\ddot\alpha + 2\zeta\omega_c\dot\alpha + \omega_c^2\alpha = 0$$

Gains:

$$k_d=-\frac{2\zeta\omega_c}{B},\qquad k_p=-\frac{A+\omega_c^2}{B}$$

With $\omega_c=15$, $\zeta=0.8$, $A=100.8$, $B=1.952$:

$$k_p=-166.9,\qquad k_d=-12.3$$

Negative gains arise from the sign convention $\alpha=0$ upright; the controller provides restoring acceleration opposing the unstable gravitational term.

Stepper conversion ($N=1600$ microsteps/rev):

$$\mathrm{steps/rad}=\frac{N}{2\pi}=254.65,\qquad \mathrm{steps/deg}=\frac{N}{360}=4.444$$

Degree variables ($\alpha_{\deg}=\alpha\,180/\pi$):

$$\ddot\theta_{\mathrm{steps}}=-(742\,\alpha_{\deg}+54.6\,\dot\alpha_{\deg})$$

---

## 10. State-space representation and LQR design

### 10.1 State definition

Define the state vector as

$$\mathbf{x}=\begin{bmatrix}x_1\\x_2\\x_3\\x_4\end{bmatrix}=\begin{bmatrix}\theta\\\alpha\\\dot\theta\\\dot\alpha\end{bmatrix}$$

The control input is chosen as the commanded arm angular acceleration

$$u\equiv \ddot\theta$$

---

### 10.2 Linearized state-space model

The state equations are

$$\dot x_1=x_3$$
$$\dot x_2=x_4$$
$$\dot x_3=u$$
$$\dot x_4=A\,x_2-B\,u$$

where

$$A\equiv \frac{G}{J_1},\qquad B\equiv \frac{K}{J_1}$$

Collecting terms, the state-space model is

$$\dot{\mathbf{x}}=\mathbf{A}\mathbf{x}+\mathbf{B}u$$

with

$$\mathbf{A}=\begin{bmatrix}
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
0 & 0 & 0 & 0\\
0 & A & 0 & 0
\end{bmatrix},\qquad
\mathbf{B}=\begin{bmatrix}0\\0\\1\\-B\end{bmatrix}$$

Numerically,

$$\mathbf{A}=\begin{bmatrix}
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
0 & 0 & 0 & 0\\
0 & 100.8 & 0 & 0
\end{bmatrix},\qquad
\mathbf{B}=\begin{bmatrix}0\\0\\1\\-1.952\end{bmatrix}$$

---

### 10.3 LQR formulation

A continuous-time linear quadratic regulator (LQR) is designed with the control law

$$u=-\mathbf{K}\mathbf{x}$$

minimizing the cost function

$$J=\int_0^\infty\left(\mathbf{x}^\mathsf{T}\mathbf{Q}\mathbf{x}+u^\mathsf{T}\mathbf{R}u\right)\,dt$$

---

### 10.4 LQR gain

The gain is computed by solving the continuous-time algebraic Riccati equation (CARE)

$$\mathbf{A}^\mathsf{T}\mathbf{P}+\mathbf{P}\mathbf{A}-\mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^\mathsf{T}\mathbf{P}+\mathbf{Q}=\mathbf{0}$$

and then

$$\mathbf{K}=\mathbf{R}^{-1}\mathbf{B}^\mathsf{T}\mathbf{P}.$$

For the numerical gain below, the weights are

$$\mathbf{Q}=\mathrm{diag}(0.5,\ 50.0,\ 0.05,\ 5.0),\qquad \mathbf{R}=\begin{bmatrix}1.0\end{bmatrix}.$$

This choice prioritizes pendulum angle stabilization (large weight on $\alpha$) while penalizing control effort.

Numerical values are obtained by solving the CARE (e.g., via SciPy `solve_continuous_are`) and evaluating $\mathbf{K}=\mathbf{R}^{-1}\mathbf{B}^\mathsf{T}\mathbf{P}$.

$$\mathbf{K}=\begin{bmatrix}-0.70710678 & -117.18259227 & -1.3583044 & -11.86304115\end{bmatrix}$$

The resulting control law is

$$u=-(-0.7071\,\theta-117.183\,\alpha-1.3583\,\dot\theta-11.8630\,\dot\alpha)$$

Stepper units using degree variables. Let

$$\mathbf{x}_{\deg}=\begin{bmatrix}\theta_{\deg}\\\alpha_{\deg}\\\dot\theta_{\deg}\\\dot\alpha_{\deg}\end{bmatrix},\qquad \mathbf{x}=\frac{\pi}{180}\mathbf{x}_{\deg}$$

Let $u_{\deg}\equiv \ddot\theta_{\deg}$ denote acceleration in $\mathrm{deg/s^2}$, so that $u=(\pi/180)u_{\deg}$. Then

$$\ddot\theta_{\mathrm{steps}}=\frac{N}{2\pi}u=\frac{N}{360}u_{\deg}$$

Then

$$\ddot\theta_{\mathrm{steps}}=-\mathbf{K}_{\mathrm{steps}}\mathbf{x}_{\deg}$$

$$\mathbf{K}_{\mathrm{steps}}=\left(\frac{N}{360}\right)\mathbf{K}=4.444\,\mathbf{K}=\begin{bmatrix}-3.1424 & -520.76 & -6.0363 & -52.72\end{bmatrix}$$

$$\ddot\theta_{\mathrm{steps}}=-(-3.1424\,\theta_{\deg}-520.76\,\alpha_{\deg}-6.0363\,\dot\theta_{\deg}-52.72\,\dot\alpha_{\deg})$$

---

## 11. Full-state feedback design by pole placement (coefficient matching)

This section provides an explicit *hand-calculable* method to compute a single full-state feedback law

$$u\equiv \ddot\theta = -\mathbf K \mathbf x,\qquad \mathbf x=\begin{bmatrix}\theta\\ \alpha\\ \dot\theta\\ \dot\alpha\end{bmatrix},\qquad \mathbf K=\begin{bmatrix}k_\theta & k_\alpha & k_{\dot\theta} & k_{\dot\alpha}\end{bmatrix}$$

Unlike the two-loop implementation (inner on $\alpha,\dot\alpha$ + outer on $\theta,\dot\theta$), this uses **one equation** that simultaneously stabilizes the pendulum and recenters the base.

### 11.1 Plant (linearized) used for state feedback

From Sec. 10.2, the linearized model about upright is

$$\dot x_1=x_3,\qquad \dot x_2=x_4,\qquad \dot x_3=u,\qquad \dot x_4=A\,x_2-B\,u$$

where

$$A\equiv \frac{G}{J_1},\qquad B\equiv \frac{K}{J_1}$$

Numerically (Sec. 10.2),

$$A=100.8,\qquad B=1.952$$

### 11.2 Full-state feedback law

Choose

$$u = -(k_\theta x_1 + k_\alpha x_2 + k_{\dot\theta} x_3 + k_{\dot\alpha} x_4)$$

Equivalently (using physical names),

$$\ddot\theta = -(k_\theta\,\theta + k_\alpha\,\alpha + k_{\dot\theta}\,\dot\theta + k_{\dot\alpha}\,\dot\alpha)$$

### 11.3 Closed-loop characteristic polynomial (derived explicitly)

Substitute the feedback law into the state equations. The closed-loop matrix is

$$\mathbf A_{cl}=\mathbf A-\mathbf B\mathbf K$$

With

$$\mathbf A=\begin{bmatrix} 0&0&1&0\\ 0&0&0&1\\ 0&0&0&0\\ 0&A&0&0 \end{bmatrix},\quad \mathbf B=\begin{bmatrix}0\\0\\1\\-B\end{bmatrix}$$

we get

$$\mathbf A_{cl}= \begin{bmatrix} 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1\\ -k_\theta & -k_\alpha & -k_{\dot\theta} & -k_{\dot\alpha}\\ Bk_\theta & A+Bk_\alpha & Bk_{\dot\theta} & Bk_{\dot\alpha} \end{bmatrix}$$

To obtain the characteristic polynomial, write the dynamics in scalar form and eliminate $\theta$:

$$\dot\theta=\dot x_1=x_3,\quad \ddot\theta=\dot x_3=u$$

$$\dot\alpha=\dot x_2=x_4,\quad \ddot\alpha=\dot x_4=A\alpha-Bu$$

Using the feedback law,

$$u=-(k_\theta\theta+k_\alpha\alpha+k_{\dot\theta}\dot\theta+k_{\dot\alpha}\dot\alpha)$$

Then

$$\ddot\theta + k_{\dot\theta}\dot\theta + k_\theta\theta + k_{\dot\alpha}\dot\alpha + k_\alpha\alpha = 0$$

and

$$\ddot\alpha = A\alpha + B(k_\theta\theta+k_\alpha\alpha+k_{\dot\theta}\dot\theta+k_{\dot\alpha}\dot\alpha)$$

Differentiate the $\ddot\alpha$ equation twice and substitute for $\theta,\dot\theta,\ddot\theta$ using the $\theta$-equation. The closed-loop characteristic polynomial becomes

$$s^4 + a_3 s^3 + a_2 s^2 + a_1 s + a_0 = 0$$

with coefficients expressed directly in terms of the feedback gains:

$$\begin{aligned} a_3 &= k_{\dot\theta}-B\,k_{\dot\alpha}\\ a_2 &= k_\theta - A - B\,k_\alpha\\ a_1 &= -A\,k_{\dot\theta}\\ a_0 &= -A\,k_\theta \end{aligned}$$

This allows **direct coefficient matching** to solve for $k_\theta, k_\alpha, k_{\dot\theta}, k_{\dot\alpha}$.

### 11.4 Choosing desired poles using two second-order factors

Choose a desired 4th-order polynomial as the product of two standard 2nd-order factors:

$$(s^2+2\zeta_1\omega_1 s+\omega_1^2)\,(s^2+2\zeta_2\omega_2 s+\omega_2^2)$$

Interpretation:

- $(\omega_1,\zeta_1)$: **fast pendulum stabilization mode** (upright balance).
- $(\omega_2,\zeta_2)$: **slower base centering mode** (drift rejection).

Expand the product:

$$s^4 + \tilde a_3 s^3 + \tilde a_2 s^2 + \tilde a_1 s + \tilde a_0$$

where

$$\begin{aligned} \tilde a_3 &= 2(\zeta_1\omega_1+\zeta_2\omega_2)\\ \tilde a_2 &= \omega_1^2+\omega_2^2+4\zeta_1\zeta_2\omega_1\omega_2\\ \tilde a_1 &= 2(\zeta_1\omega_1\omega_2^2+\zeta_2\omega_2\omega_1^2)\\ \tilde a_0 &= \omega_1^2\omega_2^2 \end{aligned}$$

### 11.5 Pole-placement gains by coefficient matching (closed-form)

Set $(a_0,a_1,a_2,a_3)=(\tilde a_0,\tilde a_1,\tilde a_2,\tilde a_3)$. Using the relations in Sec. 11.3, solve in this order:

From $a_0=-A k_\theta$:

$$k_\theta = -\frac{\tilde a_0}{A}$$

From $a_1=-A k_{\dot\theta}$:

$$k_{\dot\theta} = -\frac{\tilde a_1}{A}$$

From $a_3=k_{\dot\theta}-B k_{\dot\alpha}$:

$$k_{\dot\alpha} = \frac{k_{\dot\theta}-\tilde a_3}{B}$$

From $a_2=k_\theta-A-Bk_\alpha$:

$$k_\alpha = \frac{k_\theta-A-\tilde a_2}{B}$$

These formulas provide a fully analytic state feedback design based on the chosen $(\omega_1,\zeta_1,\omega_2,\zeta_2)$.

### 11.6 Numerical example (using the identified plant constants)

Use the numerical plant values (Sec. 10.2):

$$A=100.8,\qquad B=1.952$$

Choose design parameters:

- Fast mode: $\omega_1=15\ \mathrm{rad/s},\ \zeta_1=0.8$ (same as Sec. 9 PD choice).
- Slow mode: $\omega_2=1.0\ \mathrm{rad/s},\ \zeta_2=1.0$ (gentle recentering).

Compute the desired polynomial coefficients:

$$\tilde a_3 = 2(\zeta_1\omega_1+\zeta_2\omega_2)=2(0.8\cdot 15 + 1.0\cdot 1)=2(12+1)=26$$

$$\tilde a_2 = \omega_1^2+\omega_2^2+4\zeta_1\zeta_2\omega_1\omega_2 = 15^2+1^2+4(0.8)(1.0)(15)(1) =225+1+48=274$$

$$\tilde a_1 = 2(\zeta_1\omega_1\omega_2^2+\zeta_2\omega_2\omega_1^2) =2(0.8\cdot 15\cdot 1^2 + 1.0\cdot 1\cdot 15^2) =2(12+225)=474$$

$$\tilde a_0 = \omega_1^2\omega_2^2 = 15^2\cdot 1^2=225$$

Now compute gains:

$$k_\theta=-\frac{\tilde a_0}{A}=-\frac{225}{100.8}=-2.232$$

$$k_{\dot\theta}=-\frac{\tilde a_1}{A}=-\frac{474}{100.8}=-4.702$$

$$k_{\dot\alpha}=\frac{k_{\dot\theta}-\tilde a_3}{B} =\frac{-4.702-26}{1.952}=-15.729$$

$$k_\alpha=\frac{k_\theta-A-\tilde a_2}{B} =\frac{-2.232-100.8-274}{1.952}=-193.15$$

Thus, the pole-placement full-state gain is

$$\mathbf K= \begin{bmatrix} -2.232 & -193.15 & -4.702 & -15.729 \end{bmatrix}$$

giving the control law

$$\ddot\theta = -\big(-2.232\,\theta -193.15\,\alpha -4.702\,\dot\theta -15.729\,\dot\alpha\big)$$

(As with Sec. 9, the negative gains arise from the $\alpha=0$ upright sign convention.)

### 11.7 Conversion to stepper units (degree variables)

Let the controller be implemented in degree variables:

$$\mathbf x_{\deg}=\begin{bmatrix}\theta_{\deg}\\\alpha_{\deg}\\\dot\theta_{\deg}\\\dot\alpha_{\deg}\end{bmatrix}$$

Since each state in radians equals $(\pi/180)$ times the degree state, and the commanded acceleration in $\mathrm{steps/s^2}$ is

$$\ddot\theta_{\mathrm{steps}}=\left(\frac{N}{360}\right)\ddot\theta_{\deg}=(\mathrm{steps/deg})\,\ddot\theta_{\deg}$$

the gain vector in stepper units using degree states is

$$\mathbf K_{\mathrm{steps}} = (\mathrm{steps/deg})\,\mathbf K = 4.444\,\mathbf K$$

For the numerical example above:

$$\mathbf K_{\mathrm{steps}} = 4.444 \begin{bmatrix} -2.232 & -193.15 & -4.702 & -15.729 \end{bmatrix} = \begin{bmatrix} -9.92 & -858.4 & -20.9 & -69.9 \end{bmatrix}$$

So the implementable acceleration command is

$$\ddot\theta_{\mathrm{steps}}= -\Big( -9.92\,\theta_{\deg} -858.4\,\alpha_{\deg} -20.9\,\dot\theta_{\deg} -69.9\,\dot\alpha_{\deg} \Big)$$

### 11.8 Practical notes for implementation

- The two-mode pole selection $(\omega_1,\zeta_1,\omega_2,\zeta_2)$ provides a tunable tradeoff: larger $\omega_1$ increases upright stiffness but increases required acceleration and sensitivity to noise; larger $\omega_2$ recenters faster but can inject base motion and couple back into $\alpha$.
- If acceleration saturates frequently, reduce $\omega_1$ (or increase damping $\zeta_1$) and/or reduce $\omega_2$.
- If slow drift persists despite adequate gains, augment with a small integral term on $\theta$ (bias rejection), implemented with anti-windup and small limits.

---
