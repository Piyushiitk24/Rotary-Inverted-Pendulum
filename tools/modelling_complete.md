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

$L_r$: arm length (motor axis to pendulum pivot axis).

$L_h$: horizontal length of the L-shaped pendulum (pivot shaft segment).

$L_v$: vertical length of the L-shaped pendulum (swinging segment).

$m_p$: swinging mass.

$l_p$: distance from pivot axis to pendulum COM (of the swinging mass).

$g$: gravity.

$\tau$: actuation torque about $\theta$.

Geometry note (this rig): the pendulum is an \textbf{L-shaped rod}. The horizontal segment is aligned with the arm (radial direction) and forms a bearing-supported pivot shaft (three 688RS bearings). The vertical segment carries most of the swinging mass and swings in the $(\mathbf e_t,\mathbf k)$ plane.

In the reduced model used for control, $m_p$ and $l_p$ refer to the mass distribution that moves with $\alpha$ (vertical segment plus concentrated mass). The horizontal segment lies on the pivot axis and contributes primarily to arm-side inertia.

Define

$$K \equiv m_p L_r l_p,\qquad G \equiv m_p g l_p$$

These grouped constants are introduced for compactness: $G$ appears when collecting gravity potential/torque terms, and $K$ appears when collecting inertial coupling terms between arm rotation and pendulum swing.

Let

$\hat J_1$: yaw inertia about motor axis (arm-side).

$\hat J_2$: pendulum pivot inertia (about the pivot axis).

$J_p$: pendulum inertia about its center of mass, about an axis perpendicular to the swing plane.
$$\hat J_0 \triangleq \hat J_1 + m_p L_r^2$$

The definition of $\hat J_0$ follows from collecting the $\dot{\theta}^2$ terms in the kinetic energy: it is the arm-side yaw inertia about the motor axis plus the parallel-axis contribution of the swinging mass at radius $L_r$.

Rigid links; frictionless joints.

A schematic of the coordinates and parameters is shown in Fig.\ \ref{fig:furuta_schematic}.

\begin{figure}[htbp]
  \centering
  \begin{subfigure}[t]{\linewidth}
    \centering
    \begin{tikzpicture}[x=1cm, y=1cm, line cap=round, line join=round]
      \def\thetad{35}
      \def\Rarm{6.0}
      \coordinate (O) at (0,0);
      \coordinate (P) at ({\Rarm*cos(\thetad)},{\Rarm*sin(\thetad)});

      % Inertial axes (top view) and yaw angle.
      \draw[-{Latex[length=3mm]}] (O) -- (7.2,0) node[below] {$\mathbf i$};
      \draw[-{Latex[length=3mm]}] (O) -- (0,4.6) node[left] {$\mathbf j$};
      \draw[-{Latex[length=3mm]}] (1.2,0) arc (0:\thetad:1.2) node[pos=0.55, right] {$\theta$};

      % Out-of-plane axis.
      \fill (O) circle (0.06);
      \node[above left] at (O) {$\mathbf k$ (out of plane)};

      % Arm and pivot location.
      \draw[line width=1.1pt] (O) -- (P);
      \node[above] at ($(O)!0.55!(P)$) {$L_r$};
      \fill (P) circle (0.06);

      % Local arm basis at the pivot.
      \draw[-{Latex[length=3mm]}] (P) -- ($(P)+({1.4*cos(\thetad)},{1.4*sin(\thetad)})$) node[above] {$\mathbf e_r$};
      \draw[-{Latex[length=3mm]}] (P) -- ($(P)+({1.4*cos(\thetad+90)},{1.4*sin(\thetad+90)})$) node[left] {$\mathbf e_t$};

      % Bearing-supported pivot shaft (schematic).
      \coordinate (B1) at ($(P)+({-0.9*cos(\thetad)},{-0.9*sin(\thetad)})$);
      \coordinate (B2) at ($(P)+({-0.5*cos(\thetad)},{-0.5*sin(\thetad)})$);
      \coordinate (B3) at ($(P)+({-0.1*cos(\thetad)},{-0.1*sin(\thetad)})$);
      \foreach \B in {B1,B2,B3} {
        \draw[thin] (\B) circle (0.18);
        \draw[thin] (\B) circle (0.12);
      }
      \node[align=left, anchor=west] (btxt) at ($(P)+({-1.9*cos(\thetad+90)},{-1.9*sin(\thetad+90)})$) {3$\times$ 688RS bearings\\(pivot shaft)};
      \draw[-{Latex[length=3mm]}] (btxt.east) -- (B2);

      % Actuation torque about the base axis (schematic).
      \draw[-{Latex[length=3mm]}] ($(O)+(1.6,0)$) arc (0:-300:1.6);
      \node at (2.0,-1.1) {$\tau$};
    \end{tikzpicture}
    \caption{Top view (arm plane).}
  \end{subfigure}

  \vspace{0.5em}

  \begin{subfigure}[t]{\linewidth}
    \centering
    \begin{tikzpicture}[x=1cm, y=1cm, line cap=round, line join=round]
      \def\alphad{25}
      \def\Lrod{6.0}
      \def\Lcom{4.4}
      \coordinate (O) at (0,0);
      \coordinate (E) at ({\Lrod*sin(\alphad)},{\Lrod*cos(\alphad)});
      \coordinate (C) at ({\Lcom*sin(\alphad)},{\Lcom*cos(\alphad)});

      % Axes in the swing plane.
      \draw[-{Latex[length=3mm]}] (O) -- (7.2,0) node[below] {$\mathbf e_t$};
      \draw[-{Latex[length=3mm]}] (O) -- (0,6.6) node[left] {$\mathbf k$};
      \node[anchor=west] at (0.2,0.3) {pivot axis $\mathbf e_r$ (out of page)};

      % Upright reference.
      \draw[dashed] (O) -- (0,6.0);

      % L-shaped pendulum (swinging segment shown in this plane).
      \draw[line width=1.1pt] (O) -- (E);
      \fill (C) circle (0.08);
      \node[right] at (C) {$m_p$ (COM)};
      \node[pos=0.70, above left] at ($(O)!0.70!(E)$) {$L_v$};
      \node[pos=0.55, above right] at ($(O)!0.55!(C)$) {$l_p$};

      % Angle and gravity.
      \draw[-{Latex[length=3mm]}] (0,1.6) arc (90:90-\alphad:1.6) node[pos=0.60, right] {$\alpha$};
      \draw[-{Latex[length=3mm]}] ($(C)+(0.9,0)$) -- ++(0,-2.0) node[below] {$g$};
    \end{tikzpicture}
    \caption{Side view (pendulum swing plane).}
  \end{subfigure}
  \caption{Rotary (Furuta) inverted pendulum schematic and generalized coordinates used in this chapter.}
  \label{fig:furuta_schematic}
\end{figure}
\FloatBarrier

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

Pendulum pivot inertia (rod about end + sphere point mass):

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

Pendulum pivot position:

$$\mathbf r_h = L_r\mathbf e_r$$

Pendulum COM relative to pivot:

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

### 11.9 Discrete derivative filter (professor’s bilinear/Tustin method)

The controllers in this project require measurements of $\dot\alpha$ and $\dot\theta$. Direct finite differences are very noisy at 200 Hz and amplify encoder quantization and I²C glitches. Instead, the firmware implements a **filtered derivative** (a differentiator with a first-order roll-off), discretized using the **bilinear (Tustin) transform**.

#### 11.9.1 Continuous-time filter

Use the transfer function

$$H(s)=\frac{s}{1+s/\omega_c}$$

where $\omega_c$ (rad/s) is the derivative filter cutoff. This behaves like an ideal differentiator at low frequency but limits high-frequency amplification.

#### 11.9.2 Bilinear transform and discrete-time transfer function

Let the sampling period be $T$ (seconds). Apply the bilinear transform

$$s \leftarrow \frac{2}{T}\,\frac{1-z^{-1}}{1+z^{-1}}$$

to obtain

$$H(z)=\frac{\frac{2}{T}\frac{1-z^{-1}}{1+z^{-1}}}{1+\frac{1}{\omega_c}\frac{2}{T}\frac{1-z^{-1}}{1+z^{-1}}}$$

Multiply numerator and denominator by $(1+z^{-1})$:

$$H(z)=\frac{\frac{2}{T}(1-z^{-1})}{(1+z^{-1})+\frac{2}{\omega_c T}(1-z^{-1})}$$

Define $k\equiv\frac{2}{\omega_c T}$. Then the denominator becomes:

$$(1+z^{-1})+k(1-z^{-1})=(1+k)+(1-k)z^{-1}$$

Divide numerator and denominator by $(1+k)$ to obtain the standard first-order IIR form:

$$H(z)=b_0\,\frac{(1-z^{-1})}{1+a_1 z^{-1}}$$

with coefficients

$$b_0=\frac{\frac{2}{T}}{1+\frac{2}{\omega_c T}}=\frac{2\omega_c}{2+\omega_c T},\qquad
a_1=\frac{1-\frac{2}{\omega_c T}}{1+\frac{2}{\omega_c T}}=\frac{\omega_c T-2}{\omega_c T+2}$$

#### 11.9.3 Difference equation (firmware form)

Let $x[n]$ be the sampled angle (deg) and $y[n]$ the filtered derivative estimate (deg/s). Taking the inverse $z$-transform yields:

$$y[n]=b_0\,(x[n]-x[n-1]) - a_1\,y[n-1]$$

This is exactly the firmware update:

$$\dot x_{\mathrm{filt}}[n]=b_0\,\Delta x[n] - a_1\,\dot x_{\mathrm{filt}}[n-1]$$

where $\Delta x[n]=x[n]-x[n-1]$ is the one-sample increment.

#### 11.9.4 Numerical example (current firmware defaults)

The control loop runs at 200 Hz, so $T=0.005$ s. With the default $\omega_c=450$ rad/s:

$$\mathrm{denom}=2+\omega_c T = 2 + 450(0.005)=4.25$$
$$b_0=\frac{2\omega_c}{2+\omega_c T}=\frac{900}{4.25}=211.7647$$
$$a_1=\frac{\omega_c T-2}{\omega_c T+2}=\frac{2.25-2}{4.25}=0.0588$$

The corresponding cutoff frequency is $f_c=\omega_c/(2\pi)\approx 71.6$ Hz. (This is high relative to 200 Hz sampling, but still useful as a light roll-off while keeping derivative lag small; the output is also clamped in firmware for safety.)

#### 11.9.5 Angle wrap-around and glitch handling (implementation detail)

Because $\alpha$ (and sometimes $\theta$) are periodic, the increment $\Delta x[n]$ should be computed as the **shortest signed angle difference** (wrap-safe), i.e. wrapped into $(-180^\circ,180^\circ]$. In firmware, this is implemented by adjusting $\Delta x$ by $\pm 360^\circ$ when needed.

Additionally, the implementation rejects impossible one-sample jumps (I²C/mux glitches) by zeroing $\Delta x$ if $|\Delta x|$ exceeds a configured threshold, and clamps the resulting derivative estimate to a safe range. These safeguards are critical because both SMC surfaces depend directly on $\dot\alpha$.

---

## 12. Nonlinear Sliding Mode Control for upright stabilization

The full-state feedback controller in Sec. 11 uses the linearized model about upright, which neglects the nonlinear coupling terms present in the true pendulum dynamics. For operation near upright with moderate base velocities, these nonlinear effects remain active and can be exploited to improve performance. This section derives a **sliding mode controller (SMC)** that directly uses the nonlinear plant model (Sec. 6) while providing robust stabilization with finite-time reaching to a sliding surface.

Unlike the linearized designs (Sec. 9–11), SMC does **not** require small-angle approximations for the pendulum equation; it handles $\sin\alpha$ and $\cos\alpha$ exactly within a specified validity region. The design consists of three steps: (1) define a sliding surface that encodes the desired pendulum behavior, (2) derive a control law that drives the system to that surface in finite time, and (3) implement a boundary layer to eliminate chattering while preserving practical stability.

### 12.1 Nonlinear plant model for control design

From Sec. 6, the second Euler–Lagrange equation is

$$K\cos\alpha\,\ddot\theta + \hat J_2\ddot\alpha - \frac{1}{2}\hat J_2\sin(2\alpha)\dot\theta^2 - G\sin\alpha = 0$$

Solve explicitly for $\ddot\alpha$:

$$\hat J_2\ddot\alpha = G\sin\alpha + \frac{1}{2}\hat J_2\sin(2\alpha)\dot\theta^2 - K\cos\alpha\,\ddot\theta$$

Divide by $\hat J_2$:

$$\ddot\alpha = \frac{G}{\hat J_2}\sin\alpha + \frac{1}{2}\sin(2\alpha)\dot\theta^2 - \frac{K}{\hat J_2}\cos\alpha\,\ddot\theta$$

Use the double-angle identity $\sin(2\alpha)=2\sin\alpha\cos\alpha$:

$$\ddot\alpha = \frac{G}{\hat J_2}\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - \frac{K}{\hat J_2}\cos\alpha\,\ddot\theta$$

Define the constants (as in Sec. 8–10):

$$A\equiv \frac{G}{\hat J_2},\qquad B\equiv \frac{K}{\hat J_2}$$

Then the nonlinear plant model is

$$\ddot\alpha = A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - B\cos\alpha\,\ddot\theta$$

This is the **exact** pendulum dynamics (no linearization). The control input is $u\equiv \ddot\theta$, the commanded arm angular acceleration. The nonlinear terms $A\sin\alpha$ (gravity) and $\sin\alpha\cos\alpha\,\dot\theta^2$ (centrifugal coupling from base rotation) are treated explicitly.

Numerically (Sec. 8):

$$A=100.8\ \mathrm{rad/s^2},\qquad B=1.952\ \mathrm{(dimensionless)}$$

### 12.2 Sliding surface definition

Define the **sliding surface** as

$$s\equiv \dot\alpha + \lambda\alpha$$

where $\lambda>0$ is a design parameter with units of $\mathrm{rad/s}$ (or $\mathrm{s}^{-1}$). This is a linear manifold in the $(\alpha,\dot\alpha)$ phase plane.

**Interpretation:** If the system reaches the surface ($s=0$), then

$$\dot\alpha = -\lambda\alpha$$

This is a stable first-order differential equation. Solve by separation of variables:

$$\frac{d\alpha}{\alpha}=-\lambda\,dt$$

Integrate:

$$\ln|\alpha|=-\lambda t + C$$

Exponentiate:

$$\alpha(t)=\alpha_0 e^{-\lambda t}$$

where $\alpha_0=\alpha(0)$. Thus, **once on the sliding surface, the pendulum angle converges to zero exponentially with time constant $\tau=1/\lambda$**. The sliding surface encodes the desired transient behavior: larger $\lambda$ gives faster convergence but requires higher control authority.

The SMC objective is to drive $s\to 0$ in finite time (reaching phase) and then maintain $s\approx 0$ (sliding phase), thereby ensuring $\alpha\to 0$ exponentially.

### 12.3 Sliding surface dynamics

Differentiate the sliding surface with respect to time:

$$\dot s = \frac{d}{dt}(\dot\alpha+\lambda\alpha)=\ddot\alpha + \lambda\dot\alpha$$

Substitute the plant model (Sec. 12.1):

$$\dot s = A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - B\cos\alpha\,\ddot\theta + \lambda\dot\alpha$$

Rearrange to isolate the control term:

$$\dot s = \left(A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 + \lambda\dot\alpha\right) - B\cos\alpha\,\ddot\theta$$

Define the **nonlinear drift function**:

$$f(\alpha,\dot\alpha,\dot\theta)\equiv A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 + \lambda\dot\alpha$$

Then

$$\dot s = f - B\cos\alpha\,\ddot\theta$$

This shows how the sliding surface changes due to: (1) gravitational torque ($A\sin\alpha$), (2) centrifugal coupling ($\sin\alpha\cos\alpha\,\dot\theta^2$), (3) transient dynamics ($\lambda\dot\alpha$), and (4) control acceleration ($-B\cos\alpha\,\ddot\theta$).

### 12.4 Equivalent control

The **equivalent control** $\ddot\theta_{eq}$ is the control that makes $\dot s=0$ exactly, maintaining the system on the sliding surface. Set $\dot s=0$:

$$0 = f - B\cos\alpha\,\ddot\theta_{eq}$$

Solve for $\ddot\theta_{eq}$:

$$B\cos\alpha\,\ddot\theta_{eq} = f$$

Divide by $B\cos\alpha$ (assuming $\cos\alpha\neq 0$):

$$\ddot\theta_{eq} = \frac{f}{B\cos\alpha}$$

Substitute the definition of $f$:

$$\ddot\theta_{eq} = \frac{A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 + \lambda\dot\alpha}{B\cos\alpha}$$

Factor $\sin\alpha$ from the first two terms in the numerator:

$$\ddot\theta_{eq} = \frac{\sin\alpha(A+\cos\alpha\,\dot\theta^2) + \lambda\dot\alpha}{B\cos\alpha}$$

This can also be written as

$$\ddot\theta_{eq} = \frac{A\sin\alpha}{B\cos\alpha} + \frac{\sin\alpha\,\dot\theta^2}{B} + \frac{\lambda\dot\alpha}{B\cos\alpha}$$

The equivalent control cancels the nonlinear drift exactly. However, it does not guarantee that the system **reaches** the surface from arbitrary initial conditions. For that, a reaching law is required.

### 12.5 Reaching law and complete control law

To ensure finite-time convergence to the sliding surface, augment the equivalent control with a **reaching term**. A standard reaching law is

$$\dot s = -K\cdot\mathrm{sgn}(s)$$

where $K>0$ is the reaching gain (units: same as $\dot s$, i.e., $\mathrm{rad/s^2}$ or $\mathrm{s}^{-2}$), and $\mathrm{sgn}$ is the signum function:

$$\mathrm{sgn}(s)=\begin{cases}+1 & s>0\\0 & s=0\\-1 & s<0\end{cases}$$

This law drives $s$ toward zero at constant rate $K$. Combine with the plant dynamics:

$$f - B\cos\alpha\,\ddot\theta = -K\cdot\mathrm{sgn}(s)$$

Solve for $\ddot\theta$:

$$B\cos\alpha\,\ddot\theta = f + K\cdot\mathrm{sgn}(s)$$

$$\ddot\theta = \frac{f + K\cdot\mathrm{sgn}(s)}{B\cos\alpha}$$

Expand $f$:

$$\ddot\theta = \frac{A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 + \lambda\dot\alpha + K\cdot\mathrm{sgn}(s)}{B\cos\alpha}$$

This is the **complete sliding mode control law** for the rotary inverted pendulum. It consists of:

1. **Gravitational compensation**: $A\sin\alpha/(B\cos\alpha)$ cancels the unstable pendulum torque.
2. **Centrifugal compensation**: $\sin\alpha\,\dot\theta^2/B$ cancels the coupling from base rotation.
3. **Sliding surface dynamics**: $\lambda\dot\alpha/(B\cos\alpha)$ enforces the desired convergence rate on the surface.
4. **Reaching control**: $K\cdot\mathrm{sgn}(s)/(B\cos\alpha)$ drives the system to the surface.

**Sign convention note:** The sign of the control must match the physical actuator convention. If the implemented system uses $\alpha=0$ upright with a particular sign convention, an overall sign flip (implemented via a tunable parameter) may be required.

### 12.6 Lyapunov stability analysis

Define the Lyapunov function

$$V(s)=\frac{1}{2}s^2$$

This is a measure of the distance from the sliding surface. $V\geq 0$ with $V=0$ iff $s=0$. Differentiate with respect to time:

$$\dot V = s\dot s$$

Substitute the reaching law $\dot s=-K\cdot\mathrm{sgn}(s)$:

$$\dot V = s\cdot(-K\cdot\mathrm{sgn}(s))=-K\,s\,\mathrm{sgn}(s)$$

By definition of the signum function:

$$s\,\mathrm{sgn}(s)=|s|$$

Thus

$$\dot V = -K|s|$$

Since $K>0$, we have $\dot V<0$ for all $s\neq 0$. This proves that $V$ decreases monotonically, and the system reaches $s=0$ in finite time. Once on the surface, the equivalent control maintains $s=0$ and $\alpha\to 0$ exponentially (Sec. 12.2).

**Finite-time reaching:** Integrate $\dot V=-K|s|$. For $s>0$:

$$\frac{dV}{dt}=-K\sqrt{2V}$$

$$\frac{dV}{\sqrt{V}}=-\sqrt{2}K\,dt$$

$$2\sqrt{V}=-\sqrt{2}K\,t+C$$

If $V(0)=V_0$, then $C=2\sqrt{V_0}$, giving

$$\sqrt{V(t)}=\sqrt{V_0}-\frac{K}{\sqrt{2}}t$$

The surface is reached when $V=0$, i.e., at time

$$t_r=\sqrt{\frac{2V_0}{K^2}}=\frac{\sqrt{2}|s_0|}{K}$$

where $s_0=s(0)$. Larger $K$ reduces reaching time but increases control effort and can excite unmodeled dynamics.

### 12.7 Boundary layer for chattering elimination

The signum function $\mathrm{sgn}(s)$ is discontinuous at $s=0$. In practice, this causes **chattering**: rapid switching of the control signal due to finite sampling rate, actuator dynamics, and measurement noise. Chattering is undesirable as it excites high-frequency unmodeled dynamics, increases wear, and reduces efficiency.

To eliminate chattering, replace the signum function with a **continuous approximation** inside a **boundary layer** of thickness $\phi>0$. Define the saturation function:

$$\mathrm{sat}(x)=\begin{cases}+1 & x>1\\x & |x|\leq 1\\-1 & x<-1\end{cases}$$

Then replace $\mathrm{sgn}(s)$ with $\mathrm{sat}(s/\phi)$ in the reaching law:

$$\dot s = -K\cdot\mathrm{sat}\left(\frac{s}{\phi}\right)$$

The modified control law is

$$\ddot\theta = \frac{A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 + \lambda\dot\alpha + K\cdot\mathrm{sat}(s/\phi)}{B\cos\alpha}$$

**Behavior:**

- **Outside the boundary layer** ($|s|>\phi$): $\mathrm{sat}(s/\phi)=\pm 1$, behavior is identical to the original reaching law.
- **Inside the boundary layer** ($|s|\leq\phi$): $\mathrm{sat}(s/\phi)=s/\phi$, giving $\dot s=-Ks/\phi$, which is a stable linear dynamics. The system converges to a region $|s|\leq\phi$ instead of exactly $s=0$.

The boundary layer trades exact tracking ($s\equiv 0$) for chattering elimination. The steady-state error in $\alpha$ is bounded by $|\alpha|\lesssim \phi/\lambda$ (since $s=\dot\alpha+\lambda\alpha$ and $|s|\leq\phi$). Choose $\phi$ large enough to smooth the control but small enough to keep tracking error acceptable.

**Modified Lyapunov analysis:** Inside the boundary layer ($|s|\leq\phi$):

$$\dot V = s\dot s = -K\frac{s^2}{\phi}=-\frac{K}{\phi}s^2=-\frac{2K}{\phi}V$$

This shows **exponential convergence** to $s=0$ with rate $K/\phi$ once inside the layer. For $|s|>\phi$, the finite-time reaching analysis (Sec. 12.6) still applies.

### 12.8 Numerical parameter selection (using the identified plant constants)

Use the numerical plant values (Sec. 10.2):

$$A=100.8\ \mathrm{rad/s^2},\qquad B=1.952$$

Choose design parameters:

- **Sliding surface slope**: $\lambda=15.0\ \mathrm{rad/s}$ (same bandwidth as the fast mode in Sec. 11.6, giving $\tau=1/\lambda=67\ \mathrm{ms}$ time constant on the surface).
- **Reaching gain**: $K=800.0\ \mathrm{deg/s^2}$ (in degree units for implementation; converted below).
- **Boundary layer thickness**: $\phi=50.0\ \mathrm{deg/s}$ (in degree units; suppresses chattering while keeping steady-state error small).

**Unit conversions:** The plant constants $A$ and $B$ are in rad-based units. For implementation in degree variables, convert $A$:

$$A_{\deg}=A\cdot\frac{180}{\pi}=100.8\cdot\frac{180}{\pi}=5779.2\ \mathrm{deg/s^2}$$

$B$ is dimensionless and unchanged. The sliding surface in degree variables is

$$s_{\deg}=\dot\alpha_{\deg}+\lambda\,\alpha_{\deg}$$

where $\lambda=15.0\ \mathrm{s}^{-1}$ (dimensionally consistent in both rad and deg systems since it multiplies angle and rate with the same units).

The drift function in degree variables is

$$f_{\deg}=A_{\deg}\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta_{\deg}^2\cdot\frac{\pi}{180} + \lambda\dot\alpha_{\deg}$$

However, note that the $\dot\theta^2$ term requires careful units: if $\dot\theta_{\deg}$ is in $\mathrm{deg/s}$, then $\dot\theta_{\deg}^2$ is in $\mathrm{deg^2/s^2}$, but the centrifugal term in the plant model expects $\mathrm{rad^2/s^2}$. The conversion is

$$\dot\theta_{\mathrm{rad}}=\dot\theta_{\deg}\cdot\frac{\pi}{180}$$

so

$$\dot\theta_{\mathrm{rad}}^2=\dot\theta_{\deg}^2\cdot\left(\frac{\pi}{180}\right)^2$$

Thus, in degree variables, the control law is

$$\ddot\theta_{\deg} = \frac{A_{\deg}\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta_{\deg}^2\cdot(\pi/180)^2 + \lambda\dot\alpha_{\deg} + K\cdot\mathrm{sat}(s_{\deg}/\phi)}{B\cos\alpha}$$

Simplify $(\pi/180)^2=0.0003046174$. The term $\sin\alpha\cos\alpha\,\dot\theta_{\deg}^2\cdot 0.0003046174$ is the centrifugal coupling in mixed units. For numerical robustness, track $\dot\theta$ internally in the same unit system as $\alpha$.

**Alternative (radian-based internal computation):** Convert sensor readings to radians, compute control in $\mathrm{rad/s^2}$, then convert output to $\mathrm{steps/s^2}$. This avoids unit confusion. However, for consistency with the existing firmware (which uses degree variables for state feedback), the hybrid approach is:

1. Sensor angles in degrees: $\alpha_{\deg}$, $\dot\alpha_{\deg}$, $\dot\theta_{\deg}$.
2. Convert $A$ to degree units: $A_{\deg}=5779.2$ (but this is large and numerically awkward; typically the conversion is implicit).
3. In practice, the firmware uses $A=100.8$ (radian-based) and computes the centrifugal term as $\sin\alpha\cos\alpha\,\dot\theta^2$ where $\alpha$ is in radians derived from degrees.

**Simplified implementation choice (as in the actual firmware):** Accept $\alpha$ and $\dot\theta$ in degrees, convert to radians for the nonlinear functions:

$$\alpha_{\mathrm{rad}}=\alpha_{\deg}\cdot\frac{\pi}{180}$$
$$\dot\theta_{\mathrm{rad}}=\dot\theta_{\deg}\cdot\frac{\pi}{180}$$

Compute

$$f_{\mathrm{rad}}=A\sin(\alpha_{\mathrm{rad}}) + \sin(\alpha_{\mathrm{rad}})\cos(\alpha_{\mathrm{rad}})\,\dot\theta_{\mathrm{rad}}^2 + \lambda\dot\alpha_{\mathrm{rad}}$$

where $\dot\alpha_{\mathrm{rad}}=\dot\alpha_{\deg}\cdot(\pi/180)$ and $A=100.8\ \mathrm{rad/s^2}$. However, dimensional analysis shows the result $f_{\mathrm{rad}}$ is in $\mathrm{rad/s^2}$. For output in $\mathrm{deg/s^2}$:

$$f_{\deg}=f_{\mathrm{rad}}\cdot\frac{180}{\pi}$$

Then

$$\ddot\theta_{\deg}=\frac{f_{\deg}+K\cdot\mathrm{sat}(s_{\deg}/\phi)}{B\cos(\alpha_{\mathrm{rad}})}$$

**Practical numerical values (degree-based for $K$ and $\phi$):**

With $\lambda=15.0$, $K=800.0\ \mathrm{deg/s^2}$, $\phi=50.0\ \mathrm{deg/s}$, the steady-state sliding surface error is bounded by

$$|s_{\deg}|\lesssim\phi=50\ \mathrm{deg/s}$$

Inside the boundary layer, this gives $|\alpha_{\deg}|\lesssim 50/15=3.3\ \mathrm{deg}$ steady-state pendulum angle error (an overestimate; in practice much smaller due to the continuous approach within the layer).

### 12.9 Conversion to stepper units (degree variables)

The commanded acceleration $\ddot\theta_{\deg}$ (in $\mathrm{deg/s^2}$) must be converted to stepper acceleration (in $\mathrm{steps/s^2}$). From Sec. 9, the conversion factor is

$$\mathrm{steps/deg}=\frac{N}{360}=\frac{1600}{360}=4.444$$

Thus

$$\ddot\theta_{\mathrm{steps}}=4.444\,\ddot\theta_{\deg}$$

Substitute the control law:

$$\ddot\theta_{\mathrm{steps}}=4.444\cdot\frac{f_{\deg}+K\cdot\mathrm{sat}(s_{\deg}/\phi)}{B\cos\alpha}$$

where

$$f_{\deg}=A_{\deg}\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta_{\deg}^2\cdot\left(\frac{\pi}{180}\right)^2 + \lambda\dot\alpha_{\deg}$$

Alternatively, using radian-based computation internally (cleaner):

$$f_{\mathrm{rad/s^2}}=100.8\sin\alpha_{\mathrm{rad}} + \sin\alpha_{\mathrm{rad}}\cos\alpha_{\mathrm{rad}}\,\dot\theta_{\mathrm{rad}}^2$$

Convert to $\mathrm{deg/s^2}$:

$$f_{\deg}=f_{\mathrm{rad}}\cdot\frac{180}{\pi} + \lambda\dot\alpha_{\deg}$$

Then

$$\ddot\theta_{\mathrm{steps}}=4.444\cdot\frac{f_{\deg}+K\cdot\mathrm{sat}(s_{\deg}/\phi)}{B\cos\alpha_{\mathrm{rad}}}$$

**Implementable formula (matching firmware):**

Let $\deg2\mathrm{rad}=\pi/180=0.017453$ and $\mathrm{rad}2\deg=180/\pi=57.2958$. Compute:

1. $\alpha_{\mathrm{rad}}=\alpha_{\deg}\cdot\deg2\mathrm{rad}$
2. $\dot\theta_{\mathrm{rad}}=\dot\theta_{\deg}\cdot\deg2\mathrm{rad}$
3. $\sin\alpha=\sin(\alpha_{\mathrm{rad}})$, $\cos\alpha=\cos(\alpha_{\mathrm{rad}})$
4. $f_{\mathrm{rad}}=A\sin\alpha+\sin\alpha\cos\alpha\,\dot\theta_{\mathrm{rad}}^2$
5. $f_{\deg}=f_{\mathrm{rad}}\cdot\mathrm{rad}2\deg$
6. $s_{\deg}=\dot\alpha_{\deg}+\lambda\alpha_{\deg}$
7. $\mathrm{sat}(s_{\deg}/\phi)=\min(1,\max(-1,s_{\deg}/\phi))$
8. $\ddot\theta_{\deg}=\dfrac{f_{\deg}+\lambda\dot\alpha_{\deg}+K\cdot\mathrm{sat}(s_{\deg}/\phi)}{B\cos\alpha}$
9. $\ddot\theta_{\mathrm{steps}}=4.444\,\ddot\theta_{\deg}$

### 12.10 Implementation notes and practical considerations

**Validity region:** SMC is valid near upright where $\cos\alpha$ is bounded away from zero. The implemented controller enforces $|\alpha|<25^\circ$ (0.436 rad) to ensure $\cos\alpha\geq\cos(25^\circ)=0.906$, well above zero. Outside this region, the control law experiences numerical issues (division by small $\cos\alpha$) and the model assumptions break down (swing-up would require energy-based control, not local stabilization).

**Singularity protection:** If $\cos\alpha\to 0$, the control command diverges. Implement a safeguard:

$$\cos\alpha_{\mathrm{safe}}=\max(|\cos\alpha|,\cos\alpha_{\min})\cdot\mathrm{sign}(\cos\alpha)$$

where $\cos\alpha_{\min}=0.2$ (corresponding to $|\alpha|\approx 78^\circ$, well outside the validity window). This clamps the denominator and prevents numerical overflow if the pendulum approaches horizontal.

**Derivative measurement:** $\dot\alpha$ and $\dot\theta$ are computed via the bilinear-transform derivative filter (Sec. 11.9, firmware implementation). Noise in $\dot\alpha$ directly affects $s$ and hence the switching behavior. The boundary layer $\phi$ must be chosen larger than the peak derivative noise to avoid residual chattering.

**Centrifugal coupling significance:** The term $\sin\alpha\cos\alpha\,\dot\theta^2$ is significant when the base rotates rapidly (e.g., during aggressive recentering maneuvers or large disturbances). For typical operation near upright with $|\dot\theta|<5\ \mathrm{deg/s}=0.087\ \mathrm{rad/s}$, this term contributes $\lesssim 0.087^2\sin\alpha\cos\alpha\approx 0.0076\sin\alpha\cos\alpha\ \mathrm{rad/s^2}$, which is small compared to $A\sin\alpha=100.8\sin\alpha$. However, during rapid base motion ($|\dot\theta|\sim 50\ \mathrm{deg/s}$), the coupling is non-negligible and improves disturbance rejection.

**Parameter tuning:**

- Increase $\lambda$ for faster pendulum convergence (higher bandwidth), but this increases the required control authority and sensitivity to noise.
- Increase $K$ for faster reaching to the sliding surface, but excessive $K$ causes large control spikes and can saturate the actuator.
- Increase $\phi$ to further smooth the control and eliminate chattering, but this increases the steady-state tracking error.
- For this hardware ($\mathrm{steps/s^2}$ limited to $\pm 20{,}000$), the chosen parameters provide good balance: $\lambda=15$ gives 67 ms time constant, $K=800\ \mathrm{deg/s^2}=3555\ \mathrm{steps/s^2}$ (well below saturation), $\phi=50\ \mathrm{deg/s}$ eliminates chattering with $<2^\circ$ steady-state error.

**Base tracking blend (secondary objective):** The SMC law focuses solely on pendulum stabilization. To simultaneously drive the base toward a reference position $\theta_{\mathrm{ref}}$, the implementation adds a **gated base-centering term**:

$$\ddot\theta_{\mathrm{base}}=\ddot\theta_{\mathrm{ref}} + k_\theta(\theta_{\mathrm{ref}}-\theta) + k_{\dot\theta}(\dot\theta_{\mathrm{ref}}-\dot\theta)$$

where $k_\theta$ and $k_{\dot\theta}$ are the base position/velocity gains from the linear controller (Sec. 11.7). This term is scaled by a tunable factor $0\leq s_{\mathrm{scale}}\leq 2$ (default 1.0) and gated to be active only when the pendulum is stable:

$$|\alpha|<5^\circ,\qquad |\dot\alpha|<150\ \mathrm{deg/s}$$

Note that this gate uses a **looser $\dot\alpha$ threshold** (150 deg/s) compared to the trapezoidal move-profile gate (80 deg/s), allowing base centering to remain active during typical transients while still ensuring the SMC law dominates during large disturbances. The scale factor $s_{\mathrm{scale}}$ trades off between pure pendulum stabilization ($s_{\mathrm{scale}}=0$) and aggressive base tracking ($s_{\mathrm{scale}}>1$); values above 1.0 prioritize recentering over upright stiffness and are useful for drift rejection but can destabilize if too large.

**Code mapping:** The firmware implements this control law in the `CTRL_SMC` branch (lines 1430–1475 of `src/main.cpp`):

```cpp
if (ctrlMode == CTRL_SMC) {
  const float deg2rad = PI / 180.0f;
  const float rad2deg = 180.0f / PI;

  const float alphaDeg = alphaRawSigned;
  const float alphaDotDegS = alphaDotFilt;
  const float thetaDotDegS = thetaDotFilt;

  const float alphaRad = alphaDeg * deg2rad;
  const float sinA = sin(alphaRad);
  const float cosA = cos(alphaRad);
  const float cosSafe = copysignf(max(fabs(cosA), SMC_COS_MIN), cosA);

  // f_deg term: A*sin(alpha) converted to deg/s^2,
  // plus centrifugal coupling (in rad^2/s^2 → deg/s^2)
  const float A_deg = MODEL_A * rad2deg;  // 100.8 * 57.3 = 5779 deg/s^2
  const float fDeg =
      A_deg * sinA +
      (sinA * cosA) * (thetaDotDegS * thetaDotDegS) * deg2rad;

  // Sliding surface
  smcS = alphaDotDegS + smcLambda * alphaDeg;
  const float phi = max(smcPhiDegS, 1e-3f);
  const float smcSat = constrain(smcS / phi, -1.0f, 1.0f);

  // Control law (deg/s^2)
  thetaDDotSmcDegS2 =
      (fDeg + smcLambda * alphaDotDegS + smcKDegS2 * smcSat) /
      (MODEL_B * cosSafe);

  thetaDDotSmcDegS2 *= smcSign;  // overall sign flip if needed
  thetaDDotSmcDegS2 = constrain(thetaDDotSmcDegS2, -THETA_DDOT_MAX_DEG_S2, THETA_DDOT_MAX_DEG_S2);

  const float accSMC_steps_s2 = STEPS_PER_DEG * thetaDDotSmcDegS2;

  // Secondary objective: gentle base tracking (reference + feedforward)
  float accBase_steps_s2 =
      accFF_steps_s2 +
      K_THETA * thetaErr +
      K_THETADOT * thetaDotErr;

  accBase_steps_s2 *= constrain(smcBaseScale, 0.0f, 2.0f);  // scale 0..2
  
  // Gate with looser thresholds than move-profile (allows centering during transients)
  const bool smcBaseGate =
      engageGraceOver &&
      (fabs(alphaRawSigned) < 5.0f) &&             // SMC_BASE_GATE_ALPHA_WINDOW_DEG
      (fabs(alphaDotFilt) < 150.0f);               // SMC_BASE_GATE_ALPHADOT_WINDOW_DEG_S
  if (!smcBaseGate) accBase_steps_s2 = 0.0f;

  accCmdPhysical = accSMC_steps_s2 + accBase_steps_s2;
}
```

The runtime-tunable parameter `smcBaseScale` (command `O <val>`, range 0–2, default 1.0) controls the blend intensity. Status output includes `accSMC`, `accBase`, and `gate` for real-time visibility into the control decomposition.

**Comparison with linear controllers (Sec. 9–11):**

- **SMC advantage**: Explicitly handles $\sin\alpha$ and centrifugal coupling; robust to parameter uncertainty (via discontinuous control); finite-time reaching guarantees.
- **Linear advantage**: Simpler gains (no trig functions in real-time loop); easier to tune (just pole locations); globally valid if linearization errors are small.
- **Performance**: For small angles ($|\alpha|<5^\circ$), both approaches perform similarly. For larger disturbances or faster base motion, SMC provides better tracking by leveraging the true nonlinear dynamics.

The implemented system supports mode selection via `C <0|1|2>` **when disarmed** (i.e., not in `STATE_ACTIVE`), enabling direct performance comparison under identical experiment protocols without introducing mode-switch transients or serial-parse stalls in the control loop.

---

## 13. Full-state surface Sliding Mode Control for upright stabilization with base reference tracking

The nonlinear SMC law derived in Sec. 12 uses a sliding surface defined solely by the pendulum states $(\alpha,\dot\alpha)$, with base centering implemented as a separate gated blend term. This section extends the SMC framework by defining a **full-state sliding surface** that incorporates all four system states—pendulum angle, pendulum velocity, base position error, and base velocity error—into a single scalar manifold.

The resulting controller is implemented in firmware as `CTRL_SMC_FULL` (command `C 2`, status label `mode=SMC4`). In theory, this full-state surface can unify upright stabilization and base reference tracking in a single SMC law. In practice (on this stepper-driven rig), two additional implementation details are critical for repeatable experiments:

1. **Reference-aware formulation:** all $\theta$ terms use the same reference-tracking coordinates as Sec. 11–12, i.e., $\theta_{\mathrm{err}}=\theta-\theta_{\mathrm{ref}}$ and $\dot\theta_{\mathrm{err}}=\dot\theta-\dot\theta_{\mathrm{ref}}$, where $(\theta_{\mathrm{ref}},\dot\theta_{\mathrm{ref}},\ddot\theta_{\mathrm{ref}})$ come from the trapezoidal profile generator (Sec. 12.10, `tools/base_tracking.md`).
2. **Stepper-friendly robustness additions:** the firmware retains a **gated base-centering assist term** (PD+FF, scaled by `O`) and applies SMC4-only command shaping (tighter accel/speed caps, plus an $\dot\alpha$ clamp inside the SMC4 math). These additions mitigate slow base “random-walk” drift and reduce missed-step/stall failures under finger-tap disturbances.

**Design objectives:**

- Stabilize the pendulum at upright equilibrium ($\alpha\to 0$).
- Track a base reference trajectory $\theta_{\mathrm{ref}}(t)$ provided by the trapezoidal motion profile generator.
- Maintain SMC robustness characteristics (finite-time reaching, boundary layer chattering suppression).
- Ensure numerical safety through denominator protection and validity region enforcement.

### 13.1 Plant model and reference tracking formulation

The nonlinear pendulum dynamics (Sec. 12.1) remain the foundation:

$$\ddot\alpha = A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - B\cos\alpha\,\ddot\theta$$

where the plant constants are

$$A\equiv \frac{G}{\hat J_2},\qquad B\equiv \frac{K}{\hat J_2}$$

with numerical values (Sec. 8)

$$A=100.8\ \mathrm{rad/s^2},\qquad B=1.952$$

The control input is the commanded arm angular acceleration:

$$u \equiv \ddot\theta$$

**Reference trajectory:** A desired base trajectory $\theta_{\mathrm{ref}}(t)$ is specified with its derivatives $\dot\theta_{\mathrm{ref}}(t)$ and $\ddot\theta_{\mathrm{ref}}(t)$ (generated via trapezoidal velocity profile). For regulation to upright with base centering, $\theta_{\mathrm{ref}}(t)\equiv 0$.

**Tracking error coordinates:** Define

$$\theta_{\mathrm{err}} \equiv \theta - \theta_{\mathrm{ref}},\qquad
\dot\theta_{\mathrm{err}} \equiv \dot\theta - \dot\theta_{\mathrm{ref}}$$

so that

$$\ddot\theta_{\mathrm{err}} = \ddot\theta - \ddot\theta_{\mathrm{ref}}$$

### 13.2 Full-state sliding surface definition

Define the **full-state sliding surface** as a scalar function of all four system states:

$$s \equiv \dot\alpha + \lambda_\alpha\,\alpha + k\left(\dot\theta_{\mathrm{err}} + \lambda_\theta\,\theta_{\mathrm{err}}\right)$$

where the design parameters are:

- $\lambda_\alpha>0$ (units: $\mathrm{s^{-1}}$): pendulum angle convergence rate,
- $\lambda_\theta>0$ (units: $\mathrm{s^{-1}}$): base error shaping parameter,
- $k\ge 0$ (dimensionless): coupling weight between pendulum and base dynamics.

**Interpretation:** The surface $s=0$ defines a three-dimensional manifold in the four-dimensional state space. When the system reaches and remains on this surface, the constrained dynamics are

$$\dot\alpha = -\lambda_\alpha\,\alpha - k\left(\dot\theta_{\mathrm{err}} + \lambda_\theta\,\theta_{\mathrm{err}}\right)$$

**Special cases:**

1. **Pure upright stabilization** ($k=0$): The surface reduces to $s=\dot\alpha+\lambda_\alpha\alpha$, recovering the pendulum-only SMC (Sec. 12.2). On the surface, $\dot\alpha=-\lambda_\alpha\alpha$ gives exponential decay $\alpha(t)=\alpha_0 e^{-\lambda_\alpha t}$.

2. **Perfect base tracking** ($\theta_{\mathrm{err}}=\dot\theta_{\mathrm{err}}=0$): The base terms vanish and the pendulum dynamics are identical to case 1.

3. **Nonzero coupling** ($k>0$): Base tracking errors $\theta_{\mathrm{err}},\dot\theta_{\mathrm{err}}$ influence the pendulum sliding manifold, creating a unified control objective where pendulum stabilization and base tracking are coupled through the single surface $s$.

### 13.3 Sliding surface dynamics and control effectiveness

Differentiate the sliding surface with respect to time:

\begin{align}
\dot s
&= \frac{d}{dt}\left(\dot\alpha + \lambda_\alpha\alpha + k\dot\theta_{\mathrm{err}} + k\lambda_\theta\theta_{\mathrm{err}}\right)\\
&= \ddot\alpha + \lambda_\alpha\dot\alpha + k\ddot\theta_{\mathrm{err}} + k\lambda_\theta\dot\theta_{\mathrm{err}}
\end{align}

Substitute the error acceleration $\ddot\theta_{\mathrm{err}}=\ddot\theta-\ddot\theta_{\mathrm{ref}}$:

$$\dot s = \ddot\alpha + \lambda_\alpha\dot\alpha + k(\ddot\theta-\ddot\theta_{\mathrm{ref}}) + k\lambda_\theta\dot\theta_{\mathrm{err}}$$

Now substitute the nonlinear plant model for $\ddot\alpha$:

\begin{align}
\dot s
&= \left(A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - B\cos\alpha\,\ddot\theta\right) + \lambda_\alpha\dot\alpha\\
&\quad + k\ddot\theta - k\ddot\theta_{\mathrm{ref}} + k\lambda_\theta\dot\theta_{\mathrm{err}}
\end{align}

Collect all terms involving the control input $\ddot\theta$:

\begin{align}
\dot s
&= \underbrace{A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2}_{\triangleq f_0(\alpha,\dot\theta)}
  + \lambda_\alpha\dot\alpha + k\lambda_\theta\dot\theta_{\mathrm{err}} - k\ddot\theta_{\mathrm{ref}}\\
&\quad + (k - B\cos\alpha)\ddot\theta
\end{align}

Define the **drift function**

$$f_{\mathrm{full}}(\alpha,\dot\alpha,\dot\theta,\dot\theta_{\mathrm{err}}) \equiv f_0(\alpha,\dot\theta) + \lambda_\alpha\dot\alpha + k\lambda_\theta\dot\theta_{\mathrm{err}} - k\ddot\theta_{\mathrm{ref}}$$

where $f_0(\alpha,\dot\theta)=A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2$ represents the unforced pendulum dynamics (gravitational and centrifugal terms). Then the surface dynamics are

$$\dot s = f_{\mathrm{full}} + (k - B\cos\alpha)\ddot\theta$$

**Control effectiveness analysis:** The coefficient $(k-B\cos\alpha)$ multiplies the control input $\ddot\theta$. Rearranging:

$$\ddot\theta\text{ contributes to }\dot s\text{ with gain }(k-B\cos\alpha)$$

For the control to influence $\dot s$ with the correct sign, we require

$$(B\cos\alpha - k) > 0$$

Near upright, $\cos\alpha\approx 1$, so $B\cos\alpha\approx B=1.952$. Therefore:

- If $k=0$ (no base coupling), the effectiveness is $B\cos\alpha$, which is the same as the pendulum-only SMC (Sec. 12).
- If $0<k<B\cos\alpha$, the effective gain is reduced but remains positive. Larger $k$ reduces the denominator, requiring larger $\ddot\theta$ to achieve the same $\dot s$.
- If $k\to B\cos\alpha$, the effectiveness vanishes and the control law becomes singular.
- If $k>B\cos\alpha$, the sign of the effectiveness reverses, invalidating the reaching law stability analysis.

**Safety constraint:** To ensure well-conditioned control, enforce $k < B\cos\alpha$ with a safety margin (Sec. 13.7). For upright operation with $|\alpha|<25^\circ$, we have $\cos\alpha\ge 0.906$, giving $B\cos\alpha\ge 1.77$. A conservative limit is $k\le 1.2$, providing margin $B\cos\alpha-k\ge 0.57$.

### 13.4 Equivalent control

The **equivalent control** $\ddot\theta_{\mathrm{eq}}$ is the particular control input that maintains $\dot s=0$ exactly, thereby enforcing sliding mode operation on the surface $s=0$. Set $\dot s=0$ in the surface dynamics:

$$0 = f_{\mathrm{full}} + (k - B\cos\alpha)\ddot\theta_{\mathrm{eq}}$$

Solve for $\ddot\theta_{\mathrm{eq}}$:

$$(k - B\cos\alpha)\ddot\theta_{\mathrm{eq}} = -f_{\mathrm{full}}$$

$$(B\cos\alpha - k)\ddot\theta_{\mathrm{eq}} = f_{\mathrm{full}}$$

$$\ddot\theta_{\mathrm{eq}} = \frac{f_{\mathrm{full}}}{B\cos\alpha - k}$$

Expanding the drift function:

$$\ddot\theta_{\mathrm{eq}} = \frac{A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 + \lambda_\alpha\dot\alpha + k\lambda_\theta\dot\theta_{\mathrm{err}} - k\ddot\theta_{\mathrm{ref}}}{B\cos\alpha - k}$$

This equivalent control compensates for:

1. **Gravitational torque**: $A\sin\alpha$ (unstable upright equilibrium),
2. **Centrifugal coupling**: $\sin\alpha\cos\alpha\,\dot\theta^2$ (base rotation induced pendulum acceleration),
3. **Sliding surface transient shaping**: $\lambda_\alpha\dot\alpha$ (desired pendulum velocity decay),
4. **Base error dynamics**: $k\lambda_\theta\dot\theta_{\mathrm{err}}$ (base velocity error correction),
5. **Reference feedforward**: $-k\ddot\theta_{\mathrm{ref}}$ (acceleration tracking).

However, equivalent control alone does **not guarantee convergence to the surface** from arbitrary initial conditions. A reaching law is required.

### 13.5 Reaching law with boundary layer and complete control law

To ensure finite-time convergence to the sliding surface, augment the equivalent control with a **reaching term**. Use the same continuous approximation as Sec. 12.7 to eliminate chattering. The reaching law is

$$\dot s = -K\cdot\mathrm{sat}\!\left(\frac{s}{\phi}\right)$$

where $K>0$ is the reaching gain (units: $\mathrm{rad/s^2}$ or $\mathrm{deg/s^2}$), $\phi>0$ is the boundary layer thickness (units: $\mathrm{rad/s}$ or $\mathrm{deg/s}$), and the saturation function is

$$\mathrm{sat}(x)=\begin{cases}+1 & x>1\\ x & |x|\le 1\\ -1 & x<-1\end{cases}$$

**Behavior:**

- **Outside the boundary layer** ($|s|>\phi$): $\mathrm{sat}(s/\phi)=\pm 1$, giving $\dot s=-K\cdot\mathrm{sgn}(s)$. This drives the surface coordinate toward zero at constant rate $K$ (finite-time reaching).
- **Inside the boundary layer** ($|s|\le\phi$): $\mathrm{sat}(s/\phi)=s/\phi$, giving $\dot s=-Ks/\phi$. This is a stable linear dynamics with exponential convergence to $s=0$.

Substitute the reaching law into the surface dynamics:

$$-K\cdot\mathrm{sat}\!\left(\frac{s}{\phi}\right) = f_{\mathrm{full}} + (k - B\cos\alpha)\ddot\theta$$

Solve for the control input $\ddot\theta$:

$$(k - B\cos\alpha)\ddot\theta = -K\cdot\mathrm{sat}\!\left(\frac{s}{\phi}\right) - f_{\mathrm{full}}$$

$$(B\cos\alpha - k)\ddot\theta = f_{\mathrm{full}} + K\cdot\mathrm{sat}\!\left(\frac{s}{\phi}\right)$$

Thus the **complete full-state surface sliding mode control law** is

$$\boxed{
\ddot\theta
=
\frac{
f_0(\alpha,\dot\theta)
+ \lambda_\alpha\dot\alpha
+ k\lambda_\theta\dot\theta_{\mathrm{err}}
- k\ddot\theta_{\mathrm{ref}}
+ K\cdot\mathrm{sat}\!\left(\dfrac{s}{\phi}\right)
}{
B\cos\alpha - k
}
}$$

where

\begin{align}
f_0(\alpha,\dot\theta) &= A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2\\
s &= \dot\alpha + \lambda_\alpha\alpha + k(\dot\theta_{\mathrm{err}}+\lambda_\theta\theta_{\mathrm{err}})
\end{align}

**Components:**

1. $f_0(\alpha,\dot\theta)/(B\cos\alpha-k)$: nonlinear plant inversion (gravity + centrifugal compensation),
2. $\lambda_\alpha\dot\alpha/(B\cos\alpha-k)$: sliding surface transient shaping,
3. $k\lambda_\theta\dot\theta_{\mathrm{err}}/(B\cos\alpha-k)$: base velocity error coupling,
4. $-k\ddot\theta_{\mathrm{ref}}/(B\cos\alpha-k)$: reference acceleration feedforward,
5. $K\cdot\mathrm{sat}(s/\phi)/(B\cos\alpha-k)$: reaching control with chattering elimination.

**Reduction to Sec. 12:** Setting $k=0$ eliminates all base tracking terms and recovers the pendulum-only SMC law:

$$\ddot\theta
=
\frac{
A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2
+ \lambda_\alpha\dot\alpha
+ K\cdot\mathrm{sat}\!\left(\dfrac{\dot\alpha+\lambda_\alpha\alpha}{\phi}\right)
}{
B\cos\alpha
}$$

### 13.6 Lyapunov stability and finite-time reaching

Define the Lyapunov function candidate

$$V(s)=\frac{1}{2}s^2$$

This is a measure of the squared distance from the sliding surface. Clearly $V\geq 0$ with $V=0$ if and only if $s=0$. Differentiate with respect to time:

$$\dot V = s\dot s$$

Substitute the reaching law $\dot s=-K\cdot\mathrm{sat}(s/\phi)$:

$$\dot V = -K\,s\cdot\mathrm{sat}\!\left(\frac{s}{\phi}\right)$$

**Case 1: Outside the boundary layer** ($|s|>\phi$)

Here $\mathrm{sat}(s/\phi)=\mathrm{sgn}(s)$, so

$$\dot V = -K\,s\,\mathrm{sgn}(s) = -K|s|$$

Since $K>0$, we have $\dot V<0$ for all $s\neq 0$ outside the layer. This proves that $V$ decreases monotonically.

To find the **reaching time** to the boundary layer, note that from $s\dot s=-K|s|$ we have

$$\frac{d}{dt}\left(\frac{1}{2}s^2\right)=-K|s|$$

For $s>0$:

$$s\dot s = -Ks$$

$$\dot s = -K$$

$$s(t)=s_0-Kt$$

The boundary is reached when $s(t_r)=\phi$, giving

$$t_r=\frac{s_0-\phi}{K}\quad\text{(for }s_0>\phi\text{)}$$

If $s_0<0$, by symmetry $t_r=(|s_0|-\phi)/K$. Thus the **worst-case reaching time** to the boundary layer is

$$t_r=\frac{|s_0|-\phi}{K}$$

This is **finite-time reaching**: the system is guaranteed to enter the boundary layer $|s|\le\phi$ in finite time regardless of the initial condition $s_0$, provided the controller does not saturate.

**Case 2: Inside the boundary layer** ($|s|\le\phi$)

Here $\mathrm{sat}(s/\phi)=s/\phi$, so

$$\dot V = -K\,s\cdot\frac{s}{\phi}=-\frac{K}{\phi}s^2=-\frac{2K}{\phi}V$$

This is exponential convergence with rate $K/\phi$:

$$V(t)=V_0 e^{-2Kt/(\phi)}$$

which means

$$s(t)=s_0 e^{-Kt/(\phi)}$$

**Steady-state error bound:** On the sliding surface ($s\approx 0$), the pendulum angle satisfies

$$\dot\alpha \approx -\lambda_\alpha\alpha - k(\dot\theta_{\mathrm{err}}+\lambda_\theta\theta_{\mathrm{err}})$$

For perfect base tracking ($\theta_{\mathrm{err}}=\dot\theta_{\mathrm{err}}=0$), this gives $|\alpha|\lesssim|s|/\lambda_\alpha\le\phi/\lambda_\alpha$. With typical values $\phi=50\ \mathrm{deg/s}$ and $\lambda_\alpha=15\ \mathrm{s^{-1}}$, the steady-state pendulum angle error is bounded by $\approx 3.3^\circ$ (in practice, much smaller due to exponential convergence within the layer).

**Summary of stability properties:**

- **Global attractivity** (within the validity region): Any trajectory starting with $|s|>\phi$ reaches the boundary layer in finite time $t_r=(|s_0|-\phi)/K$.
- **Exponential convergence** inside the layer: Once $|s|\le\phi$, exponential decay to $s=0$ with time constant $\phi/K$.
- **Chattering suppression**: The continuous saturation function eliminates infinite-frequency switching while preserving convergence.

### 13.7 Implementation safeguards: denominator protection and coupling limits

The control law denominator $(B\cos\alpha-k)$ requires careful treatment to prevent numerical issues and ensure well-conditioned control. The implementation enforces multiple layers of protection:

**1. Validity region enforcement**

Restrict operation to the near-upright region where the linearization-based surface design is valid and $\cos\alpha$ is bounded safely away from zero:

$$|\alpha|<\alpha_{\mathrm{max}}=25^\circ$$

In this region, $\cos\alpha\ge\cos(25^\circ)=0.906$, giving $B\cos\alpha\ge 1.952\times 0.906\approx 1.77$.

**2. Cosine floor protection**

Enforce a minimum magnitude for $\cos\alpha$ to prevent division issues if the pendulum exceeds the validity window:

$$\cos\alpha_{\mathrm{safe}}=\max(|\cos\alpha|,\cos\alpha_{\min})\cdot\mathrm{sign}(\cos\alpha)$$

where $\cos\alpha_{\min}=0.2$ (corresponding to $|\alpha|\approx 78^\circ$, well outside normal operation).

**3. Coupling parameter hard limit**

Clamp the user-specified coupling to a conservative maximum:

$$k_{\mathrm{user}}\le k_{\mathrm{max,hard}}=1.20$$

This ensures $B\cos\alpha_{\mathrm{safe}}-k_{\mathrm{max,hard}}\ge 1.77-1.20=0.57$ in the validity region.

**4. Ramp-in after engagement**

To avoid transient kicks at control onset, ramp the effective coupling from zero to the requested value over a period $T_{\mathrm{ramp}}$:

$$k_{\mathrm{req}}(t)=k_{\mathrm{user}}\cdot\min\!\left(\frac{t-t_{\mathrm{engage}}}{T_{\mathrm{ramp}}},1\right)$$

**Firmware value:** $T_{\mathrm{ramp}}=200\ \mathrm{ms}$ (`SMC_FULL_K_RAMP_MS` in `src/main.cpp`).

**5. Denominator-safe effective coupling**

Enforce a minimum denominator margin $\mathrm{den}_{\min}>0$ (e.g., $\mathrm{den}_{\min}=0.60$):

$$k_{\mathrm{eff}}=\max\!\left(0,\min\!\left(k_{\mathrm{req}},\ B\cos\alpha_{\mathrm{safe}}-\mathrm{den}_{\min}\right)\right)$$

This ensures

$$\mathrm{den}=B\cos\alpha_{\mathrm{safe}}-k_{\mathrm{eff}}\ge\mathrm{den}_{\min}$$

at all times, preventing division by small numbers.

**6. Theta-term clamps**

Clamp the base tracking error signals before they enter the sliding surface to prevent large outliers (from sensor glitches or reference discontinuities) from dominating the control:

\begin{align}
\theta_{\mathrm{err,c}} &= \mathrm{constrain}(\theta_{\mathrm{err}},-\theta_{\max},+\theta_{\max})\\
\dot\theta_{\mathrm{err,c}} &= \mathrm{constrain}(\dot\theta_{\mathrm{err}},-\dot\theta_{\max},+\dot\theta_{\max})\\
\theta_{\mathrm{term}} &= \dot\theta_{\mathrm{err,c}}+\lambda_\theta\theta_{\mathrm{err,c}}\\
\theta_{\mathrm{term,c}} &= \mathrm{constrain}(\theta_{\mathrm{term}},-\theta_{\mathrm{term,max}},+\theta_{\mathrm{term,max}})
\end{align}

Typical limits (firmware defaults): $\theta_{\max}=78^\circ$, $\dot\theta_{\max}=200\ \mathrm{deg/s}$, $\theta_{\mathrm{term,max}}=300\ \mathrm{deg/s}$. These are large enough to be inactive during normal operation but protective against edge cases.

**7. SMC4 (full-surface) internal $\dot\alpha$ clamp (stepper-friendly)**

In `CTRL_SMC_FULL` (SMC4), the implementation clamps the measured $\dot\alpha$ used **inside the SMC4 math** to reduce spikes in the $\lambda_\alpha\dot\alpha$ term:

$$\dot\alpha_{\mathrm{ctrl}}=\mathrm{constrain}(\dot\alpha,\ -200,\ +200)\ \mathrm{deg/s}$$

This clamp does **not** change the safety abort logic: the separate upright-only abort on extreme $|\dot\alpha|$ still uses the filtered measured $\dot\alpha$.

**8. SMC4 actuator shaping (stepper-friendly)**

To reduce stall/missed-step events during disturbance recovery, the firmware applies tighter caps in SMC4:

- acceleration saturation: $|\ddot\theta_{\mathrm{steps}}|\le 12{,}000\ \mathrm{steps/s^2}$ (in addition to the global $20{,}000$ cap)
- speed saturation: $|\dot\theta_{\mathrm{steps}}|\le 2{,}000\ \mathrm{steps/s}$

### 13.8 Parameter selection guidelines

The full-state surface SMC has five tunable parameters. This section provides guidance for systematic tuning based on the identified plant constants (Sec. 8) and implementation constraints.

**Shared parameters (from Sec. 12):**

1. **$\lambda_\alpha$ (pendulum convergence rate):** Sets the time constant $\tau=1/\lambda_\alpha$ for pendulum decay on the sliding surface.
   - **Default:** $\lambda_\alpha=15.0\ \mathrm{s^{-1}}$ (matching the fast pole in Sec. 11.6, giving $\tau\approx 67\ \mathrm{ms}$).
   - **Effect:** Larger $\lambda_\alpha$ increases pendulum stiffness and bandwidth but amplifies noise sensitivity and increases required control authority.
   - **Range:** Typical range $10$–$20\ \mathrm{s^{-1}}$ for this hardware.

2. **$K$ (reaching gain):** Controls the rate of convergence to the sliding surface outside the boundary layer.
   - **Default:** $K=800\ \mathrm{deg/s^2}$ (equivalent to $\approx 3555\ \mathrm{steps/s^2}$, well below the saturation limit of $20{,}000\ \mathrm{steps/s^2}$).
   - **Effect:** Larger $K$ reduces reaching time but increases control effort and can cause saturation during large disturbances; smaller $K$ reduces aggressiveness at the cost of slower transient response.
   - **Range:** $500$–$1500\ \mathrm{deg/s^2}$.

3. **$\phi$ (boundary layer thickness):** Trades off chattering suppression against steady-state tracking accuracy.
   - **Default:** $\phi=50.0\ \mathrm{deg/s}$.
   - **Effect:** Larger $\phi$ produces smoother control (less chattering) but increases steady-state error; smaller $\phi$ improves tracking but can cause residual switching noise.
   - **Guideline:** Choose $\phi$ larger than the observed noise magnitude in $\dot\alpha$ (typical sensor noise after filtering: $10$–$30\ \mathrm{deg/s}$).
   - **Range:** $30$–$80\ \mathrm{deg/s}$.

**New parameters (full-state coupling):**

4. **$k$ (base-pendulum coupling weight):** Determines how strongly base tracking errors influence the sliding surface and modifies the control effectiveness denominator $(B\cos\alpha-k)$.
   - **Default:** $k=0.50$ (conservative; provides base tracking while maintaining robust denominator margin).
   - **Effect:** Increasing $k$ strengthens base tracking authority but reduces the effective control gain (smaller denominator) and can cause twitchy behavior if $k\to B\cos\alpha$; decreasing $k$ toward zero decouples the base dynamics and approaches the hybrid SMC behavior (Sec. 12).
   - **Range:** $0.2$–$1.0$ (hard limit $k\le 1.2$, but typical operation uses $k\le 0.8$ for safety margin).
   - **Tuning strategy:** Start with $k=0.3$; if base drift is excessive, increase gradually in steps of $0.1$ while monitoring control effort and checking for saturation or oscillations.

5. **$\lambda_\theta$ (base error shaping):** Shapes how base position error $\theta_{\mathrm{err}}$ and velocity error $\dot\theta_{\mathrm{err}}$ combine in the sliding surface term $\dot\theta_{\mathrm{err}}+\lambda_\theta\theta_{\mathrm{err}}$.
   - **Default (firmware):** $\lambda_\theta=2.0\ \mathrm{s^{-1}}$ (`smcFullLambdaTheta` in `src/main.cpp`).
   - **Effect:** Larger $\lambda_\theta$ increases the weight of position error relative to velocity error, providing stronger centering stiffness; smaller $\lambda_\theta$ makes the surface more sensitive to base velocity error.
   - **Range:** $0.5$–$2.0\ \mathrm{s^{-1}}$.

**Practical note (important for this rig):** Although the SMC4 surface couples base errors via $k$, the firmware also retains a **gated base-centering assist term** (PD+FF on $\theta_{\mathrm{err}},\dot\theta_{\mathrm{err}}$, scaled by `O` / `smcBaseScale`). Empirically, this assist prevents slow base “random-walk” drift that can occur from modeling mismatch / unmodeled friction when relying purely on the surface coupling. For the pinned dataset runs, treat `O` as a first-line knob for “keep arm near center”, and keep $k$ moderate to preserve denominator margin.

**Interaction effects:**

- **$k$ and $K$:** Increasing $k$ reduces the effective gain (larger numerator coefficient, smaller denominator), which may necessitate reducing $K$ to avoid saturation or increasing $\phi$ to suppress resulting chatter.
- **$\lambda_\alpha$ and $\phi$:** Increasing $\lambda_\alpha$ increases the magnitude of the surface velocity $\dot s$, which may require increasing $\phi$ proportionally to maintain smooth control.

**Systematic tuning procedure:**

1. Start with the default values.
2. If pendulum stabilization is inadequate (large $|\alpha|$ oscillations), increase $\lambda_\alpha$ or $K$ (check for saturation).
3. If control is chattery or noisy, increase $\phi$.
4. If base drift is significant, increase $k$ gradually (monitor denominator margin and control smoothness).
5. If base position error overshoots or oscillates, adjust $\lambda_\theta$ (decrease for more damping, increase for stiffer centering).

**Firmware mapping (SMC4 controls):**

- Select mode: `C 2` → `CTRL_SMC_FULL` (`mode=SMC4` in `[STATUS]`)
- Tune full-surface parameters:
  - `D <k>` sets $k$ (firmware enforces $k\ge 0$ and $k\le 1.2$)
  - `L <lambda_theta>` sets $\lambda_\theta$ (firmware enforces $\lambda_\theta>0$)
- Shared SMC parameters (also affect SMC4): `J` ($\lambda_\alpha$), `K` (reaching gain), `P` ($\phi$), `Q` (`smcSign`), `O` (base-centering assist scale).
- Use `G` once per run to log the current parameter values into `events.txt`.

For safety and timing integrity, these controller/tuning commands are allowed only when **disarmed** (not in `STATE_ACTIVE`); use `!` to disarm before changing them.

### 13.9 Conversion to stepper units and implementable formula

The control law derivation (Sec. 13.5) uses radian-based quantities. The firmware implementation uses degree variables for states and stepper units ($\mathrm{steps/s^2}$) for acceleration commands. This section provides the unit conversions and implementable formula.

**Unit conversion constants:**

$$\deg2\mathrm{rad}=\frac{\pi}{180}=0.017453,\qquad \mathrm{rad}2\deg=\frac{180}{\pi}=57.2958$$

$$\mathrm{steps/deg}=\frac{N}{360}=\frac{1600}{360}=4.444$$

where $N=1600$ is the microsteps per motor revolution (Sec. 9).

**Implementable computation sequence (degree-based):**

Given sensor readings in degrees: $\alpha_{\deg}$, $\dot\alpha_{\deg}$, $\theta_{\deg}$, $\dot\theta_{\deg}$, and reference signals $\theta_{\mathrm{ref},\deg}$, $\dot\theta_{\mathrm{ref},\deg}$, $\ddot\theta_{\mathrm{ref},\deg}$:

1. **Convert angles for trigonometric functions:**
   $$\alpha_{\mathrm{rad}}=\alpha_{\deg}\cdot\deg2\mathrm{rad}$$

2. **Compute trig terms with cosine protection:**
   $$\sin\alpha=\sin(\alpha_{\mathrm{rad}}),\qquad \cos\alpha=\cos(\alpha_{\mathrm{rad}})$$
   $$\cos\alpha_{\mathrm{safe}}=\max(|\cos\alpha|,0.2)\cdot\mathrm{sign}(\cos\alpha)$$

3. **Compute tracking errors with clamps (Sec. 13.7):**
   $$\theta_{\mathrm{err}}=\theta_{\deg}-\theta_{\mathrm{ref},\deg}$$
   $$\dot\theta_{\mathrm{err}}=\dot\theta_{\deg}-\dot\theta_{\mathrm{ref},\deg}$$
   (In firmware, $\theta_{\mathrm{err}}$ uses a wrap-safe shortest-path difference; apply clamps: $|\theta_{\mathrm{err}}|\le 78^\circ$, $|\dot\theta_{\mathrm{err}}|\le 200\ \mathrm{deg/s}$.)

4. **Compute effective coupling with ramp-in and denominator safety:**
   $$t_{\mathrm{active}}=\text{time since engagement (ms)}$$
   $$k_{\mathrm{ramp}}=\min(t_{\mathrm{active}}/T_{\mathrm{ramp}},1.0)\quad(T_{\mathrm{ramp}}=200\ \mathrm{ms})$$
   $$k_{\mathrm{req}}=k_{\mathrm{user}}\cdot k_{\mathrm{ramp}}$$
   $$k_{\mathrm{max}}=B\cos\alpha_{\mathrm{safe}}-0.60$$
   $$k_{\mathrm{eff}}=\max(0,\min(k_{\mathrm{req}},k_{\mathrm{max}}))$$

5. **Theta-term with clamp:**
   $$\theta_{\mathrm{term}}=\dot\theta_{\mathrm{err}}+\lambda_\theta\theta_{\mathrm{err}}$$
   (Apply clamp: $|\theta_{\mathrm{term}}|\le 300\ \mathrm{deg/s}$)

6. **SMC4 internal $\dot\alpha$ control value (firmware):**
   In `CTRL_SMC_FULL` (SMC4), the firmware uses a clamped pendulum rate inside the SMC4 computation:
   $$\dot\alpha_{\mathrm{ctrl}}=\mathrm{constrain}(\dot\alpha_{\deg},\ -200,\ +200)\ \mathrm{deg/s}$$

7. **Sliding surface:**
   $$s=\dot\alpha_{\mathrm{ctrl}}+\lambda_\alpha\alpha_{\deg}+k_{\mathrm{eff}}\theta_{\mathrm{term}}\quad(\mathrm{deg/s})$$

8. **Saturation function:**
   $$\mathrm{sat}(s/\phi)=\min(1,\max(-1,s/\phi))$$

9. **Drift function in deg/s² (mixed units):** The term $f_0$ involves $\dot\theta^2$ which must be converted. Internal computation in radians is cleaner:
   $$\dot\theta_{\mathrm{rad}}=\dot\theta_{\deg}\cdot\deg2\mathrm{rad}$$
   $$f_{0,\mathrm{rad}}=A\sin\alpha+\sin\alpha\cos\alpha\,\dot\theta_{\mathrm{rad}}^2\quad(A=100.8\ \mathrm{rad/s^2})$$
   $$f_{0,\deg}=f_{0,\mathrm{rad}}\cdot\mathrm{rad}2\deg$$

10. **Full drift term:**
   $$f_{\mathrm{full},\deg}=f_{0,\deg}+\lambda_\alpha\dot\alpha_{\mathrm{ctrl}}+k_{\mathrm{eff}}\lambda_\theta\dot\theta_{\mathrm{err}}-k_{\mathrm{eff}}\ddot\theta_{\mathrm{ref},\deg}$$

11. **Control law in deg/s²:**
    $$\ddot\theta_{\deg}=\frac{f_{\mathrm{full},\deg}+K\cdot\mathrm{sat}(s/\phi)}{B\cos\alpha_{\mathrm{safe}}-k_{\mathrm{eff}}}$$

12. **Clamp to maximum acceleration (numeric safety):**
    $$\ddot\theta_{\deg}=\mathrm{constrain}(\ddot\theta_{\deg},-\ddot\theta_{\max},+\ddot\theta_{\max})\quad(\ddot\theta_{\max}=4500\ \mathrm{deg/s^2})$$
    This corresponds to the global firmware limit $20{,}000\ \mathrm{steps/s^2}$.

13. **Convert to stepper acceleration:**
    $$\ddot\theta_{\mathrm{steps}}=4.444\cdot\ddot\theta_{\deg}\quad(\mathrm{steps/s^2})$$

14. **Base-centering assist term (firmware addition, stepper-friendly):**

    In addition to the SMC4 acceleration computed from the surface, the firmware retains a small, gated base-centering assist:

    $$\ddot\theta_{\mathrm{base,steps}} = O\cdot\left(\ddot\theta_{\mathrm{ref,steps}} + K_{\Theta}\,\theta_{\mathrm{err}} + K_{\dot\Theta}\,\dot\theta_{\mathrm{err}}\right)$$

    where $\ddot\theta_{\mathrm{ref,steps}} = 4.444\cdot \ddot\theta_{\mathrm{ref},\deg}$ and $O\in[0,2]$ is the runtime-tunable scale (command `O <val>`, firmware variable `smcBaseScale`). In firmware this term uses the same gains as the linear mode (`K_THETA`, `K_THETADOT`, units: steps/s² per deg and steps/s² per deg/s).

    **Gate (SMC4):** enabled only when engaged and near upright, approximately
    $$|\alpha|<5^\circ,\quad |\dot\alpha|<250\ \mathrm{deg/s}$$
    so it does not fight recovery when far from upright. The final physical acceleration is the sum:

    $$\ddot\theta_{\mathrm{cmd,steps}} = \ddot\theta_{\mathrm{SMC4,steps}} + \ddot\theta_{\mathrm{base,steps}}$$

15. **Apply overall sign and actuator limits** (firmware pipeline):

    - Apply `smcSign` to $\ddot\theta$ (SMC-only sign knob), then apply `CTRL_SIGN` after assembling the mode’s full `accCmdPhysical`.
    - Apply the engage ramp (100 ms) and saturation. In SMC4, the firmware uses a tighter acceleration limit:
      $$\ddot\theta_{\mathrm{steps}}=\mathrm{constrain}(\ddot\theta_{\mathrm{steps}},-12{,}000,+12{,}000)$$
    - Integrate to a speed command and apply a tighter SMC4 speed limit:
      $$\dot\theta_{\mathrm{steps}}=\mathrm{constrain}(\dot\theta_{\mathrm{steps}},-2{,}000,+2{,}000)$$

    These SMC4-specific caps are a stepper-friendly robustness choice and are not part of the ideal continuous-time derivation.

This sequence is implemented in the firmware `CTRL_SMC_FULL` branch, which outputs the final stepper acceleration command $\ddot\theta_{\mathrm{steps}}$ to the existing velocity-integration and speed-command pipeline (unchanged from Sec. 12).

### 13.10 Comparison with hybrid SMC and linear controllers

The full-state surface SMC (Sec. 13) represents the third nonlinear control mode implemented in the system, alongside the hybrid SMC (Sec. 12) and linear full-state feedback (Sec. 11). This section compares the three approaches to clarify design tradeoffs and guide mode selection.

**Linear full-state feedback (`CTRL_LINEAR`, command `C 0`):**

- **Surface/law:** Uses the linearized plant model $\ddot\alpha=A\alpha-B\ddot\theta$ (small-angle approximation). Control law is a linear combination of all four states: $\ddot\theta=-k_\theta\theta-k_\alpha\alpha-k_{\dot\theta}\dot\theta-k_{\dot\alpha}\dot\alpha$.
- **Tuning:** Gains are computed analytically via pole placement (Sec. 11.5) using two second-order characteristic polynomial factors. Straightforward to tune by specifying desired natural frequencies $\omega_1,\omega_2$ and damping ratios $\zeta_1,\zeta_2$.
- **Advantages:** Simple (no trig functions in real-time loop); smooth control (no switching behavior); globally valid if linearization errors are small.
- **Limitations:** Does not explicitly compensate for centrifugal coupling $\sin\alpha\cos\alpha\,\dot\theta^2$ or nonlinear gravity term $A\sin\alpha$; performance degrades for large angles ($|\alpha|>5^\circ$) or fast base motion.

**Hybrid SMC (`CTRL_SMC`, command `C 1`):**

- **Surface/law:** Uses a pendulum-only sliding surface $s=\dot\alpha+\lambda\alpha$ with nonlinear plant compensation. Outputs SMC acceleration for upright stabilization; adds a separate PD-based base-centering term (gated by $|\alpha|<5^\circ$, $|\dot\alpha|<150\ \mathrm{deg/s}$). Two control objectives are separated and blended.
- **Tuning:** Pendulum SMC parameters ($\lambda,K,\phi$) are tuned independently; base centering is controlled by a scale factor $s_{\mathrm{scale}}\in[0,2]$ (command `O`).
- **Advantages:** Easier to tune than full-state surface SMC (decoupled objectives); robust pendulum stabilization using exact $\sin\alpha$, $\cos\alpha$ terms; base centering is optional and can be disabled without affecting upright control.
- **Limitations:** Base tracking is secondary and only active when pendulum is nearly upright and still; gate logic introduces switching behavior that can be conservative.

**Full-state surface SMC (`CTRL_SMC_FULL`, command `C 2`):**

- **Surface/law:** Uses a unified sliding surface $s=\dot\alpha+\lambda_\alpha\alpha+k(\dot\theta_{\mathrm{err}}+\lambda_\theta\theta_{\mathrm{err}})$ encoding both pendulum and base dynamics. The firmware computes an SMC4 acceleration from this surface **and** (for this stepper rig) retains a small gated base-centering assist term (PD+FF on $\theta_{\mathrm{err}},\dot\theta_{\mathrm{err}}$, scaled by `O`), plus SMC4-only actuator shaping (Sec. 13.7).
- **Tuning:** Five parameters ($\lambda_\alpha,K,\phi,k,\lambda_\theta$) with interactions: $k$ directly affects control effectiveness denominator $(B\cos\alpha-k)$.
- **Advantages:** Both objectives emerge primarily from a single surface; theoretically elegant and suitable for formal analysis (denominator conditioning and surface coupling are explicit).
- **Limitations:** More sensitive to parameter choice (coupling $k$ affects both surface dynamics and denominator conditioning); requires careful safety margins (Sec. 13.7); on a stepper actuator it can be more stall-sensitive under aggressive disturbances unless $\phi$ is large and actuator shaping is enabled.

**Performance comparison (empirically, near-upright operation $|\alpha|<5^\circ$):**

- **Pendulum stabilization:** All three modes achieve similar RMS pendulum angle ($<1^\circ$) for typical disturbances. SMC modes slightly outperform linear for larger angles or fast base motion due to explicit nonlinear compensation.
- **Base tracking:** Linear and full-state surface SMC provide continuous base actuation; hybrid SMC base response depends on gate state. For reference tracking tasks (trapezoidal profiles), linear and SMC_FULL are preferred.
- **Control smoothness:** Linear mode produces the smoothest acceleration commands; hybrid SMC has minor discontinuities at gate transitions; SMC_FULL smoothness depends on boundary layer thickness $\phi$.
- **Computational cost:** Linear mode is cheapest (no trig); both SMC modes compute $\sin\alpha$, $\cos\alpha$, and $f_0$ (similar cost).

**Mode selection guidelines:**

- Use **`CTRL_LINEAR`** for typical upright balancing with gentle base centering; easiest to tune and adequate for $|\alpha|<5^\circ$.
- Use **`CTRL_SMC`** (hybrid) for robust pendulum stabilization with optional aggressive base tracking (tune $s_{\mathrm{scale}}$); best for disturbance rejection experiments where base drift is secondary.
- Use **`CTRL_SMC_FULL`** (SMC4) for demonstration of a full-state surface SMC with $\theta$ terms. In this implementation it is “full-surface” in the SMC law, but still uses stepper-friendly additions (base-centering assist `O`, actuator shaping); treat it as a proof-of-concept mode and expect more sensitivity to actuator limits under aggressive tap/nudge tests.

The command `C <0|1|2>` selects the controller mode (only when disarmed) for direct experimental comparison under identical conditions.

---
