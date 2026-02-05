# Rotary Inverted Pendulum — Nonlinear Upright Control Notes (SMC)

This note documents the **nonlinear upright-only controller** implemented in firmware (`CTRL_SMC` in `src/main.cpp`) and the key unit conversions used in the real-time code.

Filename note: this file is named `modelling_notes.md` for consistency with `tools/modelling_complete.md`.

Scope:
- Upright balancing only (no swing-up).
- Stepper actuation is via **velocity mode**; the controller computes an acceleration-like command which is integrated to a speed command.

Related docs:
- Full derivations: `tools/modelling_complete.md` (Sec. 12)
- Base position “nudge mode”: `tools/base_tracking.md`
- Experiment workflow: `tools/experiment_quickstart.md`

---

## 1) Plant relation used for SMC

The SMC design uses the exact pendulum equation from the Furuta model (see `tools/modelling_complete.md`):

\[
\ddot\alpha = A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - B\cos\alpha\,\ddot\theta
\]

Where:
- \(\alpha\) = pendulum angle from upright
- \(\theta\) = base/arm angle
- \(u \equiv \ddot\theta\) is treated as the control input

Identified constants (used in firmware):
- \(A = 100.8\ \mathrm{rad/s^2}\)
- \(B = 1.952\) (dimensionless)

Important implementation note:
- Trig functions must use **radians**. Firmware measures angles in **degrees**, then converts to radians only for `sin()/cos()`.

---

## 2) Sliding surface and reaching law

Define the sliding surface (in the same angle units you implement in):

\[
s \equiv \dot\alpha + \lambda\alpha
\]

\(\lambda>0\) sets the exponential decay rate once on the surface: if \(s=0\), then \(\dot\alpha=-\lambda\alpha\).

To avoid chattering, use a boundary layer (saturation) reaching law:

\[
\dot s = -K\,\mathrm{sat}\!\left(\frac{s}{\phi}\right)
\]

Where:
- \(K > 0\) is the switching strength
- \(\phi > 0\) is the boundary layer thickness (bigger \(\phi\) → smoother control, slower/sloppier reaching)
- \(\mathrm{sat}(x)=\mathrm{clamp}(x,-1,+1)\)

---

## 3) Solve for the control law (\(\ddot\theta\))

Differentiate the sliding surface:

\[
\dot s = \ddot\alpha + \lambda\dot\alpha
\]

Substitute the plant model:

\[
\dot s =
\underbrace{A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2}_{f(\alpha,\dot\theta)}
+ \lambda\dot\alpha
+ \underbrace{(-B\cos\alpha\,\ddot\theta)}_{\text{control}}
\]

Impose the reaching law \(\dot s = -K\,\mathrm{sat}(s/\phi)\) and solve for \(\ddot\theta\):

\[
\ddot\theta =
\frac{f(\alpha,\dot\theta) + \lambda\dot\alpha + K\,\mathrm{sat}(s/\phi)}{B\cos\alpha}
\]

This is exactly what the firmware computes (with safety guards and degree-based scaling).

---

## 4) Degree-based implementation details (matches `src/main.cpp`)

Firmware signals (already signed by `ALPHA_SIGN` / `THETA_SIGN`):
- `alphaRawSigned` = \(\alpha_{\deg}\)
- `alphaDotFilt` = \(\dot\alpha_{\deg}\) (deg/s)
- `thetaDotFilt` = \(\dot\theta_{\deg}\) (deg/s, **measured**, not commanded)

### 4.1 Trig

Convert once:
- \(\alpha_{\mathrm{rad}} = \alpha_{\deg}\cdot(\pi/180)\)
- `sinA = sin(alphaRad)`, `cosA = cos(alphaRad)`

### 4.2 The drift term in deg/s²: why the centrifugal term uses one `deg2rad`

Start from the rad-based model:
\[
f_{\mathrm{rad}} = A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta_{\mathrm{rad}}^2
\]

Convert \(\dot\theta\) from degrees:
\[
\dot\theta_{\mathrm{rad}} = \dot\theta_{\deg}\cdot(\pi/180)
\]

Convert \(f\) to degrees:
\[
f_{\deg} = f_{\mathrm{rad}}\cdot(180/\pi)
= (A\cdot 180/\pi)\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta_{\deg}^2\cdot(\pi/180)
\]

So in code:
- `A_deg = MODEL_A * RAD2DEG`
- `fDeg = A_deg*sinA + (sinA*cosA)*(thetaDotDegS^2)*DEG2RAD`

### 4.3 Division safety: `cosSafe`

The ideal law divides by \(\cos\alpha\). Firmware uses:
- A hard upright-only validity window: `|alpha| > 25°` → abort to IDLE
- Plus a glitch guard: `cosSafe = copysign(max(|cosA|, SMC_COS_MIN), cosA)`

### 4.4 Final firmware law (degrees)

With:
- `s = alphaDotDegS + smcLambda*alphaDeg`
- `sat = clamp(s / smcPhiDegS, -1, +1)`

Firmware computes:

\[
\ddot\theta_{\deg} =
\mathrm{smcSign}\cdot
\frac{f_{\deg} + \lambda\dot\alpha_{\deg} + K\,\mathrm{sat}(s/\phi)}{B\cos_{\mathrm{safe}}(\alpha)}
\]

Then clamps:
- `thetaDDotSmcDegS2 = constrain(thetaDDotSmcDegS2, ±THETA_DDOT_MAX_DEG_S2)`

And converts to stepper units:
- `accSMC_steps_s2 = STEPS_PER_DEG * thetaDDotSmcDegS2`

### 4.5 Sign conventions in firmware

The “raw” SMC law above is then mapped through the same sign pipeline as the linear controller:
- Multiply by `CTRL_SIGN`
- Multiply by `motorSign`

If the SMC direction is wrong **without changing global signs**, flip the **SMC-only** knob:
- `Q -1` (sets `smcSign=-1`)

---

## 5) Keeping the arm near center (secondary objective)

Pure SMC stabilizes \(\alpha\), but does not guarantee \(\theta\) stays near 0 (it can drift due to bias, friction asymmetry, sensor offsets, etc).

Firmware adds a small base-centering acceleration term (in steps/s²):

```
accBase = (accFF + K_THETA*thetaErr + K_THETADOT*thetaDotErr) * smcBaseScale
```

Where:
- `thetaErr = wrapDiff(thetaDeg, thetaRefDeg)`
- `thetaRef*` comes from the existing reference generator / hold logic (see `tools/base_tracking.md`)
- `smcBaseScale` is `O <0..2>` (default **1.0**)

This base term is **gated** so it does not fight large disturbances:
- enabled only when `|alpha| < 5°` and `|alphaDot| < 150 deg/s` (and after engage grace)
- `gate=1` / `gate=0` is printed in `[STATUS]` to make this visible

---

## 6) Upright-only safety decisions

SMC is intentionally “local”:
- `|alpha| > 25°` → abort to IDLE (`FALLEN limit=smc_alpha`)
- `|alphaDot| > 250 deg/s` for a few consecutive ticks → abort to IDLE (`FALLEN limit=smc_alphaDot`)

This prevents numerical issues in the division and enforces “no swing-up” behavior.

---

## 7) Practical tuning order (don’t over-tune)

1. Get the direction right: `Q ±1`
2. Ensure base centering is active: watch `[STATUS] ... gate=1` frequently
3. If it is smooth but “weak” to disturbances: increase `K` and/or `lambda`
4. If it is too aggressive/noisy: increase `phi` and/or reduce `K`
5. If the base still drifts slowly: increase `O` (up to 2.0), but keep it gated and watch for destabilization
