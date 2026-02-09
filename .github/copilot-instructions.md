# Rotary Inverted Pendulum — AI Agent Instructions

This repo contains firmware + tooling for a **rotary inverted pendulum (Furuta)** with a **stepper motor** and **two AS5600 encoders** (via an I²C mux). The system is now stable and supports:
- Upright balancing with **three controller modes**:
  - Linear full-state feedback (`C 0`)
  - Hybrid nonlinear SMC (`C 1`)
  - Full-state surface SMC (“SMC4”, `C 2`)
- Optional base/arm position commands (“nudge mode”) while balancing.
- Logging to timestamped CSV + events via `tools/run_balance.py`.
- Thesis-grade batch analysis + report generation (`tools/analyze_experiments.py`, `tools/thesis_results.ipynb`).

If you are changing anything control-related: **read `src/main.cpp` first**.

## Canonical “how to run”

Use `tools/experiment_quickstart.md` as the standard experiment workflow.

## Thesis dataset (current, Feb 2026)

The current thesis comparison set is captured in:
- Manifest: `experiments/manifest_thesis_20260209.json`
- Report output (generated from the manifest): `reports/thesis/report_thesis_20260209/`

Sessions included (7 total):
- Exp1 (Linear hold): `logs/balance/session_20260209_095631`
- Exp2 (Hybrid SMC hold): `logs/balance/session_20260209_095908`
- Exp3 (SMC4 hold): `logs/balance/session_20260209_121635`
- Exp4 (Linear commanded nudge): `logs/balance/session_20260209_114605`
- Exp5 (Linear tap): `logs/balance/session_20260209_114857`
- Exp6 (Hybrid SMC tap): `logs/balance/session_20260209_114953`
- Exp7 (SMC4 tap): `logs/balance/session_20260209_121803`

Re-generate the exact thesis report with:
- `./.venv/bin/python tools/analyze_experiments.py --manifest experiments/manifest_thesis_20260209.json --out reports/thesis/report_thesis_20260209`

## Key Files (source of truth)

- `src/main.cpp` — active firmware (state machine, controllers, serial commands)
- `tools/run_balance.py` — host logger (expects a fixed 9-column CSV format)
- `tools/experiment_quickstart.md` — minimal command workflow for experiments
- `tools/base_tracking.md` — “nudge mode” reference tracking + trapezoid + pause/resume
- `modelling_notes.md` — SMC design notes + unit conversions (matches firmware)
- `tools/modelling_complete.md` — full modelling + derivations (includes SMC)
- `tools/analyze_experiments.py` — batch analyzer → thesis report folder
- `tools/balance_analysis/` — reusable analysis library used by the analyzer/notebook
- `tools/thesis_results.ipynb` — polished notebook that loads a report folder
- `platformio.ini` — build flags (FastAccelStepper timer module), libs, serial monitor settings
- `docs/PinConfiguration.md` — wiring/pinout notes

## Hardware + IO (current firmware)

- MCU: Arduino Mega 2560
- Stepper driver: Step/Dir interface (FastAccelStepper)
- Encoders: 2× AS5600 via I²C mux at `0x70`
  - mux ch0 = pendulum encoder
  - mux ch1 = base encoder
- Pins (see `src/main.cpp`):
  - `STEP_PIN=11`, `DIR_PIN=6`, `EN_PIN=7`

## Firmware architecture (important invariants)

### State machine

- `STATE_CALIBRATE`: waiting for `Z`
- `STATE_IDLE`: motor outputs disabled; monitors “upright and still” to auto-engage if armed
- `STATE_ACTIVE`: closed-loop control + logging

### Loop timing / logs

- Control tick: `LOOP_US=5000` → 200 Hz
- CSV telemetry: 50 Hz (decimated)
- Human status: 1 Hz (`[STATUS] ...`) plus diagnostics (`[DBG] ...`)
- Jitter counters: `dbgDtUsMin/dbgDtUsMax/dbgDtOverrun` (watch for regressions)

### Sensors

- Always call `selectMux(ch)` before reading the AS5600.
- Angle errors use `getAngleDiffDeg()` (wrap-safe in ±180°) — do not raw-subtract periodic angles.

### Stepper actuation (velocity mode)

This project drives the stepper in **continuous speed mode**:
- Controller computes an acceleration-like command (steps/s²)
- Firmware integrates to a speed command (steps/s)
- `commandMotorSpeed()` applies speed using `runForward()/runBackward()`

Do not stop by calling `setSpeedInHz(0)`; use `stopMove()` (already implemented).

## Controllers (upright-only)

Controller mode is selected by `ctrlMode`:
- `CTRL_LINEAR` (command `C 0`): linear full-state feedback (reference-tracking form)
- `CTRL_SMC` (command `C 1`): nonlinear sliding mode controller with boundary layer
- `CTRL_SMC_FULL` (command `C 2`): full-state surface SMC (“SMC4”) with theta terms + stepper-friendly shaping

### Linear full-state (CTRL_LINEAR)

- Uses reference-tracking errors:
  - `thetaErr = getAngleDiffDeg(thetaDeg, thetaRefDeg)`
  - `thetaDotErr = thetaDotFilt - thetaRefVelDegS`
- Includes feedforward `thetaRefAcc` (converted to steps/s²)
- Drift protections:
  - velocity leak (bias killer) enabled only in linear mode and only near upright/center
  - auto-trim of pendulum upright reference when stable

### SMC (CTRL_SMC)

- Uses the nonlinear relation (see `modelling_notes.md` / `tools/modelling_complete.md`):
  \(\ddot\alpha = A\sin\alpha + \sin\alpha\cos\alpha\,\dot\theta^2 - B\cos\alpha\,\ddot\theta\)
- Boundary layer (`phi`) avoids chatter; `thetaDDot` is clamped before converting to steps/s².
- Upright-only abort windows:
  - abort on large `|alpha|` and extreme `|alphaDot|`
- **Velocity leak is disabled in SMC mode** by design.
- Base centering is a **secondary blended term** (`O` / `smcBaseScale`), gated by an upright/still window and visible in `[STATUS]` via `accBase` and `gate`.

### Full-state surface SMC (CTRL_SMC_FULL / SMC4)

- Uses a single sliding surface including base errors:
  - `s = alphaDot + smcLambda*alpha + kEff*(thetaDotErr + smcFullLambdaTheta*thetaErr)` (SMC4 surface)
- Denominator safety:
  - clamps `kEff` and enforces a minimum `den = (B*cosSafe - kEff)` margin.
- Still uses the same base reference signals (`thetaRefDeg`, `thetaRefVelDegS`, `thetaRefAccDegS2`) as linear/nudge mode.
- Practical (firmware) additions for this stepper-based rig:
  - keeps a gated base-centering assist term (same PD+FF form as hybrid SMC), scaled by `O`/`smcBaseScale`
  - uses SMC4-only internal shaping: tighter accel/speed caps and an `alphaDot` clamp in the SMC4 math to reduce “tick”/stall events.
- Common failure mode when tapping: `[WARN] BASE_STALL? ...` then `FALLEN ...` (the controller commands motion but the stepper can’t produce it, so alpha runs away).

## Serial interface (minimal and safe)

Minimal workflow commands:
- `!` emergency disarm (safe anytime)
- `Z` calibrate (arm centered; pendulum upright)
- `E` arm/disarm (auto-engages when upright & still)
- `G` print current config (includes controller + SMC params)

Signs + persistence:
- `S` sign diagnostic wizard (guided hardware test; updates signs in RAM)
- `Y` save persisted settings to EEPROM
- `R` clear EEPROM settings (restores defaults in RAM)
- EEPROM stores: `motorSign`, `ALPHA_SIGN`, `THETA_SIGN`, `CTRL_SIGN`, `omega_c`, `speedStopHz` (not SMC params)

Controller select / SMC tuning:
- `C 0|1|2` controller mode (**DISARMED-only**; not `STATE_ACTIVE`)
- `Q 1|-1` SMC-only sign flip (**DISARMED-only**; not `STATE_ACTIVE`)
- `O <0..2>` SMC base-centering blend (**DISARMED-only**; not `STATE_ACTIVE`)
- `J/K/P` SMC tuning knobs (**DISARMED-only**; not `STATE_ACTIVE`)
- `D <k>` and `L <lambda_theta>` SMC4 tuning (**DISARMED-only**; not `STATE_ACTIVE`)

Default nonlinear params (as shipped in firmware; verify with `G`):
- Shared SMC: `J=15`, `K=800`, `P=50`, `Q=+1`, `O=1.0`
- SMC4 extras: `D=0.50`, `L=2.00`

Motion (nudge mode, optional):
- `T <deg>` base target angle
- `V <deg/s>` trapezoid max velocity
- `X <deg/s^2>` trapezoid max acceleration

Why some commands are DISARMED-only:
- Prevents “mode switch kicks” while balancing.
- Prevents `Serial.parseFloat()/parseInt()` stalls from causing control-loop overruns.

If you add/change commands: update the logger event keys in `tools/run_balance.py`.

## Thesis analysis workflow (do not break)

- Python must run inside the repo venv: `./.venv/bin/python`.
- Data collection produces `logs/balance/session_YYYYMMDD_HHMMSS/` folders.
- Batch analysis produces `reports/thesis/<timestamp>/` folders:
  - CLI: `./.venv/bin/python tools/analyze_experiments.py --sessions ...`
  - Notebook: `tools/thesis_results.ipynb` loads an existing report folder.

### Analysis invariants / limitations (thesis-safe claims)

- `events.txt` is host wall-clock time; CSV is device `millis()` time → **never** assume timestamps align.
  - The analyzer aligns host↔device time using robust matching of printed `[STATUS]` fields to the CSV stream.
- Spectrum/FFT plots are **low-frequency only** (≤25 Hz) because CSV logging is ~50 Hz (Nyquist ~25 Hz).
- For disturbance (“tap”) runs: unless you type `# <marker>` in the logger, disturbances are *not* explicitly labeled.

## Logger contract (do not break casually)

`tools/run_balance.py` expects exactly **9 CSV columns** at 50 Hz:
`t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped`

If you must change telemetry, update the host parser and bump the doc.
