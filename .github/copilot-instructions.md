# Rotary Inverted Pendulum (Furuta) — Repo Context for Coding Agents

This repo contains the full end-to-end stack for a **rotary (Furuta) inverted pendulum** driven by a **stepper motor** and measured by **two AS5600 encoders** (via an I²C mux). The system supports:
- Upright balancing with **three controller modes** (linear + two sliding-mode variants).
- Optional **reference tracking / “nudge mode”**: commanded base/arm angle targets while staying balanced.
- Host logging to timestamped CSV + event logs (`tools/run_balance.py`).
- Batch analysis → thesis-ready report folder (`tools/analyze_experiments.py` + `tools/balance_analysis/`).
- A standalone LaTeX thesis project in `thesis/` fed by an export step (`tools/export_thesis_assets.py`).

If you are changing anything control-related: **read `src/main.cpp` first**.

## Canonical workflows

- Experiments: `tools/experiment_quickstart.md`
- Analysis (batch): `tools/analyze_experiments.py`
- Thesis export + build: `tools/export_thesis_assets.py` → `thesis/` → `latexmk`

## Key files (source of truth)

- Firmware: `src/main.cpp` (state machine, controllers, serial commands, safety)
- Modelling + derivations: `tools/modelling_complete.md` (first principles → linear + SMC)
- Nudge/reference tracking details: `tools/base_tracking.md`
- Host logger: `tools/run_balance.py` (CSV contract + event capture)
- Analysis library: `tools/balance_analysis/` (IO/events/alignment/trials/metrics/plots)
- Batch report generator: `tools/analyze_experiments.py`
- Polished analysis notebook: `tools/thesis_results.ipynb` (loads a report folder)
- Thesis LaTeX project: `thesis/main.tex` (builds to `thesis/build/main.pdf`)
- Thesis export pipeline: `tools/export_thesis_assets.py` + `tools/build_modelling_chapter.py`

## Hardware + IO (current firmware)

- MCU: Arduino Mega 2560
- Stepper driver: STEP/DIR/EN interface (FastAccelStepper)
- Encoders: 2× AS5600 via I²C mux at `0x70`
  - mux ch0: pendulum encoder
  - mux ch1: base encoder
- Pins (see `src/main.cpp`):
  - `STEP_PIN=11`, `DIR_PIN=6`, `EN_PIN=7`

## Firmware architecture (important invariants)

### State machine

- `STATE_CALIBRATE`: waiting for `Z`
- `STATE_IDLE`: motor outputs disabled; monitors “upright and still” to auto-engage if armed
- `STATE_ACTIVE`: closed-loop control + logging

### Loop timing / logs

- Control tick: `LOOP_US=5000` → **200 Hz**
- CSV telemetry: **50 Hz** (decimated)
- Status print: **1 Hz** (`[STATUS] ...`) + diagnostics (`[DBG] ...`, `[WARN] ...`)
- Jitter counters: `dbgDtUsMin/dbgDtUsMax/dbgDtOverrun` (watch for regressions)

### Sensor handling

- Always call `selectMux(ch)` before reading the AS5600.
- All **angle differences** must use `getAngleDiffDeg()` (wrap-safe in ±180°). Do not raw-subtract periodic angles.

### Stepper actuation (velocity mode; acceleration-input control)

This project drives the stepper in **continuous speed mode**:
- Controller computes an acceleration-like command `u` in **steps/s²**
- Firmware integrates to a speed command in **steps/s**
- `commandMotorSpeed()` applies direction/speed with `runForward()/runBackward()`

Do not stop by calling `setSpeedInHz(0)`; use `stopMove()` (already implemented).

## Controllers (upright-only; 3 modes)

Serial command `C` selects `ctrlMode`:
- `C 0` → `CTRL_LINEAR`: linear full-state feedback (reference-tracking form)
- `C 1` → `CTRL_SMC`: hybrid nonlinear sliding mode controller (pendulum surface + base assist)
- `C 2` → `CTRL_SMC_FULL`: full-surface SMC (“SMC4”: four-state surface including base errors)

### Linear full-state (CTRL_LINEAR)

- Reference-tracking errors:
  - `thetaErr = getAngleDiffDeg(thetaDeg, thetaRefDeg)`
  - `thetaDotErr = thetaDotFilt - thetaRefVelDegS`
- Includes feedforward `thetaRefAcc` (deg/s² → steps/s²)
- Drift protections:
  - velocity leak enabled **only** in linear mode and only near upright/center
  - slow auto-trim of the upright pendulum reference when stable

### Hybrid SMC (CTRL_SMC)

- Uses the nonlinear relation (see `tools/modelling_complete.md`):
  `alphaDDot = A*sin(alpha) + sin(alpha)*cos(alpha)*thetaDot^2 - B*cos(alpha)*thetaDDot`
- Boundary layer (`phi`) reduces chatter.
- Clamp `thetaDDot` in deg/s² before converting to steps/s².
- Upright-only abort windows: abort on large `|alpha|` and extreme `|alphaDot|`.
- Velocity leak is disabled by design in nonlinear modes.
- Base centering is a **secondary blended term** (`O`/`smcBaseScale`), gated by an upright/still window; printed in `[STATUS]` as `accBase` and `gate`.

### Full-surface SMC (CTRL_SMC_FULL / SMC4)

- Single surface including base errors (wrap-safe):
  - `s = alphaDot + lambda_alpha*alpha + kEff*(thetaDotErr + lambda_theta*thetaErr)`
- Denominator safety: enforces a minimum margin on `den = B*cosSafe - kEff`.
- Mode-local shaping:
  - `smcFullK` is constrained to be **>= 0** (negative values cause base anti-damping; firmware rejects them).
  - `kEff` ramps in after engage (`SMC_FULL_K_RAMP_MS`).
- Still uses the same base reference signals as nudge mode (`thetaRef*`), but the full-surface controller can demand higher effort under disturbance recovery (watch for `[WARN] BASE_STALL? ...` then `FALLEN ...`).

## Reference tracking (“nudge mode”)

- `T <deg>` set base target angle (deg)
- `V <deg/s>` set trapezoid max reference velocity
- `X <deg/s^2>` set trapezoid max reference acceleration

Implementation:
- trapezoidal profile generator for `(thetaRefDeg, thetaRefVelDegS, thetaRefAccDegS2)`
- pause/resume logic on instability using **one-shot handoff on pause entry** (`thetaRefDeg = thetaMeas` once)
- wrap-safe diffs everywhere for theta angles

## Serial interface (safe subset)

Minimal workflow:
- `!` emergency disarm (safe anytime)
- `Z` calibrate (arm centered; pendulum upright)
- `E` arm/disarm (auto-engages when upright & still)
- `G` print current config (including controller + SMC parameters)

Signs + persistence:
- `S` sign wizard (guided hardware test)
- `Y` save persisted settings to EEPROM
- `R` clear EEPROM settings
- EEPROM stores: `motorSign`, `ALPHA_SIGN`, `THETA_SIGN`, `CTRL_SIGN`, `omega_c`, `speedStopHz` (not SMC params)

SMC/SMC4 tuning (guarded):
- `C 0|1|2` controller select
- `J/K/P` SMC tuning, `Q` sign flip, `O` base-assist scale (`0..2`)
- `D`/`L` full-surface SMC tuning (`smcFullK`, `smcFullLambdaTheta`)

Safety rule:
- These commands are rejected when `STATE_ACTIVE` (balancing) to avoid “mode switch kicks” and `Serial.parseFloat()/parseInt()` stalls; firmware sets `dropUntilEol` to flush trailing characters.

Default nonlinear params (verify with `G`):
- Shared: `J=15`, `K=800`, `P=50`, `Q=+1`, `O=1.0`
- Full-surface extras: `D=0.50`, `L=2.00`

## Pinned thesis dataset (Feb 2026)

Pinned comparison set:
- Manifest: `experiments/manifest_thesis_20260209.json`
- Analysis report output: `reports/thesis/report_thesis_20260209/`

Sessions (7 total):
- Linear hold: `logs/balance/session_20260209_095631`
- Hybrid SMC hold: `logs/balance/session_20260209_095908`
- Full-surface SMC hold: `logs/balance/session_20260209_121635`
- Linear commanded nudge: `logs/balance/session_20260209_114605`
- Linear tap: `logs/balance/session_20260209_114857`
- Hybrid SMC tap: `logs/balance/session_20260209_114953`
- Full-surface SMC tap: `logs/balance/session_20260209_121803`

## Analysis workflow (do not break)

All Python commands must run inside the repo venv:
- `./.venv/bin/python ...`

Generate the pinned report:
- `./.venv/bin/python tools/analyze_experiments.py --manifest experiments/manifest_thesis_20260209.json --out reports/thesis/report_thesis_20260209`

Report folder contract (consumed by thesis export):
- `metrics_trials.csv`
- `metrics_summary_by_experiment_mode.csv`
- `metrics_nudge_steps.csv`
- `alignment_quality.csv`
- `figures/` (PDF preferred; PNG fallback)

Important analysis constraints:
- `events.txt` is host wall-clock; CSV is device `millis()` → never assume timestamps align.
- FFT/spectrum plots are **low-frequency only** (≤25 Hz) due to ~50 Hz logging (Nyquist ~25 Hz).

## Thesis (LaTeX) workflow

1) Export assets + generate tables + write metadata + generate modelling chapter:
- `./.venv/bin/python tools/export_thesis_assets.py --report reports/thesis/report_thesis_20260209 --manifest experiments/manifest_thesis_20260209.json --out thesis`

2) Build PDF:
- `cd thesis && latexmk` (outputs to `thesis/build/main.pdf`)

Notes:
- `tools/export_thesis_assets.py` copies curated figures into `thesis/figures/` and generates LaTeX tables in `thesis/tables/`.
- `thesis/chapters/03_modelling_derivation_generated.tex` is auto-generated from `tools/modelling_complete.md`. Do not hand-edit it.

## Logger contract (do not break casually)

`tools/run_balance.py` expects exactly **9 CSV columns**:
`t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped`

If you must change telemetry: update the host parser, analysis scripts, and docs together.
