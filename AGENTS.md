# Rotary Inverted Pendulum Agent Guide

This repo is the end-to-end stack for a rotary/Furuta inverted pendulum:
Arduino Mega firmware, host logging, Python analysis, thesis export, LaTeX
source, and thesis slide generation.

If the task is control-related, read `src/main.cpp` first. It is the live
runtime source of truth for pins, controller modes, telemetry, serial commands,
and safety behavior.

## Source Of Truth

- Firmware: `src/main.cpp`.
- PlatformIO target: `platformio.ini`, env `megaatmega2560`.
- Experiment workflow and serial cheat sheet: `tools/experiment_quickstart.md`.
- Mathematical model and constants: `tools/modelling_complete.md`.
- Nudge/reference-tracking notes: `tools/base_tracking.md`.
- Host logger and CSV contract: `tools/run_balance.py`.
- Analysis pipeline: `tools/analyze_experiments.py` and `tools/balance_analysis/`.
- Thesis export: `tools/export_thesis_assets.py` and
  `tools/build_modelling_chapter.py`.
- Thesis build instructions: `thesis/README.md`.
- Slide build instructions: `Slides/rotary_inverted_pendulum/README.md`.
- Existing IDE-agent context: `.github/copilot-instructions.md`.

`docs/PinConfiguration.md` is useful hardware background, but verify it against
`src/main.cpp` before relying on it. It still contains an older base-sensor PWM
section, while the current firmware reads both AS5600 sensors through the I2C
multiplexer.

## Firmware Facts

- Board/framework: Arduino Mega 2560, Arduino framework.
- Libraries: `FastAccelStepper` and `AS5600`.
- PlatformIO build flag: `-DFAS_TIMER_MODULE=1`, needed so FastAccelStepper can
  use pin 11 as STEP on the Mega.
- Active pins in firmware:
  - `STEP_PIN=11`
  - `DIR_PIN=6`
  - `EN_PIN=7`
  - I2C mux address `0x70`
  - mux channel 0: pendulum AS5600
  - mux channel 1: base AS5600
- Firmware serial baud is `500000`; `tools/run_balance.py` also uses `500000`.
  `platformio.ini` still has `monitor_speed = 115200`, so do not assume the
  PlatformIO monitor speed matches the active logger contract.

## Control Invariants

- State machine:
  - `STATE_CALIBRATE`: waits for `Z`.
  - `STATE_IDLE`: motor outputs disabled; can auto-engage when armed and the
    pendulum is upright/still.
  - `STATE_ACTIVE`: closed-loop control, telemetry, diagnostics, and safety
    checks.
- Main loop period: `LOOP_US=5000`, so control runs at about 200 Hz.
- Telemetry is decimated to 50 Hz and printed as exactly 9 comma-separated
  fields:
  `t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped`.
- `tools/run_balance.py` depends on that 9-column telemetry contract. If
  telemetry changes, update the logger, analysis code, quickstart, and this
  guide together.
- Always select the mux channel before reading an AS5600.
- Use `getAngleDiffDeg()` for angle differences. Do not raw-subtract wrapped
  angles.
- The stepper is driven in continuous velocity mode. Controllers compute an
  acceleration-like command in steps/s^2, firmware integrates it to steps/s,
  and `commandMotorSpeed()` applies direction/speed.
- For stopping, use `stopMove()` and disable outputs where appropriate. Do not
  introduce `setSpeedInHz(0)` as the stop path.

## Controller Modes

Serial command `C` selects the controller when disarmed:

- `C 0`: linear full-state feedback with base reference tracking.
- `C 1`: hybrid nonlinear sliding mode control with base-centering assist.
- `C 2`: full-surface SMC, also called SMC4.

Important controller constraints:

- SMC modes are upright-only; there is no swing-up controller.
- Firmware rejects SMC/SMC4 mode and tuning changes while `STATE_ACTIVE`.
- SMC4 requires `smcFullK >= 0` and `smcFullLambdaTheta > 0`; negative values
  have caused base anti-damping or unstable centering and should not be
  reintroduced.
- The linear controller has velocity leak and slow pendulum-reference trim near
  upright/center. Nonlinear modes intentionally do not use the linear velocity
  leak.
- Nudge mode uses `T <deg>`, `V <deg/s>`, and `X <deg/s^2>` to update the base
  target and trapezoidal reference profile.

## Serial Workflow

Minimal safe workflow:

1. `!` emergency disarm.
2. `G` print current config into `events.txt`.
3. `C 0`, `C 1`, or `C 2` select controller mode.
4. `Z` calibrate with arm centered and pendulum upright.
5. `E` arm; firmware auto-engages when the pendulum is upright and still.
6. `!` stop at the end of a trial.

Other common commands:

- `S`: sign diagnostic wizard.
- `A`, `H`, `B`, `M`: sign and motor-direction settings.
- `Y`: save persisted settings to EEPROM.
- `R`: clear EEPROM settings and restore defaults in RAM.
- `W`: derivative cutoff.
- `N`: motor velocity deadband.
- `U`: linear-mode velocity leak.
- `J`, `K`, `P`, `Q`, `O`: SMC tuning.
- `D`, `L`: SMC4 tuning.

EEPROM stores only `motorSign`, `ALPHA_SIGN`, `THETA_SIGN`, `CTRL_SIGN`,
`omega_c`, and `speedStopHz`. SMC parameters are runtime settings; verify them
with `G` during experiments.

## Experiments And Data

- Host logging command: `./.venv/bin/python tools/run_balance.py`.
- Each logger run creates `logs/balance/session_YYYYMMDD_HHMMSS/` with
  `balance_log.csv` and `events.txt`.
- Each firmware `ENGAGED!` to `DISARMED` or `FALLEN ...` block is one trial.
- Use logger-only markers by typing `# <note>` in `run_balance.py`.
- `events.txt` uses host wall-clock timestamps, while CSV telemetry uses device
  `millis()`. Analysis must align those timelines; never assume they share an
  origin.
- The pinned thesis manifest is
  `experiments/manifest_thesis_20260209.json`.
- Raw `logs/` and generated `reports/` are ignored by git. Curated exported
  thesis assets under `thesis/figures/` and `thesis/tables/` are tracked.

Before claiming a controller improvement, inspect the exact session logs and
report phase behavior by experiment type: hold, nudge, and tap. Do not tune from
whole-run averages alone when trial-level behavior matters.

## Analysis And Thesis Pipeline

Use the repo virtual environment for Python commands:

```bash
./.venv/bin/python tools/analyze_experiments.py \
  --manifest experiments/manifest_thesis_20260209.json \
  --out reports/thesis/report_thesis_20260209
```

The report folder consumed by thesis export must contain:

- `metrics_trials.csv`
- `metrics_summary_by_experiment_mode.csv`
- `metrics_nudge_steps.csv`
- `alignment_quality.csv`
- `figures/`

Export curated assets into `thesis/`:

```bash
./.venv/bin/python tools/export_thesis_assets.py \
  --report reports/thesis/report_thesis_20260209 \
  --manifest experiments/manifest_thesis_20260209.json \
  --out thesis
```

Build the thesis:

```bash
cd thesis
latexmk
```

`thesis/chapters/03_modelling_derivation_generated.tex` is generated from
`tools/modelling_complete.md`. Do not hand-edit the generated file; edit the
Markdown source and re-run the export.

## Slides

Slides live under `Slides/rotary_inverted_pendulum/`.

Build command:

```bash
cd Slides/rotary_inverted_pendulum
../../.venv/bin/python scripts/build.py
```

The slide builder uses Python and Node tooling, renders equations, copies figure
assets, creates a base PPTX, then post-processes the final deck into `dist/`.
Follow `Slides/thesis_presentation_design_system.md` for presentation styling.

## Validation Commands

- Firmware compile: `pio run`.
- Firmware upload: `platformio run -t upload` only when hardware upload is
  explicitly intended.
- Logger dependencies: `./.venv/bin/pip install -r tools/requirements.txt`.
- Thesis-analysis dependencies:
  `./.venv/bin/pip install -r tools/requirements_thesis.txt`.
- Analysis report: use the manifest command above.
- Thesis export: use `tools/export_thesis_assets.py`.
- Thesis PDF: `cd thesis && latexmk`.
- Slides: use the slide build command above.

Hardware-visible actions such as upload, serial monitor runs, motor motion, and
live experiments should not be started without explicit user intent.

## Git And File Hygiene

- Preserve user data and existing experiment outputs. Do not delete ignored
  logs or reports unless explicitly asked.
- The root `.gitignore` intentionally ignores local build products, virtual
  environments, `logs/`, and `reports/`, while keeping exported thesis figures
  and tables trackable.
- Avoid committing generated clutter such as `.DS_Store`, `.pio/`, `.venv/`,
  `__pycache__/`, LaTeX build files, or slide `node_modules/`.
- If firmware behavior, telemetry format, serial commands, hardware pins, or
  thesis asset contracts change, update adjacent docs in the same change:
  `tools/experiment_quickstart.md`, `.github/copilot-instructions.md`,
  `docs/PinConfiguration.md`, `thesis/README.md`, and this `AGENTS.md` as
  applicable.
