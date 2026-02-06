# Experiment Quickstart (Upright Balancing)

This repo is driven by single-letter serial commands. This page lists the **minimum** commands + the **standard workflow** to run repeatable balance experiments with logging.

## 1) Start a session (PC)

1. Upload firmware (PlatformIO):
   - `platformio run -t upload`
2. Start the logger:
   - `./.venv/bin/python tools/run_balance.py`
3. Logs are created here each run:
   - `logs/balance/session_YYYYMMDD_HHMMSS/` (`balance_log.csv`, `events.txt`)

Notes:
- If the logger can’t connect, set `PORT = ...` at the top of `tools/run_balance.py`.
- If you haven't installed Python deps into the repo venv: `./.venv/bin/pip install -r tools/requirements.txt`

## 2) Minimal command cheat sheet

**Safety / required**
- `!`  Emergency disarm (stops motor + disables outputs). Safe anytime.
- `G`  Print config (captures controller parameters in `events.txt` for thesis analysis).
- `Z`  Calibrate (arm centered; pendulum upright). Do this every attempt.
- `E`  Arm/disarm toggle. When armed, it auto-engages when upright & still (`ENGAGED!`).

**Controller select**
- `C 0`  Linear controller (baseline)
- `C 1`  Nonlinear SMC controller
- `C 2`  Nonlinear SMC4 controller (full-state surface)

**SMC-only essentials (disarmed only)**
- `Q 1` / `Q -1`  Flip SMC direction if it reacts the wrong way (default: `Q 1`)
- `O <0..2>`      Base-centering strength (default: `O 1.0`)
- `D <k>`         SMC4 theta coupling (default: `D 0.5`)
- `L <val>`       SMC4 lambda_theta (default: `L 1.0`)

Important: firmware accepts `C/J/K/P/Q/O/D/L` **only when disarmed** (not balancing in `STATE_ACTIVE`). If you’re engaged, send `!` first.
After power-up you start in `STATE_CALIBRATE`, so do `Z` before `E`. (You may send `C/Q/O/...` either before or after `Z`.)

## 3) Workflow A — Linear baseline (fast sanity check)

In the logger terminal:

1. `!`
2. `G`
3. `C 0`
4. `Z`
5. `E` → wait for `ENGAGED!`
6. Let it run ~10–30s, then `!` to stop.

## 4) Workflow B — SMC upright balance (recommended)

In the logger terminal:

1. `!`
2. `G`
3. `C 1`
4. `Z`
5. `E` → wait for `ENGAGED!`

If something looks wrong:
- Wrong direction reaction: `!`, then `Q -1`, then re-run (`Z`, `E`)
- Arm slowly drifts: increase `O` (disarmed only), then re-run (`Z`, `E`)

What to watch (1 Hz lines):
- `[STATUS] ... mode=SMC gate=1` → base centering is active (arm should stay near center).
- If `gate=0` most of the time, SMC will still balance upright, but it won’t strongly “pull back” the arm (because α/α̇ aren’t inside the gate window).

Stop conditions (don’t fight it):
- If you see `FALLEN ...` or `[WARN] BASE_STALL? ...`, send `!`, re-center the arm, and restart from `Z`.

## 5) Workflow C — SMC4 (full-state surface) upright balance (thesis demo)

In the logger terminal:

1. `!`
2. `G`
3. `C 2`
4. (Optional) `D 0.5` and `L 1.0` (defaults)
5. `Z`
6. `E` → wait for `ENGAGED!`

Notes:
- `O` (base-centering blend) is a **hybrid SMC** feature; SMC4 does not use `O`.
- If SMC4 is too aggressive, reduce `D` first.

## 6) Optional — Base “nudge” while balancing

If you want to command base position while balancing:
- `T <deg>` sets a new base target angle (start small: `T 5`, `T -5`, `T 0`)
- `V <deg/s>` and `X <deg/s^2>` adjust nudge speed/accel (optional)

Tip (thesis runs):
- You can add a logger-only marker into `events.txt` without sending anything to the device by typing `# <note>` in the logger.
