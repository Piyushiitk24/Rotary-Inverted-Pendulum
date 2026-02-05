# Experiment Quickstart (Upright Balancing)

This repo is driven by single-letter serial commands. This page lists the **minimum** commands + the **standard workflow** to run repeatable balance experiments with logging.

## 1) Start a session (PC)

1. Upload firmware (PlatformIO):
   - `platformio run -t upload`
2. Start the logger:
   - `python3 tools/run_balance.py`
3. Logs are created here each run:
   - `logs/balance/session_YYYYMMDD_HHMMSS/` (`balance_log.csv`, `events.txt`)

Notes:
- If the logger can’t connect, set `PORT = ...` at the top of `tools/run_balance.py`.

## 2) Minimal command cheat sheet

**Safety / required**
- `!`  Emergency disarm (stops motor + disables outputs). Safe anytime.
- `Z`  Calibrate (arm centered; pendulum upright). Do this every attempt.
- `E`  Arm/disarm toggle. When armed, it auto-engages when upright & still (`ENGAGED!`).

**Controller select**
- `C 0`  Linear controller (baseline)
- `C 1`  Nonlinear SMC controller

**SMC-only essentials (IDLE only)**
- `Q 1` / `Q -1`  Flip SMC direction if it reacts the wrong way (default: `Q 1`)
- `O <0..2>`      Base-centering strength (default: `O 1.0`)

Important: firmware accepts `C/J/K/P/Q/O` **only in IDLE** (disarmed). If you’re engaged, send `!` first.
After power-up you start in `STATE_CALIBRATE`, so do `Z` first, then `C ...` (then `E`).

## 3) Workflow A — Linear baseline (fast sanity check)

In the logger terminal:

1. `Z`
2. `C 0` (optional; default)
3. `E` → wait for `ENGAGED!`
4. Let it run ~10–30s, then `!` to stop.

## 4) Workflow B — SMC upright balance (recommended)

In the logger terminal:

1. `Z`
2. `C 1`
3. `E` → wait for `ENGAGED!`

If something looks wrong:
- Wrong direction reaction: `!`, then `Q -1`, then re-run (`Z`, `E`)
- Arm slowly drifts: increase `O` (IDLE only), then re-run (`Z`, `E`)

What to watch (1 Hz lines):
- `[STATUS] ... mode=SMC gate=1` → base centering is active (arm should stay near center).
- If `gate=0` most of the time, SMC will still balance upright, but it won’t strongly “pull back” the arm (because α/α̇ aren’t inside the gate window).

Stop conditions (don’t fight it):
- If you see `FALLEN ...` or `[WARN] BASE_STALL? ...`, send `!`, re-center the arm, and restart from `Z`.

## 5) Optional — Base “nudge” while balancing

If you want to command base position while balancing:
- `T <deg>` sets a new base target angle (start small: `T 5`, `T -5`, `T 0`)
- `V <deg/s>` and `X <deg/s^2>` adjust nudge speed/accel (optional)
