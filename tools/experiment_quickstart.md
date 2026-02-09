# Experiment Quickstart (Upright Balancing)

This repo is driven by single-letter serial commands. This page lists the **minimum** commands + the **standard workflow** to run repeatable balance experiments with logging.

## 0) The “3 layers” (important)

This project produces results in three layers:

1. **Logger session** (PC): one run of `tools/run_balance.py` creates one folder:
   - `logs/balance/session_YYYYMMDD_HHMMSS/`
2. **Trial** (device): inside that folder, each time the firmware prints `ENGAGED!` and then later `DISARMED` / `FALLEN ...` is one *trial* (one “ENGAGED block”).
3. **Analysis report** (PC): one run of `tools/analyze_experiments.py` creates one folder:
   - `reports/thesis/<timestamp>/` (tables + figures)

If you remember only one thing: **run the logger to create sessions, then run the analyzer on a set of sessions to create a report**.

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

## 1.5) Thesis results workflow (recommended)

For thesis-quality comparisons, avoid mixing controller modes inside the same session folder.

Recommended collection plan (your current plan; clean and defensible):

1. **Hold / no disturbance**: 3 sessions (one per mode: `C 0`, `C 1`, `C 2`)
2. **Commanded nudge** (`T/V/X`): 1 session (recommend `C 0` only)
3. **Finger disturbance** (manual taps/pushes): 3 sessions (one per mode)

That is **7 logger sessions** total.

After data collection, generate **one thesis report** from those 7 sessions using a **manifest** that labels each session as `hold`, `nudge`, or `tap` (so the analyzer produces grouped figures + tables and doesn’t mix experiment types).

Recommended (single command):
- `./.venv/bin/python tools/analyze_experiments.py --manifest experiments/manifest_thesis_20260209.json --out reports/thesis/report_thesis_20260209`

Alternative (older approach): run 3 separate reports by passing 3-session/1-session/3-session subsets via `--sessions`.

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
- `D <k>`         SMC4 `k` (>=0; couples `thetaDotErr` in the surface) (default: `D 0.5`)
- `L <val>`       SMC4 `lambda_theta` (1/s; **must be > 0**) (default: `L 2.0`)

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
4. (Optional) `D 0.5` and `L 2.0` (defaults; set explicitly if you changed them earlier)
5. `Z`
6. `E` → wait for `ENGAGED!`

Notes:
- `O` controls base-centering assist while upright (works in both `C 1` and `C 2`). If the arm drifts, increase `O` (disarmed only).
- If SMC4 runs away in the wrong direction: `!` then flip `Q` (`Q -1`), then restart (`Z`, `E`).
- If SMC4 is too aggressive: reduce `D` (move it closer to 0) first; then reduce `L` if needed.
- If you hear a “tick” and see `[WARN] BASE_STALL? ...`: the stepper is missing steps / not moving despite large `velCmd`. Reduce tap strength, or tune SMC4 to be less aggressive (increase `P` and/or reduce `K`). Newer firmware also adds SMC4-only internal speed/acc clamps to reduce these events.

## 6) Optional — Base “nudge” while balancing

If you want to command base position while balancing:
- `T <deg>` sets a new base target angle (start small: `T 5`, `T -5`, `T 0`)
- `V <deg/s>` and `X <deg/s^2>` adjust nudge speed/accel (optional)

Tip (thesis runs):
- You can add a logger-only marker into `events.txt` without sending anything to the device by typing `# <note>` in the logger.

## 7) From logs → thesis figures (what to run on PC)

### Step 1: Collect sessions (logger)

For each session folder you want:
1. Run logger: `./.venv/bin/python tools/run_balance.py`
2. Run your trials (each trial is `ENGAGED!` → `!` or `FALLEN`)
3. Quit logger (`q`) to finish the session folder

Tip: within one session you can repeat multiple trials by doing:
- `Z` → `E` → wait → `!` → (repeat)

### Step 2: Analyze sessions into a report folder

Run the analyzer on the sessions you want in *one* report:

- Recommended (manifest; best for thesis):
  - `./.venv/bin/python tools/analyze_experiments.py --manifest experiments/manifest_thesis_20260209.json --out reports/thesis/report_thesis_20260209`

- Analyze by explicit session list:
  - `./.venv/bin/python tools/analyze_experiments.py --sessions logs/balance/session_... logs/balance/session_...`
- Or by glob:
  - `./.venv/bin/python tools/analyze_experiments.py --glob 'logs/balance/session_20260206_*'`

Optional but recommended: use `--out` to keep your reports clearly named:
- `./.venv/bin/python tools/analyze_experiments.py --sessions <3 hold sessions> --out reports/thesis/hold_<timestamp>`
- `./.venv/bin/python tools/analyze_experiments.py --sessions <1 nudge session> --out reports/thesis/nudge_<timestamp>`
- `./.venv/bin/python tools/analyze_experiments.py --sessions <3 finger sessions> --out reports/thesis/finger_<timestamp>`

### Step 3: View results (optional notebook)

You can use the report outputs directly:
- `reports/thesis/<report>/metrics_trials.csv` and `metrics_summary_by_mode.csv`
- `reports/thesis/<report>/figures/` (PNG/PDF)

Or open the polished notebook:
- `tools/thesis_results.ipynb`

You typically run:
- `tools/analyze_experiments.py` **once per report** (e.g., once after you finish all 7 sessions), then
- open `tools/thesis_results.ipynb` **once** to browse/export the figures/tables.

The notebook auto-loads the **latest** `reports/thesis/*` folder by default.
If you want to view an older report, edit the `report_dir = ...` cell in the notebook.

### Step 4: Export into `thesis/` and build the PDF (report writing)

The analysis report under `reports/` is great for exploration, but the thesis is built from a stable set of exported assets:

1. Export figures/tables + write `thesis/metadata.tex` + generate the modelling chapter tex:
   - `./.venv/bin/python tools/export_thesis_assets.py --report reports/thesis/report_thesis_20260209 --manifest experiments/manifest_thesis_20260209.json --out thesis`
2. Build the thesis:
   - `cd thesis && latexmk`

Output PDF:
- `thesis/build/main.pdf`
