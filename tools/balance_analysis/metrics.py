from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional

import numpy as np
import pandas as pd

from .types import Trial


# Defaults that match the firmware project (src/main.cpp).
DEFAULT_MAX_ACC_STEPS = 20000.0
DEFAULT_LIM_MOTOR_DEG = 80.0


def _rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def _robust_slope_deg_s(t_s: np.ndarray, y: np.ndarray) -> float:
    t_s = np.asarray(t_s, dtype=float)
    y = np.asarray(y, dtype=float)
    if t_s.size < 3:
        return float("nan")

    try:
        from scipy.stats import theilslopes

        slope, *_ = theilslopes(y, t_s)
        return float(slope)
    except Exception:
        # Fallback to least-squares.
        slope = np.polyfit(t_s, y, 1)[0]
        return float(slope)


def classify_outcome(
    trial: Trial,
    lim_motor_deg: float = DEFAULT_LIM_MOTOR_DEG,
    limit_margin_deg: float = 0.5,
) -> tuple[str, str]:
    """
    Determine success/failure for a trial.
    Returns (label, reason).
    """
    if trial.end_event and trial.end_event.kind == "FALLEN":
        lim = trial.end_event.limit or ""
        return "failed_fallen", f"FALLEN({lim})"

    df = trial.df
    if df is None or len(df) == 0:
        return "ambiguous", "empty_csv"

    # Prefer explicit clamped flag when available.
    if "position_clamped" in df.columns and int(df["position_clamped"].iloc[-1]) == 1:
        return "failed_limit", "clamped_end"

    if "theta_deg" in df.columns:
        if float(np.nanmax(np.abs(df["theta_deg"].to_numpy(dtype=float)))) >= (lim_motor_deg - limit_margin_deg):
            return "failed_limit", "theta_near_limit"

    return "success", "completed"


def compute_trial_metrics(
    trial: Trial,
    max_acc_steps: float = DEFAULT_MAX_ACC_STEPS,
    lim_motor_deg: float = DEFAULT_LIM_MOTOR_DEG,
) -> dict[str, Any]:
    df = trial.df
    outcome, outcome_reason = classify_outcome(trial, lim_motor_deg=lim_motor_deg)

    if df is None or len(df) == 0:
        return {
            "session": PathLikeToStr(trial.session_dir),
            "trial_index": trial.trial_index,
            "mode": trial.mode,
            "outcome": outcome,
            "outcome_reason": outcome_reason,
            "duration_s": 0.0,
        }

    t = df["time_s"].to_numpy(dtype=float) if "time_s" in df.columns else (df["timestamp_ms"].to_numpy() / 1000.0)

    alpha = df["alpha_deg"].to_numpy(dtype=float)
    alpha_dot = df["alpha_dot_deg_s"].to_numpy(dtype=float)
    theta = df["theta_deg"].to_numpy(dtype=float)
    theta_dot = df["theta_dot_deg_s"].to_numpy(dtype=float)

    acc = df["acc_cmd_steps_s2"].to_numpy(dtype=float)
    vel = df["vel_cmd_steps_s"].to_numpy(dtype=float)

    dur = float(t[-1] - t[0]) if t.size >= 2 else 0.0

    sat_mask = np.abs(acc) >= (0.99 * max_acc_steps)
    sat_pct = float(100.0 * np.mean(sat_mask)) if sat_mask.size else 0.0

    clamped_pct = float(100.0 * np.mean(df["position_clamped"].to_numpy(dtype=int) != 0)) if "position_clamped" in df.columns else float("nan")

    drift_slope = _robust_slope_deg_s(t, theta)

    # CSV dropout / missing lines indicators (device timestamp diffs).
    dt_ms = np.diff(df["timestamp_ms"].to_numpy(dtype=float)) if "timestamp_ms" in df.columns and len(df) >= 2 else np.array([])
    dt_med_ms = float(np.median(dt_ms)) if dt_ms.size else float("nan")
    dt_max_ms = float(np.max(dt_ms)) if dt_ms.size else float("nan")
    dt_gaps_gt_200 = int(np.sum(dt_ms > 200.0)) if dt_ms.size else 0

    row: dict[str, Any] = {
        "session": PathLikeToStr(trial.session_dir),
        "trial_index": int(trial.trial_index),
        "mode": trial.mode,
        "start_ms": int(trial.start_ms),
        "end_ms": int(trial.end_ms),
        "duration_s": dur,
        "outcome": outcome,
        "outcome_reason": outcome_reason,
        "alpha_rms_deg": _rms(alpha),
        "alpha_max_abs_deg": float(np.nanmax(np.abs(alpha))),
        "alpha_dot_rms_deg_s": _rms(alpha_dot),
        "alpha_dot_max_abs_deg_s": float(np.nanmax(np.abs(alpha_dot))),
        "theta_rms_deg": _rms(theta),
        "theta_max_abs_deg": float(np.nanmax(np.abs(theta))),
        "theta_dot_rms_deg_s": _rms(theta_dot),
        "theta_drift_slope_deg_s": drift_slope,
        "acc_rms_steps_s2": _rms(acc),
        "acc_max_abs_steps_s2": float(np.nanmax(np.abs(acc))),
        "vel_rms_steps_s": _rms(vel),
        "vel_max_abs_steps_s": float(np.nanmax(np.abs(vel))),
        "sat_pct": sat_pct,
        "clamped_pct": clamped_pct,
        "dt_med_ms": dt_med_ms,
        "dt_max_ms": dt_max_ms,
        "dt_gaps_gt_200": dt_gaps_gt_200,
    }

    # Attach a few key params if known (useful for thesis tables).
    for key in [
        "motorSign",
        "ALPHA_SIGN",
        "THETA_SIGN",
        "CTRL_SIGN",
        "smcLambda",
        "smcKDegS2",
        "smcPhiDegS",
        "smcSign",
        "smcBaseScale",
        "smcFullK",
        "smcFullLambdaTheta",
    ]:
        if key in trial.params:
            row[key] = trial.params[key]

    return row


def compute_nudge_step_metrics(
    trial: Trial,
    steps: list[dict[str, Any]],
    response_pos_thresh_deg: float = 0.5,
    response_vel_thresh_deg_s: float = 5.0,
    settle_band_deg: float = 1.0,
    min_settle_s: float = 1.0,
) -> list[dict[str, Any]]:
    """
    Compute step-response metrics for "nudge mode" trials.

    Expected input steps (per step):
      - t_cmd_s: float (time relative to trial start)
      - target_deg: float
      - t_next_cmd_s: float (optional; end of step window)
    """
    df = trial.df
    if df is None or len(df) == 0:
        return []

    out: list[dict[str, Any]] = []

    t = df["time_s"].to_numpy(dtype=float)
    theta = df["theta_deg"].to_numpy(dtype=float)
    theta_dot = df["theta_dot_deg_s"].to_numpy(dtype=float)
    alpha = df["alpha_deg"].to_numpy(dtype=float)

    for j, step in enumerate(steps):
        t_cmd = float(step["t_cmd_s"])
        target = float(step["target_deg"])
        t_end = float(step.get("t_next_cmd_s", t[-1]))

        if t_end <= t_cmd:
            continue

        # Baseline from 1s before command (if available).
        pre_mask = (t >= (t_cmd - 1.0)) & (t < t_cmd)
        theta0 = float(np.median(theta[pre_mask])) if np.any(pre_mask) else float(theta[np.searchsorted(t, t_cmd)])

        # Response start: when theta moves or theta_dot spikes after command.
        post_mask = (t >= t_cmd) & (t <= t_end)
        if not np.any(post_mask):
            continue

        idx_post = np.where(post_mask)[0]
        idx_start = idx_post[0]
        for idx in idx_post:
            if abs(theta[idx] - theta0) >= response_pos_thresh_deg or abs(theta_dot[idx]) >= response_vel_thresh_deg_s:
                idx_start = idx
                break
        t_start = float(t[idx_start])

        # Window for metrics.
        win_mask = (t >= t_start) & (t <= t_end)
        if not np.any(win_mask):
            continue

        t_win = t[win_mask]
        th_win = theta[win_mask]

        amp = target - theta0
        if abs(amp) < 1e-6:
            continue

        # Rise time (10–90%).
        y10 = theta0 + 0.1 * amp
        y90 = theta0 + 0.9 * amp

        def _first_cross(y: float) -> Optional[float]:
            if amp > 0:
                idxs = np.where(th_win >= y)[0]
            else:
                idxs = np.where(th_win <= y)[0]
            return float(t_win[idxs[0]]) if idxs.size else None

        t10 = _first_cross(y10)
        t90 = _first_cross(y90)
        rise_s = (t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else float("nan")

        # Overshoot.
        if amp > 0:
            overshoot = float(np.nanmax(th_win) - target)
        else:
            overshoot = float(target - np.nanmin(th_win))
        overshoot = max(0.0, overshoot)

        # Settling time: first time after which |theta-target| stays within band for >= min_settle_s.
        settle_s = float("nan")
        band = settle_band_deg
        if band > 0 and min_settle_s > 0:
            inside = np.abs(th_win - target) <= band
            if np.any(inside):
                # Find earliest index where the next N samples are all inside.
                dt = float(np.median(np.diff(t))) if t.size >= 3 else 0.02
                need = max(1, int(round(min_settle_s / max(dt, 1e-3))))
                inside_i = inside.astype(int)
                # running sum over window
                csum = np.cumsum(np.insert(inside_i, 0, 0))
                ok = (csum[need:] - csum[:-need]) >= need
                if np.any(ok):
                    first_ok = int(np.where(ok)[0][0])
                    settle_s = float(t_win[first_ok] - t_start)

        # Steady-state error: median over last 2 seconds of this step window.
        ss_mask = (t_win >= (t_end - 2.0)) & (t_win <= t_end)
        theta_ss = float(np.median(th_win[ss_mask])) if np.any(ss_mask) else float(th_win[-1])
        ss_err = theta_ss - target

        # Upright disturbance during step window.
        alpha_win = alpha[win_mask]
        alpha_rms = _rms(alpha_win)
        alpha_max = float(np.nanmax(np.abs(alpha_win))) if alpha_win.size else float("nan")

        out.append(
            {
                "session": PathLikeToStr(trial.session_dir),
                "trial_index": int(trial.trial_index),
                "mode": trial.mode,
                "step_index": int(j),
                "t_cmd_s": t_cmd,
                "t_start_s": t_start,
                "t_end_s": t_end,
                "theta0_deg": theta0,
                "target_deg": target,
                "amp_deg": amp,
                "rise_time_s": rise_s,
                "overshoot_deg": overshoot,
                "settle_time_s": settle_s,
                "theta_ss_deg": theta_ss,
                "theta_ss_err_deg": ss_err,
                "alpha_rms_deg": alpha_rms,
                "alpha_max_abs_deg": alpha_max,
            }
        )

    return out


def PathLikeToStr(p: Any) -> str:
    try:
        return str(p)
    except Exception:
        return "<path>"

