from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import pandas as pd

from .types import Alignment, Match, StatusEvent


@dataclass(frozen=True)
class MatchConfig:
    # Scales (units) to normalize squared error terms for status↔csv matching.
    alpha_deg: float = 5.0
    alpha_dot_deg_s: float = 100.0
    theta_deg: float = 20.0
    theta_dot_deg_s: float = 100.0
    acc_cmd_steps_s2: float = 5000.0
    vel_cmd_steps_s: float = 500.0

    # Minimum number of matches required to attempt time alignment.
    min_matches: int = 5

    # Median absolute residual threshold to declare alignment reliable (ms).
    reliable_median_abs_residual_ms: float = 80.0

    # Greedy time-guided matching:
    # After the first match, we predict the next device timestamp using the previous match and the
    # host-time delta between successive [STATUS] lines (device prints them ~1 Hz).
    #
    # This prevents ambiguous steady-state values (near 0) from matching to a totally different
    # engage segment later in the CSV, which would collapse the remaining matches.
    time_window_ms: float = 6000.0
    time_penalty_ms: float = 400.0

    # Clamp for the online slope estimate (device ms per host s) used to guide the next match.
    slope_guess_min_ms_per_s: float = 800.0
    slope_guess_max_ms_per_s: float = 1200.0


def match_status_to_csv(
    status_events: List[StatusEvent],
    df_csv: pd.DataFrame,
    cfg: Optional[MatchConfig] = None,
) -> List[Match]:
    """
    Match 1 Hz [STATUS] lines in events.txt to rows in balance_log.csv by minimizing a
    normalized squared error over common fields.

    This enables robust host↔device time alignment even though events.txt timestamps are host-time.
    """
    if cfg is None:
        cfg = MatchConfig()

    required_cols = [
        "alpha_deg",
        "alpha_dot_deg_s",
        "theta_deg",
        "theta_dot_deg_s",
        "acc_cmd_steps_s2",
        "vel_cmd_steps_s",
        "timestamp_ms",
    ]
    missing = [c for c in required_cols if c not in df_csv.columns]
    if missing:
        raise ValueError(f"CSV missing required columns for alignment: {missing}")

    # numpy views for speed
    a = df_csv["alpha_deg"].to_numpy(dtype=float)
    ad = df_csv["alpha_dot_deg_s"].to_numpy(dtype=float)
    th = df_csv["theta_deg"].to_numpy(dtype=float)
    thd = df_csv["theta_dot_deg_s"].to_numpy(dtype=float)
    acc = df_csv["acc_cmd_steps_s2"].to_numpy(dtype=float)
    vel = df_csv["vel_cmd_steps_s"].to_numpy(dtype=float)
    ts = df_csv["timestamp_ms"].to_numpy(dtype=int)

    matches: List[Match] = []
    prev_idx = -1
    prev_host_s: Optional[float] = None
    prev_ts_ms: Optional[float] = None
    slope_guess_ms_per_s: float = 1000.0

    for st in status_events:
        start = prev_idx + 1
        if start >= len(df_csv):
            break

        # Candidate window:
        # - First [STATUS]: we don't know where we are, so allow the full tail.
        # - Subsequent [STATUS]: search a timestamp window around the predicted device timestamp.
        if prev_host_s is None or prev_ts_ms is None:
            idxs = np.arange(start, len(df_csv))
            time_pen = 0.0
            pred_ts_ms = None
        else:
            delta_host_s = float(st.host_s - prev_host_s)
            if delta_host_s <= 0.0:
                delta_host_s = 1.0
            pred_ts_ms = float(prev_ts_ms + delta_host_s * slope_guess_ms_per_s)

            lo_ms = pred_ts_ms - cfg.time_window_ms
            hi_ms = pred_ts_ms + cfg.time_window_ms

            # ts is sorted (load_balance_csv sorts), so we can use searchsorted.
            lo_idx = int(np.searchsorted(ts, lo_ms, side="left"))
            hi_idx = int(np.searchsorted(ts, hi_ms, side="right"))

            lo_idx = max(lo_idx, start)
            hi_idx = min(max(hi_idx, lo_idx + 1), len(df_csv))

            idxs = np.arange(lo_idx, hi_idx)
            if len(idxs) == 0:
                idxs = np.arange(start, len(df_csv))

            # Time penalty is normalized; stays small when predicted time is correct and heavily
            # penalizes large jumps to unrelated segments.
            time_pen = ((ts[idxs].astype(float) - pred_ts_ms) / max(cfg.time_penalty_ms, 1.0)) ** 2

        field_err = (
            ((a[idxs] - st.alpha_deg) / cfg.alpha_deg) ** 2
            + ((ad[idxs] - st.alpha_dot_deg_s) / cfg.alpha_dot_deg_s) ** 2
            + ((th[idxs] - st.theta_deg) / cfg.theta_deg) ** 2
            + ((thd[idxs] - st.theta_dot_deg_s) / cfg.theta_dot_deg_s) ** 2
            + ((acc[idxs] - st.acc_cmd_steps_s2) / cfg.acc_cmd_steps_s2) ** 2
            + ((vel[idxs] - st.vel_cmd_steps_s) / cfg.vel_cmd_steps_s) ** 2
        )

        total_err = field_err + time_pen

        best_rel = int(np.argmin(total_err))
        best_idx = int(idxs[best_rel])
        best_err = float(total_err[best_rel])

        matches.append(
            Match(
                status=st,
                csv_index=best_idx,
                csv_timestamp_ms=int(ts[best_idx]),
                error=best_err,
            )
        )
        prev_idx = best_idx
        prev_host_s = float(st.host_s)
        prev_ts_ms = float(ts[best_idx])

        # Update slope guess from the first match to current (online), to reduce drift over long sessions.
        if len(matches) >= 2:
            h0 = float(matches[0].status.host_s)
            t0 = float(matches[0].csv_timestamp_ms)
            dh = prev_host_s - h0
            if dh > 0.5:
                slope_est = (prev_ts_ms - t0) / dh
                slope_guess_ms_per_s = float(
                    np.clip(
                        slope_est,
                        cfg.slope_guess_min_ms_per_s,
                        cfg.slope_guess_max_ms_per_s,
                    )
                )

    return matches


def fit_host_to_device_time(matches: List[Match], cfg: Optional[MatchConfig] = None) -> Alignment:
    """
    Fit a linear mapping device_ms ≈ a * host_s + b using matched [STATUS]↔CSV pairs.
    """
    if cfg is None:
        cfg = MatchConfig()

    if len(matches) < cfg.min_matches:
        return Alignment(
            reliable=False,
            n_matches=len(matches),
            slope_ms_per_s=float("nan"),
            intercept_ms=float("nan"),
            median_abs_residual_ms=float("inf"),
            max_abs_residual_ms=float("inf"),
            matches=matches,
        )

    host_s = np.array([m.status.host_s for m in matches], dtype=float)
    device_ms = np.array([m.csv_timestamp_ms for m in matches], dtype=float)

    # Linear least squares.
    slope, intercept = np.polyfit(host_s, device_ms, 1)
    pred = slope * host_s + intercept
    resid = device_ms - pred

    med_abs = float(np.median(np.abs(resid)))
    max_abs = float(np.max(np.abs(resid)))

    reliable = med_abs <= cfg.reliable_median_abs_residual_ms

    return Alignment(
        reliable=reliable,
        n_matches=len(matches),
        slope_ms_per_s=float(slope),
        intercept_ms=float(intercept),
        median_abs_residual_ms=med_abs,
        max_abs_residual_ms=max_abs,
        matches=matches,
    )
