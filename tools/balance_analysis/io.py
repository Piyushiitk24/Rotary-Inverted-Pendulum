from __future__ import annotations

from pathlib import Path
from typing import List

import pandas as pd


def load_balance_csv(session_dir: str | Path) -> pd.DataFrame:
    """
    Load a session's balance_log.csv and add a time_s column relative to the first sample.

    Expected columns from tools/run_balance.py:
      timestamp_ms, alpha_deg, alpha_dot_deg_s, theta_deg, theta_dot_deg_s,
      acc_cmd_steps_s2, vel_cmd_steps_s, pos_meas_steps, position_clamped
    """
    session_dir = Path(session_dir)
    csv_path = session_dir / "balance_log.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing balance_log.csv: {csv_path}")

    df = pd.read_csv(csv_path)
    if "timestamp_ms" not in df.columns:
        raise ValueError(f"CSV missing timestamp_ms column: {csv_path}")

    # Ensure sorted by device time.
    df = df.sort_values("timestamp_ms", kind="stable").reset_index(drop=True)

    t0 = float(df["timestamp_ms"].iloc[0])
    df["time_s"] = (df["timestamp_ms"] - t0) / 1000.0
    return df


def csv_gap_segments(df: pd.DataFrame, gap_ms: int = 500) -> List[pd.DataFrame]:
    """
    Split a telemetry DataFrame into segments when timestamp_ms jumps by more than gap_ms.

    IMPORTANT: This is a helper only. True "trial" segmentation should use events markers
    (ENGAGED/FALLEN/DISARMED) when alignment is reliable.
    """
    if df.empty:
        return []

    if "timestamp_ms" not in df.columns:
        raise ValueError("df must contain timestamp_ms")

    t = df["timestamp_ms"].to_numpy()
    gaps = (t[1:] - t[:-1]) > gap_ms
    split_idx = (gaps.nonzero()[0] + 1).tolist()

    starts = [0] + split_idx
    ends = split_idx + [len(df)]

    return [df.iloc[s:e].reset_index(drop=True) for s, e in zip(starts, ends)]

