from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import pandas as pd

from .align import Alignment
from .io import csv_gap_segments
from .types import EndEvent, EngageEvent, Events, Trial


def _mode_at(events: Events, host_s: float) -> str:
    mode = "UNK"
    for ev in events.ctrl:
        if ev.host_s <= host_s:
            mode = ev.mode
        else:
            break
    return mode


def _params_at(events: Events, host_s: float) -> dict[str, Any]:
    params: dict[str, Any] = {}
    for snap in events.params:
        if snap.host_s <= host_s:
            params.update(snap.values)
        else:
            break
    return params


def _end_event_for_engage(events: Events, engage: EngageEvent, next_engage: Optional[EngageEvent]) -> Optional[EndEvent]:
    for end in events.ended:
        if end.host_dt <= engage.host_dt:
            continue
        if next_engage and end.host_dt >= next_engage.host_dt:
            continue
        return end
    return None


def extract_trials(
    df_csv: pd.DataFrame,
    events: Events,
    alignment: Optional[Alignment] = None,
    gap_ms: int = 500,
    pad_ms: int = 500,
) -> list[Trial]:
    """
    Extract engaged trials from a session.

    If alignment is reliable:
      - Use ENGAGED!/FALLEN/DISARMED host-time markers mapped into device_ms and slice CSV by time.
      - Do NOT split trials purely on CSV gaps (those can be serial hiccups).

    If alignment is not reliable:
      - Fall back to splitting CSV by large device timestamp gaps and mapping segments to ENGAGED blocks by order.
      - Emit per-trial diagnostics so any ambiguity is visible in the report outputs.
    """
    session_dir = events.session_dir

    if df_csv.empty:
        return []

    if "timestamp_ms" not in df_csv.columns:
        raise ValueError("df_csv must contain timestamp_ms")

    trials: list[Trial] = []

    eng = events.engaged
    if not eng:
        # No ENGAGED markers; treat entire CSV as one unknown trial.
        start_ms = int(df_csv["timestamp_ms"].iloc[0])
        end_ms = int(df_csv["timestamp_ms"].iloc[-1])
        trials.append(
            Trial(
                session_dir=session_dir,
                trial_index=0,
                mode="UNK",
                start_ms=start_ms,
                end_ms=end_ms,
                df=df_csv.reset_index(drop=True),
                params=_params_at(events, host_s=float("inf")),
                diagnostics={"note": "no_engaged_markers"},
            )
        )
        return trials

    reliable = bool(alignment and alignment.reliable)

    if reliable:
        for i, e0 in enumerate(eng):
            e1 = eng[i + 1] if (i + 1) < len(eng) else None
            end_ev = _end_event_for_engage(events, e0, e1)

            start_ms_pred = int(round(alignment.host_s_to_device_ms(e0.host_s)))
            if end_ev:
                end_ms_pred = int(round(alignment.host_s_to_device_ms(end_ev.host_s)))
            else:
                end_ms_pred = int(df_csv["timestamp_ms"].iloc[-1])

            start_ms_win = start_ms_pred - pad_ms
            end_ms_win = end_ms_pred + pad_ms

            df_trial = df_csv[(df_csv["timestamp_ms"] >= start_ms_win) & (df_csv["timestamp_ms"] <= end_ms_win)].copy()
            if df_trial.empty:
                trials.append(
                    Trial(
                        session_dir=session_dir,
                        trial_index=i,
                        mode=_mode_at(events, e0.host_s),
                        start_ms=start_ms_pred,
                        end_ms=end_ms_pred,
                        df=df_trial,
                        engaged_event=e0,
                        end_event=end_ev,
                        params=_params_at(events, e0.host_s),
                        diagnostics={"empty_csv_slice": True, "align_used": True},
                    )
                )
                continue

            df_trial = df_trial.reset_index(drop=True)
            start_ms = int(df_trial["timestamp_ms"].iloc[0])
            end_ms = int(df_trial["timestamp_ms"].iloc[-1])

            trials.append(
                Trial(
                    session_dir=session_dir,
                    trial_index=i,
                    mode=_mode_at(events, e0.host_s),
                    start_ms=start_ms,
                    end_ms=end_ms,
                    df=df_trial,
                    engaged_event=e0,
                    end_event=end_ev,
                    params=_params_at(events, e0.host_s),
                    diagnostics={
                        "align_used": True,
                        "start_ms_pred": start_ms_pred,
                        "end_ms_pred": end_ms_pred,
                        "pad_ms": pad_ms,
                    },
                )
            )

        return trials

    # Fallback: CSV gap-based segments mapped to ENGAGED blocks by order.
    segments = csv_gap_segments(df_csv, gap_ms=gap_ms)

    for i, seg in enumerate(segments):
        e0 = eng[i] if i < len(eng) else None
        e1 = eng[i + 1] if (i + 1) < len(eng) else None
        end_ev = _end_event_for_engage(events, e0, e1) if e0 else None

        mode = _mode_at(events, e0.host_s) if e0 else "UNK"
        params = _params_at(events, e0.host_s) if e0 else _params_at(events, float("inf"))

        start_ms = int(seg["timestamp_ms"].iloc[0])
        end_ms = int(seg["timestamp_ms"].iloc[-1])

        trials.append(
            Trial(
                session_dir=session_dir,
                trial_index=i,
                mode=mode,
                start_ms=start_ms,
                end_ms=end_ms,
                df=seg,
                engaged_event=e0,
                end_event=end_ev,
                params=params,
                diagnostics={
                    "align_used": False,
                    "gap_ms": gap_ms,
                    "seg_index": i,
                    "seg_count": len(segments),
                    "engaged_count": len(eng),
                    "unmatched_engaged": max(0, len(eng) - len(segments)),
                    "unmatched_segments": max(0, len(segments) - len(eng)),
                },
            )
        )

    return trials

