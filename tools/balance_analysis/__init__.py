"""
Balance log analysis utilities for the Rotary/Furuta Inverted Pendulum project.

This package is intentionally small and dependency-light (pandas/numpy/matplotlib/scipy).
It provides robust parsing of:
  - balance_log.csv (device-time telemetry at ~50 Hz)
  - events.txt (host-time events + periodic [STATUS]/[DBG] lines)

Primary entry points are used by tools/analyze_experiments.py.
"""

from .io import load_balance_csv, csv_gap_segments
from .events import parse_events
from .align import match_status_to_csv, fit_host_to_device_time
from .trials import extract_trials
from .metrics import compute_trial_metrics, compute_nudge_step_metrics

__all__ = [
    "load_balance_csv",
    "csv_gap_segments",
    "parse_events",
    "match_status_to_csv",
    "fit_host_to_device_time",
    "extract_trials",
    "compute_trial_metrics",
    "compute_nudge_step_metrics",
]

