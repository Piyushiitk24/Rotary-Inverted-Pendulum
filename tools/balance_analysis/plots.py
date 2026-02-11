from __future__ import annotations

from typing import Any, Optional

import numpy as np
import pandas as pd


def auto_symmetric_ylim(
    signal: np.ndarray,
    *,
    headroom: float = 1.15,
    step: float = 5.0,
    min_lim: float = 5.0,
) -> float:
    """
    Compute a "nice" symmetric y-limit (±ylim) around 0 from a 1D signal.

    Rule:
      abs_max = nanmax(|signal|)
      raw = abs_max * headroom
      ylim = max(min_lim, step * ceil(raw / step))
    """
    y = np.asarray(signal, dtype=float)
    if y.size == 0:
        return float(min_lim)
    abs_max = float(np.nanmax(np.abs(y)))
    if not np.isfinite(abs_max):
        return float(min_lim)
    raw = abs_max * float(headroom)
    ylim = max(float(min_lim), float(step) * float(np.ceil(raw / float(step))))
    return float(ylim)


def plot_timeseries(
    df: pd.DataFrame,
    title: str,
    lim_pend_deg: float = 30.0,
    lim_motor_deg: float = 80.0,
    max_acc_steps: float = 20000.0,
    ylim_pend_deg: Optional[float] = None,
    ylim_motor_deg: Optional[float] = None,
):
    import matplotlib.pyplot as plt

    t = df["time_s"].to_numpy(dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    # Alpha
    axes[0].plot(t, df["alpha_deg"], linewidth=0.9)
    axes[0].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[0].set_ylabel("α (deg)")
    alpha_ylim = float(ylim_pend_deg) if ylim_pend_deg is not None else float(lim_pend_deg + 5.0)
    axes[0].set_ylim(-alpha_ylim, +alpha_ylim)
    axes[0].grid(True, alpha=0.25)

    # Theta
    axes[1].plot(t, df["theta_deg"], linewidth=0.9)
    axes[1].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[1].axhline(+lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[1].axhline(-lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[1].set_ylabel("θ (deg)")
    theta_ylim = float(ylim_motor_deg) if ylim_motor_deg is not None else float(lim_motor_deg + 5.0)
    axes[1].set_ylim(-theta_ylim, +theta_ylim)
    axes[1].grid(True, alpha=0.25)

    # Control effort
    axes[2].plot(t, df["acc_cmd_steps_s2"], linewidth=0.9, color="purple")
    axes[2].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[2].axhline(+max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[2].axhline(-max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[2].set_ylabel("acc (steps/s²)")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylim(-max_acc_steps, +max_acc_steps)
    axes[2].grid(True, alpha=0.25)

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_phase_portrait(
    df: pd.DataFrame,
    title: str,
    lim_pend_deg: float = 30.0,
    lim_penddot_deg_s: float = 300.0,
    max_points: int = 5000,
):
    import matplotlib.pyplot as plt

    if len(df) > max_points and max_points > 0:
        stride = max(1, int(len(df) / max_points))
        dfp = df.iloc[::stride].copy()
    else:
        dfp = df

    fig, ax = plt.subplots(1, 1, figsize=(6.5, 6))
    ax.scatter(dfp["alpha_deg"], dfp["alpha_dot_deg_s"], s=2, alpha=0.35)
    ax.axhline(0, color="k", linewidth=0.7, alpha=0.4)
    ax.axvline(0, color="k", linewidth=0.7, alpha=0.4)
    ax.set_xlabel("α (deg)")
    ax.set_ylabel("α̇ (deg/s)")
    ax.set_xlim(-(lim_pend_deg + 5.0), +(lim_pend_deg + 5.0))
    ax.set_ylim(-lim_penddot_deg_s, +lim_penddot_deg_s)
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    return fig


def plot_effort_hist(df: pd.DataFrame, title: str, max_acc_steps: float = 20000.0):
    import matplotlib.pyplot as plt

    acc = df["acc_cmd_steps_s2"].to_numpy(dtype=float)
    fig, ax = plt.subplots(1, 1, figsize=(8, 4.5))
    ax.hist(acc, bins=100, range=(-max_acc_steps, +max_acc_steps), color="purple", alpha=0.8)
    ax.axvline(+max_acc_steps, color="r", linestyle=":", linewidth=1.0, alpha=0.7)
    ax.axvline(-max_acc_steps, color="r", linestyle=":", linewidth=1.0, alpha=0.7)
    ax.set_xlabel("acc (steps/s²)")
    ax.set_ylabel("count")
    ax.set_xlim(-max_acc_steps, +max_acc_steps)
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    return fig


def plot_nudge_tracking(
    df: pd.DataFrame,
    title: str,
    steps: list[dict[str, Any]],
    lim_pend_deg: float = 30.0,
    lim_motor_deg: float = 80.0,
    max_acc_steps: float = 20000.0,
    include_effort: bool = False,
    ylim_pend_deg: Optional[float] = None,
    ylim_motor_deg: Optional[float] = None,
):
    import matplotlib.pyplot as plt

    t = df["time_s"].to_numpy(dtype=float)
    theta = df["theta_deg"].to_numpy(dtype=float)
    alpha = df["alpha_deg"].to_numpy(dtype=float)

    nrows = 3 if include_effort else 2
    fig_h = 8.5 if include_effort else 7.0
    fig, axes = plt.subplots(nrows, 1, figsize=(12, fig_h), sharex=True)

    axes[0].plot(t, theta, linewidth=0.9, label="θ")
    axes[0].axhline(+lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[0].axhline(-lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)

    # Build target overlay (piecewise constant).
    if steps:
        xs = []
        ys = []
        for s in steps:
            xs.append(float(s["t_cmd_s"]))
            ys.append(float(s["target_deg"]))
        # Extend to end
        xs2 = xs + [float(steps[-1].get("t_next_cmd_s", t[-1]))]
        ys2 = ys + [ys[-1]]
        axes[0].step(xs2, ys2, where="post", linewidth=1.2, color="k", alpha=0.75, label="θ_target")

    axes[0].set_ylabel("θ (deg)")
    theta_ylim = float(ylim_motor_deg) if ylim_motor_deg is not None else float(lim_motor_deg + 5.0)
    axes[0].set_ylim(-theta_ylim, +theta_ylim)
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")

    axes[1].plot(t, alpha, linewidth=0.9, color="tab:blue")
    axes[1].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[1].set_ylabel("α (deg)")
    alpha_ylim = float(ylim_pend_deg) if ylim_pend_deg is not None else float(lim_pend_deg + 5.0)
    axes[1].set_ylim(-alpha_ylim, +alpha_ylim)
    axes[1].grid(True, alpha=0.25)

    if include_effort:
        axes[2].plot(t, df["acc_cmd_steps_s2"], linewidth=0.9, color="purple")
        axes[2].axhline(0, color="k", linewidth=0.7, alpha=0.4)
        axes[2].axhline(+max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axes[2].axhline(-max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axes[2].set_ylabel("acc (steps/s²)")
        axes[2].set_ylim(-max_acc_steps, +max_acc_steps)
        axes[2].grid(True, alpha=0.25)
        axes[2].set_xlabel("time (s)")
    else:
        axes[1].set_xlabel("time (s)")

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_lowfreq_spectrum(df: pd.DataFrame, title: str, col: str = "alpha_deg"):
    """
    Low-frequency spectrum only: the logger is ~50 Hz (Nyquist ~25 Hz).
    This plot is for low-frequency oscillations, not high-frequency chattering claims.
    """
    import matplotlib.pyplot as plt

    y = df[col].to_numpy(dtype=float)
    t = df["time_s"].to_numpy(dtype=float)
    if len(t) < 10:
        raise ValueError("Not enough samples for spectrum")

    dt = float(np.median(np.diff(t)))
    fs = 1.0 / max(dt, 1e-6)
    n = len(y)
    y0 = y - np.mean(y)
    win = np.hanning(n)
    Y = np.fft.rfft(y0 * win)
    f = np.fft.rfftfreq(n, d=dt)
    mag = np.abs(Y)

    fig, ax = plt.subplots(1, 1, figsize=(8, 4.5))
    ax.plot(f, mag, linewidth=0.9)
    ax.set_xlim(0, min(25.0, f.max()))
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("magnitude (a.u.)")
    ax.set_title(f"{title} (≤25 Hz; fs≈{fs:.1f} Hz)")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    return fig


def plot_sparse_series(t_s: np.ndarray, y: np.ndarray, title: str, ylabel: str):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 1, figsize=(10, 3.5))
    ax.plot(t_s, y, marker="o", markersize=3, linewidth=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    return fig
