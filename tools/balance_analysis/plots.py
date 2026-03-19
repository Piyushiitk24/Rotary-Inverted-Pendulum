from __future__ import annotations

from typing import Any, Optional, Sequence

import numpy as np
import pandas as pd


MODE_COLORS = {
    "LIN": "tab:blue",
    "SMC": "tab:orange",
    "SMC4": "tab:green",
}


def mode_color(mode: str) -> str:
    return MODE_COLORS.get(str(mode).upper(), "tab:blue")


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
    soft_motor_deg: Optional[float] = 78.0,
    abort_pend_deg: Optional[float] = None,
    effort_ylim_steps: Optional[float] = None,
):
    import matplotlib.pyplot as plt

    t = df["time_s"].to_numpy(dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    # Alpha
    axes[0].plot(t, df["alpha_deg"], linewidth=0.9, color="tab:blue")
    axes[0].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[0].axhline(+lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[0].axhline(-lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    if abort_pend_deg is not None:
        axes[0].axhline(+abort_pend_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
        axes[0].axhline(-abort_pend_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
    axes[0].set_ylabel("α (deg)")
    alpha_ylim = float(ylim_pend_deg) if ylim_pend_deg is not None else float(lim_pend_deg + 5.0)
    axes[0].set_ylim(-alpha_ylim, +alpha_ylim)
    axes[0].grid(True, alpha=0.25)

    # Theta
    axes[1].plot(t, df["theta_deg"], linewidth=0.9, color="tab:blue")
    axes[1].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    if soft_motor_deg is not None:
        axes[1].axhline(+soft_motor_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
        axes[1].axhline(-soft_motor_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
    axes[1].axhline(+lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[1].axhline(-lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[1].set_ylabel("θ (deg)")
    theta_ylim = float(ylim_motor_deg) if ylim_motor_deg is not None else float(lim_motor_deg + 5.0)
    axes[1].set_ylim(-theta_ylim, +theta_ylim)
    axes[1].grid(True, alpha=0.25)

    # Control effort
    effort_ylim = float(effort_ylim_steps) if effort_ylim_steps is not None else float(max_acc_steps)
    axes[2].plot(t, df["acc_cmd_steps_s2"], linewidth=0.9, color="purple")
    axes[2].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[2].axhline(+max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[2].axhline(-max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[2].set_ylabel("u (steps/s²)")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylim(-effort_ylim, +effort_ylim)
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
    ax.axvline(+lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.4)
    ax.axvline(-lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.4)
    ax.set_xlabel("α (deg)")
    ax.set_ylabel("α̇ (deg/s)")
    ax.set_xlim(-(lim_pend_deg + 5.0), +(lim_pend_deg + 5.0))
    ax.set_ylim(-lim_penddot_deg_s, +lim_penddot_deg_s)
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    return fig


def plot_effort_hist(
    df: pd.DataFrame,
    title: str,
    max_acc_steps: float = 20000.0,
    hist_xlim_steps: Optional[float] = None,
):
    import matplotlib.pyplot as plt

    xlim = float(hist_xlim_steps) if hist_xlim_steps is not None else float(max_acc_steps)
    acc = df["acc_cmd_steps_s2"].to_numpy(dtype=float)
    fig, ax = plt.subplots(1, 1, figsize=(8, 4.5))
    ax.hist(acc, bins=100, range=(-xlim, +xlim), color="purple", alpha=0.8)
    ax.axvline(+max_acc_steps, color="r", linestyle=":", linewidth=1.0, alpha=0.7)
    ax.axvline(-max_acc_steps, color="r", linestyle=":", linewidth=1.0, alpha=0.7)
    ax.set_xlabel("u (steps/s²)")
    ax.set_ylabel("count")
    ax.set_xlim(-xlim, +xlim)
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
    soft_motor_deg: Optional[float] = 78.0,
    abort_pend_deg: Optional[float] = None,
    effort_ylim_steps: Optional[float] = None,
    settle_band_deg: Optional[float] = None,
):
    import matplotlib.pyplot as plt

    t = df["time_s"].to_numpy(dtype=float)
    theta = df["theta_deg"].to_numpy(dtype=float)
    alpha = df["alpha_deg"].to_numpy(dtype=float)

    nrows = 3 if include_effort else 2
    fig_h = 8.5 if include_effort else 7.0
    fig, axes = plt.subplots(nrows, 1, figsize=(12, fig_h), sharex=True)

    axes[0].plot(t, theta, linewidth=0.9, label="θ", color="tab:blue")
    if soft_motor_deg is not None:
        axes[0].axhline(+soft_motor_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
        axes[0].axhline(-soft_motor_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
    axes[0].axhline(+lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[0].axhline(-lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)

    if steps:
        xs = [float(s["t_cmd_s"]) for s in steps]
        ys = [float(s["target_deg"]) for s in steps]
        xs2 = xs + [float(steps[-1].get("t_next_cmd_s", t[-1]))]
        ys2 = ys + [ys[-1]]
        y2 = np.asarray(ys2, dtype=float)
        axes[0].step(xs2, y2, where="post", linewidth=1.2, color="k", alpha=0.75, label="θ target")
        if settle_band_deg is not None:
            band = float(settle_band_deg)
            axes[0].fill_between(
                xs2,
                y2 - band,
                y2 + band,
                step="post",
                color="0.6",
                alpha=0.18,
                label=f"±{band:g}° settle band",
            )

    axes[0].set_ylabel("θ (deg)")
    theta_ylim = float(ylim_motor_deg) if ylim_motor_deg is not None else float(lim_motor_deg + 5.0)
    axes[0].set_ylim(-theta_ylim, +theta_ylim)
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")

    axes[1].plot(t, alpha, linewidth=0.9, color="tab:blue")
    axes[1].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    axes[1].axhline(+lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    axes[1].axhline(-lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
    if abort_pend_deg is not None:
        axes[1].axhline(+abort_pend_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
        axes[1].axhline(-abort_pend_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
    axes[1].set_ylabel("α (deg)")
    alpha_ylim = float(ylim_pend_deg) if ylim_pend_deg is not None else float(lim_pend_deg + 5.0)
    axes[1].set_ylim(-alpha_ylim, +alpha_ylim)
    axes[1].grid(True, alpha=0.25)

    if include_effort:
        effort_ylim = float(effort_ylim_steps) if effort_ylim_steps is not None else float(max_acc_steps)
        axes[2].plot(t, df["acc_cmd_steps_s2"], linewidth=0.9, color="purple")
        axes[2].axhline(0, color="k", linewidth=0.7, alpha=0.4)
        axes[2].axhline(+max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axes[2].axhline(-max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axes[2].set_ylabel("u (steps/s²)")
        axes[2].set_ylim(-effort_ylim, +effort_ylim)
        axes[2].grid(True, alpha=0.25)
        axes[2].set_xlabel("time (s)")
    else:
        axes[1].set_xlabel("time (s)")

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_mode_comparison_timeseries(
    items: Sequence[dict[str, Any]],
    title: str,
    lim_pend_deg: float = 30.0,
    lim_motor_deg: float = 80.0,
    ylim_pend_deg: Optional[float] = None,
    ylim_motor_deg: Optional[float] = None,
    soft_motor_deg: Optional[float] = 78.0,
    abort_pend_deg: Optional[float] = None,
    effort_ylim_steps: Optional[float] = None,
):
    import matplotlib.pyplot as plt

    if not items:
        raise ValueError("plot_mode_comparison_timeseries requires at least one item")

    nrows = len(items)
    effort_ylim = float(effort_ylim_steps) if effort_ylim_steps is not None else max(
        float(item.get("max_acc_steps", 20000.0)) for item in items
    )

    fig, axes = plt.subplots(nrows, 3, figsize=(14, 2.7 * nrows + 1.0), squeeze=False, sharey="col")
    alpha_ylim = float(ylim_pend_deg) if ylim_pend_deg is not None else float(lim_pend_deg + 5.0)
    theta_ylim = float(ylim_motor_deg) if ylim_motor_deg is not None else float(lim_motor_deg + 5.0)

    col_titles = ["α (deg)", "θ (deg)", "u (steps/s²)"]
    for col, name in enumerate(col_titles):
        axes[0, col].set_title(name)

    for row, item in enumerate(items):
        df = item["df"]
        t = df["time_s"].to_numpy(dtype=float)
        mode = str(item.get("mode", ""))
        label = str(item.get("label", mode))
        color = mode_color(mode)
        max_acc_steps = float(item.get("max_acc_steps", 20000.0))

        axa, axt, axu = axes[row]

        axa.plot(t, df["alpha_deg"], linewidth=0.9, color=color)
        axa.axhline(0, color="k", linewidth=0.7, alpha=0.4)
        axa.axhline(+lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axa.axhline(-lim_pend_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        if abort_pend_deg is not None:
            axa.axhline(+abort_pend_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
            axa.axhline(-abort_pend_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
        axa.set_ylim(-alpha_ylim, +alpha_ylim)
        axa.grid(True, alpha=0.25)
        axa.text(0.02, 0.88, label, transform=axa.transAxes, fontsize=10, fontweight="bold")

        axt.plot(t, df["theta_deg"], linewidth=0.9, color=color)
        axt.axhline(0, color="k", linewidth=0.7, alpha=0.4)
        if soft_motor_deg is not None:
            axt.axhline(+soft_motor_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
            axt.axhline(-soft_motor_deg, color="tab:orange", linestyle="--", linewidth=0.9, alpha=0.7)
        axt.axhline(+lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axt.axhline(-lim_motor_deg, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axt.set_ylim(-theta_ylim, +theta_ylim)
        axt.grid(True, alpha=0.25)

        axu.plot(t, df["acc_cmd_steps_s2"], linewidth=0.9, color="purple")
        axu.axhline(0, color="k", linewidth=0.7, alpha=0.4)
        axu.axhline(+max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axu.axhline(-max_acc_steps, color="r", linestyle=":", linewidth=0.9, alpha=0.6)
        axu.set_ylim(-effort_ylim, +effort_ylim)
        axu.grid(True, alpha=0.25)

    for col in range(3):
        axes[-1, col].set_xlabel("time (s)")

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_metric_bars(
    summary_df: pd.DataFrame,
    title: str,
    metrics: Optional[Sequence[tuple[str, str]]] = None,
):
    import matplotlib.pyplot as plt

    if summary_df.empty:
        raise ValueError("plot_metric_bars requires non-empty summary data")

    if metrics is None:
        metrics = (
            ("alpha_rms_deg_median", "α RMS (deg)"),
            ("alpha_max_abs_deg_median", "max |α| (deg)"),
            ("theta_max_abs_deg_median", "max |θ| (deg)"),
            ("acc_rms_steps_s2_median", "u RMS (steps/s²)"),
        )

    order = {"LIN": 0, "SMC": 1, "SMC4": 2}
    df = summary_df.copy()
    df["__order"] = df["mode"].astype(str).map(lambda m: order.get(m.upper(), 99))
    df = df.sort_values("__order")

    modes = df["mode"].astype(str).tolist()
    colors = [mode_color(m) for m in modes]
    x = np.arange(len(modes))

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.2))
    axes_flat = list(axes.flat)

    for ax, (metric, ylabel) in zip(axes_flat, metrics, strict=False):
        vals = df[metric].to_numpy(dtype=float)
        bars = ax.bar(x, vals, color=colors, alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels(modes)
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.25)
        ymax = max(float(np.nanmax(vals)) * 1.18, 1.0)
        ax.set_ylim(0.0, ymax)
        for bar, val in zip(bars, vals, strict=False):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height() + 0.02 * ymax,
                f"{val:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

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
