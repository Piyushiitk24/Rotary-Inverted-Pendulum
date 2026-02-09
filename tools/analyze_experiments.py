#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import numpy as np
import pandas as pd

from balance_analysis import (
    compute_nudge_step_metrics,
    compute_trial_metrics,
    extract_trials,
    fit_host_to_device_time,
    load_balance_csv,
    match_status_to_csv,
    parse_events,
)
from balance_analysis.types import Alignment, Events, Trial
from balance_analysis.plots import (
    plot_effort_hist,
    plot_lowfreq_spectrum,
    plot_nudge_tracking,
    plot_phase_portrait,
    plot_sparse_series,
    plot_timeseries,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _warn_if_not_venv() -> None:
    root = _repo_root()
    venv_dir = root / ".venv"
    if not venv_dir.exists():
        return
    # sys.executable can be a symlink to system Python; sys.prefix reliably points at the venv root.
    prefix = Path(getattr(sys, "prefix", "")).resolve()
    in_venv = prefix == venv_dir.resolve()
    if not in_venv:
        recommended = root / ".venv" / "bin" / "python"
        print(f"[WARN] Not running from repo venv. Recommended: {recommended} tools/analyze_experiments.py", file=sys.stderr)


def _set_matplotlib_cache_dir() -> None:
    # Avoid slow imports / warnings if ~/.matplotlib is not writable.
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib"))


def _parse_manifest(path: Path) -> dict[str, Any]:
    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text())
    else:
        try:
            import yaml  # type: ignore
        except Exception as e:
            raise RuntimeError(
                "YAML manifest requires PyYAML. Either install it "
                "(`./.venv/bin/pip install -r tools/requirements_thesis.txt`) "
                "or use a JSON manifest (`.json`)."
            ) from e

        data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError("manifest must be a mapping object (YAML/JSON)")
    return data


def _collect_sessions(args) -> tuple[list[Path], dict[str, Any], dict[str, Any]]:
    manifest_data: dict[str, Any] = {}
    manifest_meta: dict[str, Any] = {}

    if args.manifest:
        manifest_path = Path(args.manifest)
        manifest_data = _parse_manifest(manifest_path)
        sessions_raw = manifest_data.get("sessions")
        if not isinstance(sessions_raw, list):
            raise ValueError("manifest: expected top-level 'sessions: [ ... ]'")
        session_dirs: list[Path] = []
        for item in sessions_raw:
            if isinstance(item, str):
                p = Path(item)
                session_dirs.append(p)
                manifest_meta[str(p)] = {}
            elif isinstance(item, dict) and "path" in item:
                p = Path(item["path"])
                session_dirs.append(p)
                manifest_meta[str(p)] = item
            else:
                raise ValueError(f"manifest sessions item must be string or mapping with path: {item!r}")
        return session_dirs, manifest_data, manifest_meta

    session_dirs: list[Path] = []
    if args.sessions:
        session_dirs.extend(Path(s) for s in args.sessions)
    if args.glob:
        session_dirs.extend(sorted(Path(".").glob(args.glob)))

    # Normalize to directories that contain balance_log.csv
    norm: list[Path] = []
    for p in session_dirs:
        if p.is_dir() and (p / "balance_log.csv").exists():
            norm.append(p)
        elif p.is_dir() and p.name.startswith("session_") and (Path("logs/balance") / p.name / "balance_log.csv").exists():
            norm.append(Path("logs/balance") / p.name)
        elif p.is_file() and p.name == "balance_log.csv":
            norm.append(p.parent)
    # Deduplicate
    seen = set()
    out = []
    for p in norm:
        rp = str(p.resolve())
        if rp in seen:
            continue
        seen.add(rp)
        out.append(p)
    return out, manifest_data, manifest_meta


def _mode_rank(mode: str) -> int:
    return {"LIN": 0, "SMC": 1, "SMC4": 2}.get(mode, 99)


def _score_representative(row: pd.Series) -> float:
    # Higher score is better.
    if row.get("outcome") != "success":
        return -1e9
    dur = float(row.get("duration_s", 0.0))
    alpha_rms = float(row.get("alpha_rms_deg", 999.0))
    drift = abs(float(row.get("theta_drift_slope_deg_s", 999.0)))
    sat = float(row.get("sat_pct", 100.0))
    clamp = float(row.get("clamped_pct", 100.0)) if not pd.isna(row.get("clamped_pct")) else 0.0
    return 5.0 * dur - 50.0 * alpha_rms - 200.0 * drift - 10.0 * sat - 2.0 * clamp


def _extract_theta_target_steps(events: Events) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    last_val: Optional[float] = None
    for snap in events.params:
        if "thetaTargetDeg" not in snap.values:
            continue
        # Firmware prints setpoint updates as a single-token line: "thetaTargetDeg=XX.XX"
        if not snap.raw_line.strip().startswith("thetaTargetDeg="):
            continue
        if " " in snap.raw_line.strip():
            continue
        try:
            v = float(snap.values["thetaTargetDeg"])
        except Exception:
            continue
        if last_val is None or abs(v - last_val) > 1e-6:
            steps.append({"host_s": snap.host_s, "target_deg": v, "raw": snap.raw_line})
            last_val = v
    return steps


def _detect_motion_episodes_fallback(df: pd.DataFrame, moving_thresh_deg_s: float = 5.0, min_quiet_s: float = 1.5) -> list[float]:
    """
    Fallback for nudge step timing when host↔device alignment is unreliable:
    detect motion episodes from |theta_dot| rising after a quiet period.
    """
    t = df["time_s"].to_numpy(dtype=float)
    v = np.abs(df["theta_dot_deg_s"].to_numpy(dtype=float))
    moving = v > moving_thresh_deg_s
    starts: list[float] = []
    last_start_idx = None

    # Require a quiet window before a new episode.
    quiet = ~moving
    for i in range(1, len(t)):
        if moving[i] and not moving[i - 1]:
            # Look back for quiet duration.
            t_start = t[i]
            t_back = t_start - min_quiet_s
            j0 = np.searchsorted(t, t_back, side="left")
            if np.all(quiet[j0:i]):
                if last_start_idx is None or (i - last_start_idx) > 5:
                    starts.append(float(t_start))
                    last_start_idx = i
    return starts


def _trial_host_window(trial: Trial) -> tuple[Optional[float], Optional[float]]:
    if not trial.engaged_event:
        return None, None
    t0 = trial.engaged_event.host_s
    t1 = trial.end_event.host_s if trial.end_event else None
    return t0, t1


def _summarize_dbg_within(trial: Trial, events: Events) -> dict[str, Any]:
    t0, t1 = _trial_host_window(trial)
    if t0 is None:
        return {}
    dbg = [d for d in events.dbg if d.host_s >= t0 and (t1 is None or d.host_s <= t1)]
    if not dbg:
        return {}
    dt_max = [float(d.fields.get("dtUsMax", np.nan)) for d in dbg]
    over = [float(d.fields.get("over", 0.0)) for d in dbg]
    sat = [float(d.fields.get("sat", 0.0)) for d in dbg]
    stallE = [float(d.fields.get("stallE", 0.0)) for d in dbg]
    return {
        "dbg_dtUsMax_max": float(np.nanmax(dt_max)),
        "dbg_dtUsMax_med": float(np.nanmedian(dt_max)),
        "dbg_over_max": float(np.nanmax(over)),
        "dbg_sat_max": float(np.nanmax(sat)),
        "dbg_stallE_max": float(np.nanmax(stallE)),
        "dbg_samples": int(len(dbg)),
    }


def _summarize_move_within(trial: Trial, events: Events) -> dict[str, Any]:
    t0, t1 = _trial_host_window(trial)
    if t0 is None:
        return {}
    mv = [m for m in events.move if m.host_s >= t0 and (t1 is None or m.host_s <= t1)]
    if not mv:
        return {"move_paused_count": 0, "move_resumed_count": 0, "move_paused_total_s": 0.0}

    paused_total = 0.0
    paused_at: Optional[float] = None
    paused_count = 0
    resumed_count = 0
    for m in mv:
        if m.kind == "PAUSED":
            paused_count += 1
            if paused_at is None:
                paused_at = m.host_s
        elif m.kind == "RESUMED":
            resumed_count += 1
            if paused_at is not None:
                paused_total += max(0.0, m.host_s - paused_at)
                paused_at = None
    if paused_at is not None and t1 is not None:
        paused_total += max(0.0, t1 - paused_at)

    return {"move_paused_count": paused_count, "move_resumed_count": resumed_count, "move_paused_total_s": float(paused_total)}


def _ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def main() -> int:
    _warn_if_not_venv()
    _set_matplotlib_cache_dir()

    ap = argparse.ArgumentParser(description="Batch analysis for Furuta/RIP balance sessions (thesis-ready outputs).")
    ap.add_argument("--glob", help="Glob pattern for session dirs (e.g. 'logs/balance/session_20260205_*').")
    ap.add_argument("--sessions", nargs="*", help="Explicit session directories (logs/balance/session_...).")
    ap.add_argument(
        "--manifest",
        help="Optional manifest file (.json or .yaml). YAML requires PyYAML; JSON works without extra deps.",
    )
    ap.add_argument("--out", help="Output directory. Default: reports/thesis/<timestamp>/")
    ap.add_argument("--gap-ms", type=int, default=500, help="CSV timestamp gap (ms) used only in fallback segmentation.")
    ap.add_argument("--no-spectrum", action="store_true", help="Disable low-frequency spectrum plots.")
    args = ap.parse_args()

    session_dirs, manifest_data, manifest_meta = _collect_sessions(args)
    if not session_dirs:
        print("No sessions found. Use --glob or --sessions or --manifest.", file=sys.stderr)
        return 2

    out_dir = Path(args.out) if args.out else (Path("reports/thesis") / datetime.now().strftime("%Y%m%d_%H%M%S"))
    figs_dir = out_dir / "figures"
    per_session_dir = out_dir / "per_session"
    _ensure_dir(figs_dir)
    _ensure_dir(per_session_dir)

    alignment_rows: list[dict[str, Any]] = []
    trials_rows: list[dict[str, Any]] = []
    nudge_rows: list[dict[str, Any]] = []
    manifest_rows: list[dict[str, Any]] = []

    # Process each session
    for session_dir in session_dirs:
        session_dir = Path(session_dir)
        session_name = session_dir.name
        print(f"[INFO] Processing {session_name} ...")

        m = manifest_meta.get(str(session_dir), {})
        manifest_label = m.get("label", "") if isinstance(m, dict) else ""
        manifest_experiment = m.get("experiment", "") if isinstance(m, dict) else ""
        manifest_notes = m.get("notes", "") if isinstance(m, dict) else ""

        try:
            df = load_balance_csv(session_dir)
        except Exception as e:
            print(f"[WARN] Skipping {session_name}: CSV load failed: {e}", file=sys.stderr)
            continue

        try:
            ev = parse_events(session_dir)
        except Exception as e:
            print(f"[WARN] Skipping {session_name}: events parse failed: {e}", file=sys.stderr)
            continue

        matches = match_status_to_csv(ev.status, df)
        alignment = fit_host_to_device_time(matches)

        alignment_rows.append(
            {
                "session": str(session_dir),
                "n_status": len(ev.status),
                "n_matches": alignment.n_matches,
                "align_reliable": int(alignment.reliable),
                "median_abs_residual_ms": alignment.median_abs_residual_ms,
                "max_abs_residual_ms": alignment.max_abs_residual_ms,
            }
        )

        trials = extract_trials(df, ev, alignment=alignment if alignment.reliable else None, gap_ms=args.gap_ms)

        # Trial diagnostics per session
        diag_path = per_session_dir / session_name / "trial_diagnostics.csv"
        _ensure_dir(diag_path.parent)
        diag_df = pd.DataFrame(
            [
                {
                    "session": str(session_dir),
                    "trial_index": t.trial_index,
                    "mode": t.mode,
                    "start_ms": t.start_ms,
                    "end_ms": t.end_ms,
                    "rows": int(len(t.df)) if t.df is not None else 0,
                    "align_used": int(bool(t.diagnostics.get("align_used"))),
                    **{k: v for k, v in t.diagnostics.items() if k not in {"align_used"}},
                }
                for t in trials
            ]
        )
        diag_df.to_csv(diag_path, index=False)

        # Extract nudge target changes (session-wide)
        session_steps = _extract_theta_target_steps(ev)

        for t in trials:
            # Attach dbg/move summaries into diagnostics so compute_trial_metrics can include them.
            t.diagnostics.update(_summarize_dbg_within(t, ev))
            t.diagnostics.update(_summarize_move_within(t, ev))
            row = compute_trial_metrics(t)
            if manifest_label:
                row["label"] = manifest_label
            if manifest_experiment:
                row["experiment"] = manifest_experiment
            if manifest_notes:
                row["notes"] = manifest_notes
            # include a few diagnostics columns
            for k in ["dbg_dtUsMax_max", "dbg_over_max", "dbg_sat_max", "dbg_stallE_max", "move_paused_total_s", "move_paused_count"]:
                if k in t.diagnostics:
                    row[k] = t.diagnostics[k]
            trials_rows.append(row)

            # Nudge step metrics: map thetaTargetDeg changes into trial time if possible.
            steps_for_trial: list[dict[str, Any]] = []
            if session_steps:
                if alignment.reliable:
                    for s in session_steps:
                        dev_ms = alignment.host_s_to_device_ms(float(s["host_s"]))
                        if dev_ms < t.start_ms or dev_ms > t.end_ms:
                            continue
                        steps_for_trial.append(
                            {
                                "t_cmd_s": float(dev_ms - t.start_ms) / 1000.0,
                                "target_deg": float(s["target_deg"]),
                                "raw": s["raw"],
                            }
                        )
                    steps_for_trial.sort(key=lambda x: x["t_cmd_s"])
                    for i in range(len(steps_for_trial) - 1):
                        steps_for_trial[i]["t_next_cmd_s"] = steps_for_trial[i + 1]["t_cmd_s"]
                else:
                    # Fallback: infer motion episodes from theta_dot and pair with target sequence by order.
                    targets = [float(s["target_deg"]) for s in session_steps]
                    episode_starts = _detect_motion_episodes_fallback(t.df)
                    if len(episode_starts) == len(targets):
                        for i, (ts, tgt) in enumerate(zip(episode_starts, targets)):
                            steps_for_trial.append({"t_cmd_s": float(ts), "target_deg": float(tgt)})
                        for i in range(len(steps_for_trial) - 1):
                            steps_for_trial[i]["t_next_cmd_s"] = steps_for_trial[i + 1]["t_cmd_s"]

            if steps_for_trial:
                rows = compute_nudge_step_metrics(t, steps_for_trial)
                for r in rows:
                    if manifest_label:
                        r["label"] = manifest_label
                    if manifest_experiment:
                        r["experiment"] = manifest_experiment
                    if manifest_notes:
                        r["notes"] = manifest_notes
                nudge_rows.extend(rows)

        manifest_rows.append(
            {
                "session": str(session_dir),
                "events_start_dt": ev.start_dt.isoformat() if ev.start_dt else "",
                "manifest_label": manifest_label,
                "manifest_experiment": manifest_experiment,
                "manifest_notes": manifest_notes,
            }
        )

    # Write outputs
    _ensure_dir(out_dir)
    trials_df = pd.DataFrame(trials_rows)
    trials_csv = out_dir / "metrics_trials.csv"
    trials_df.to_csv(trials_csv, index=False)

    if not trials_df.empty:
        summary_cols = [
            "duration_s",
            "alpha_rms_deg",
            "alpha_max_abs_deg",
            "theta_drift_slope_deg_s",
            "theta_max_abs_deg",
            "acc_rms_steps_s2",
            "acc_max_abs_steps_s2",
            "sat_pct",
            "clamped_pct",
        ]
        existing = [c for c in summary_cols if c in trials_df.columns]
        summary = trials_df.groupby("mode")[existing].agg(["mean", "median", "std", "count"])
        # Flatten MultiIndex columns for a thesis-friendly CSV.
        summary.columns = [f"{metric}_{stat}" for metric, stat in summary.columns]
        summary = summary.reset_index()
        summary.to_csv(out_dir / "metrics_summary_by_mode.csv", index=False)

        # Optional: summary by (experiment, mode) when a manifest provides experiment labels.
        if "experiment" in trials_df.columns and trials_df["experiment"].notna().any():
            sub = trials_df[trials_df["experiment"].astype(str) != ""].copy()
            if not sub.empty:
                summary2 = sub.groupby(["experiment", "mode"])[existing].agg(["mean", "median", "std", "count"])
                summary2.columns = [f"{metric}_{stat}" for metric, stat in summary2.columns]
                summary2 = summary2.reset_index()
                summary2.to_csv(out_dir / "metrics_summary_by_experiment_mode.csv", index=False)

    if nudge_rows:
        pd.DataFrame(nudge_rows).to_csv(out_dir / "metrics_nudge_steps.csv", index=False)

    pd.DataFrame(alignment_rows).to_csv(out_dir / "alignment_quality.csv", index=False)
    pd.DataFrame(manifest_rows).to_csv(out_dir / "manifest_resolved.csv", index=False)

    # Representative selection (either per mode, or per (experiment, mode) when a manifest provides experiments).
    if not trials_df.empty:
        reps: list[tuple[str, str, pd.Series]] = []  # (experiment, mode, row)

        # Optional: pinned representatives in manifest
        pinned = manifest_data.get("representatives") if isinstance(manifest_data, dict) else None
        pinned_map: dict[str, dict[str, Any]] = {}
        if isinstance(pinned, dict):
            for mode, spec in pinned.items():
                if isinstance(spec, str):
                    pinned_map[str(mode).upper()] = {"session": spec}
                elif isinstance(spec, dict):
                    pinned_map[str(mode).upper()] = spec

        use_experiment = "experiment" in trials_df.columns and trials_df["experiment"].notna().any()

        if use_experiment:
            exp_vals = sorted({str(x) for x in trials_df["experiment"].dropna().tolist() if str(x) != ""})
            for exp in exp_vals:
                df_exp = trials_df[trials_df["experiment"].astype(str) == exp]
                for mode in sorted(df_exp["mode"].unique(), key=_mode_rank):
                    mode_u = str(mode).upper()
                    sub = df_exp[df_exp["mode"] == mode].copy()
                    if sub.empty:
                        continue

                    # Pinned representatives are mode-global; apply only when they match this experiment too.
                    if mode_u in pinned_map:
                        spec = pinned_map[mode_u]
                        sess = spec.get("session")
                        tix = spec.get("trial_index")
                        if sess is not None:
                            sub2 = sub[sub["session"] == str(sess)]
                            if tix is not None:
                                sub2 = sub2[sub2["trial_index"] == int(tix)]
                            if not sub2.empty:
                                reps.append((exp, str(mode), sub2.iloc[0]))
                                continue

                    sub["rep_score"] = sub.apply(_score_representative, axis=1)
                    best = sub.sort_values("rep_score", ascending=False).iloc[0]
                    reps.append((exp, str(mode), best))
        else:
            for mode in sorted(trials_df["mode"].unique(), key=_mode_rank):
                mode_u = str(mode).upper()
                sub = trials_df[trials_df["mode"] == mode].copy()
                if sub.empty:
                    continue

                # Use pinned representative if provided and present in table.
                if mode_u in pinned_map:
                    spec = pinned_map[mode_u]
                    sess = spec.get("session")
                    tix = spec.get("trial_index")
                    if sess is not None:
                        sub2 = sub[sub["session"] == str(sess)]
                        if tix is not None:
                            sub2 = sub2[sub2["trial_index"] == int(tix)]
                        if not sub2.empty:
                            reps.append(("", str(mode), sub2.iloc[0]))
                            continue

                sub["rep_score"] = sub.apply(_score_representative, axis=1)
                best = sub.sort_values("rep_score", ascending=False).iloc[0]
                reps.append(("", str(mode), best))

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        for exp, mode, best in reps:
            session_dir = Path(best["session"])
            trial_index = int(best["trial_index"])
            df = load_balance_csv(session_dir)
            ev = parse_events(session_dir)
            matches = match_status_to_csv(ev.status, df)
            alignment = fit_host_to_device_time(matches)
            trials = extract_trials(df, ev, alignment=alignment if alignment.reliable else None, gap_ms=args.gap_ms)
            trial = next((t for t in trials if int(t.trial_index) == trial_index), None)
            if trial is None or trial.df is None or len(trial.df) == 0:
                continue

            prefix = f"{exp}_" if exp else ""
            base_name = f"{prefix}{mode}_rep_{session_dir.name}_trial{trial_index}"
            title_prefix = f"{exp} / " if exp else ""
            fig1 = plot_timeseries(
                trial.df,
                title=f"{title_prefix}{mode} representative: {session_dir.name} (trial {trial_index})",
            )
            fig1.savefig(figs_dir / f"{base_name}_timeseries.png", dpi=160)
            fig1.savefig(figs_dir / f"{base_name}_timeseries.pdf")
            plt.close(fig1)

            fig2 = plot_phase_portrait(trial.df, title=f"{title_prefix}{mode} α vs α̇")
            fig2.savefig(figs_dir / f"{base_name}_phase_alpha.png", dpi=160)
            fig2.savefig(figs_dir / f"{base_name}_phase_alpha.pdf")
            plt.close(fig2)

            fig3 = plot_effort_hist(trial.df, title=f"{title_prefix}{mode} effort histogram")
            fig3.savefig(figs_dir / f"{base_name}_effort_hist.png", dpi=160)
            fig3.savefig(figs_dir / f"{base_name}_effort_hist.pdf")
            plt.close(fig3)

            # Sparse SMC diagnostics from printed [STATUS] fields (safe; no reconstruction).
            # Only plot when the firmware printed the field (e.g., s=...).
            pts = [m for m in matches if trial.start_ms <= m.csv_timestamp_ms <= trial.end_ms]
            if pts:
                def _plot_sparse_key(key: str, ylabel: str):
                    xs = []
                    ys = []
                    for m in pts:
                        if key not in m.status.extra:
                            continue
                        try:
                            yv = float(m.status.extra[key])
                        except Exception:
                            continue
                        xs.append((m.csv_timestamp_ms - trial.start_ms) / 1000.0)
                        ys.append(yv)
                    if len(xs) >= 3:
                        fig = plot_sparse_series(
                            np.asarray(xs, dtype=float),
                            np.asarray(ys, dtype=float),
                            title=f"{title_prefix}{mode} sparse {key}(t) from [STATUS]",
                            ylabel=ylabel,
                        )
                        fig.savefig(figs_dir / f"{base_name}_sparse_{key}.png", dpi=160)
                        fig.savefig(figs_dir / f"{base_name}_sparse_{key}.pdf")
                        plt.close(fig)

                _plot_sparse_key("s", "s (deg/s)")
                _plot_sparse_key("den", "den (a.u.)")
                _plot_sparse_key("kEff", "kEff")
                _plot_sparse_key("kRamp", "kRamp")

            if not args.no_spectrum:
                try:
                    fig4 = plot_lowfreq_spectrum(trial.df, title=f"{title_prefix}{mode} α spectrum", col="alpha_deg")
                    fig4.savefig(figs_dir / f"{base_name}_alpha_spectrum.png", dpi=160)
                    fig4.savefig(figs_dir / f"{base_name}_alpha_spectrum.pdf")
                    plt.close(fig4)
                except Exception:
                    pass

            # Nudge plot (if step metrics exist for this trial)
            if nudge_rows:
                steps = [r for r in nudge_rows if r["session"] == str(session_dir) and int(r["trial_index"]) == trial_index]
                # Rebuild step list for plotting (command schedule)
                if steps:
                    # Use t_cmd_s / target_deg from the step table (already in trial time).
                    plot_steps = [{"t_cmd_s": float(s["t_cmd_s"]), "target_deg": float(s["target_deg"]), "t_next_cmd_s": float(s.get("t_end_s", trial.df["time_s"].iloc[-1]))} for s in steps]
                    fig5 = plot_nudge_tracking(trial.df, title=f"{title_prefix}{mode} nudge tracking", steps=plot_steps)
                    fig5.savefig(figs_dir / f"{base_name}_nudge.png", dpi=160)
                    fig5.savefig(figs_dir / f"{base_name}_nudge.pdf")
                    plt.close(fig5)

    # README
    readme = out_dir / "README.md"
    readme.write_text(
        "\n".join(
            [
                "# Thesis Analysis Report",
                "",
                "Generated by `tools/analyze_experiments.py`.",
                "",
                "## Outputs",
                "- `metrics_trials.csv`: per-trial metrics",
                "- `metrics_summary_by_mode.csv`: aggregated metrics by controller mode",
                "- `metrics_summary_by_experiment_mode.csv`: aggregated metrics by (experiment, mode) when a manifest provides experiment labels",
                "- `metrics_nudge_steps.csv`: per-step metrics for nudge experiments (when detected)",
                "- `alignment_quality.csv`: host↔device time alignment quality per session",
                "- `figures/`: representative-run figures per mode",
                "- `per_session/<session>/trial_diagnostics.csv`: extraction diagnostics",
                "",
                "## Notes",
                "- FFT/spectrum plots are low-frequency only (≤25 Hz) due to ~50 Hz logging.",
                "- Event annotations use robust `[STATUS]` matching. If alignment is unreliable, time-precise annotations are skipped.",
                "",
            ]
        )
        + "\n"
    )

    print(f"[DONE] Wrote report to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
