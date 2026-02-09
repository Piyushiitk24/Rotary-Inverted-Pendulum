#!/usr/bin/env python3

from __future__ import annotations

import argparse
import datetime as dt
import shutil
import subprocess
import sys
from pathlib import Path

import pandas as pd


REQUIRED_REPORT_CSVS = [
    "metrics_trials.csv",
    "metrics_summary_by_experiment_mode.csv",
    "metrics_nudge_steps.csv",
    "alignment_quality.csv",
]

MODE_DISPLAY = {
    "LIN": "Linear",
    "SMC": "Hybrid SMC",
    "SMC4": "Full-surface SMC",
}

EXPERIMENT_DISPLAY = {
    "hold": "Hold",
    "nudge": "Nudge",
    "tap": "Tap",
}


def warn_if_not_venv(repo_root: Path) -> None:
    exe = Path(sys.executable).resolve()
    venv_python = (repo_root / ".venv" / "bin" / "python").resolve()
    if exe != venv_python:
        print(f"[WARN] Not running from repo venv: {exe}")
        print(f"       Expected: {venv_python}")


def get_git_hash(repo_root: Path) -> str:
    try:
        out = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo_root, text=True).strip()
        return out
    except Exception:
        return "unknown"


def pick_figure_source(fig_dir: Path, experiment: str, mode: str, plot: str, prefer: str) -> Path | None:
    exts = ["pdf", "png"] if prefer == "pdf" else ["png", "pdf"]
    for ext in exts:
        matches = sorted(fig_dir.glob(f"{experiment}_{mode}_rep_*_trial0_{plot}.{ext}"))
        if matches:
            return matches[0]
    return None


def export_figures(report_fig_dir: Path, thesis_fig_dir: Path, prefer: str) -> list[str]:
    # Curated exports (minimum set).
    spec = [
        # Hold
        ("hold", "LIN", ["timeseries", "phase_alpha", "effort_hist", "alpha_spectrum"]),
        ("hold", "SMC", ["timeseries", "phase_alpha", "effort_hist", "alpha_spectrum"]),
        ("hold", "SMC4", ["timeseries", "phase_alpha", "effort_hist", "alpha_spectrum", "sparse_s", "sparse_den", "sparse_kEff", "sparse_kRamp"]),
        # Nudge (linear only)
        ("nudge", "LIN", ["nudge", "timeseries", "phase_alpha", "effort_hist", "alpha_spectrum"]),
        # Tap
        ("tap", "LIN", ["timeseries", "phase_alpha", "effort_hist", "alpha_spectrum"]),
        ("tap", "SMC", ["timeseries", "phase_alpha", "effort_hist", "alpha_spectrum"]),
        ("tap", "SMC4", ["timeseries", "phase_alpha", "effort_hist", "alpha_spectrum", "sparse_s", "sparse_den", "sparse_kEff", "sparse_kRamp"]),
    ]

    thesis_fig_dir.mkdir(parents=True, exist_ok=True)
    warnings: list[str] = []

    for experiment, mode, plots in spec:
        for plot in plots:
            src = pick_figure_source(report_fig_dir, experiment, mode, plot, prefer)
            stem = f"{experiment}_{mode}_{plot}"
            if src is None:
                warnings.append(f"Missing figure: {stem} (no rep_*_trial0_{plot}.pdf/png)")
                continue
            dst = thesis_fig_dir / f"{stem}{src.suffix}"
            shutil.copy2(src, dst)

    return warnings


def df_to_longtable(df: pd.DataFrame, caption: str, label: str) -> str:
    # Minimal longtable with booktabs.
    cols = list(df.columns)
    # Left align non-numeric, right align numeric.
    col_spec_parts: list[str] = []
    for c in cols:
        col_spec_parts.append("r" if pd.api.types.is_numeric_dtype(df[c]) else "l")
    col_spec = "".join(col_spec_parts)

    lines: list[str] = []
    lines.append("\\begin{longtable}{" + col_spec + "}")
    lines.append("\\caption{" + caption + "}\\\\")
    lines.append("\\label{" + label + "}\\\\")
    lines.append("\\toprule")
    lines.append(" & ".join(cols) + " \\\\")
    lines.append("\\midrule")
    lines.append("\\endfirsthead")
    lines.append("\\toprule")
    lines.append(" & ".join(cols) + " \\\\")
    lines.append("\\midrule")
    lines.append("\\endhead")
    lines.append("\\bottomrule")
    lines.append("\\endfoot")

    for _, row in df.iterrows():
        vals = []
        for c in cols:
            v = row[c]
            if pd.isna(v):
                vals.append("--")
            else:
                s = str(v)
                # Basic LaTeX escaping for common CSV strings.
                s = s.replace("_", "\\_")
                vals.append(s)
        lines.append(" & ".join(vals) + " \\\\")

    lines.append("\\end{longtable}")
    return "\n".join(lines) + "\n"


def df_to_tabular(df: pd.DataFrame, caption: str, label: str, *, resize: bool = True) -> str:
    cols = list(df.columns)
    col_spec_parts: list[str] = []
    for c in cols:
        col_spec_parts.append("r" if pd.api.types.is_numeric_dtype(df[c]) else "l")
    col_spec = "".join(col_spec_parts)
    lines: list[str] = []
    lines.append("\\begin{table}[htbp]")
    lines.append("\\centering")
    lines.append("\\small")
    lines.append("\\setlength{\\tabcolsep}{3pt}")
    lines.append("\\renewcommand{\\arraystretch}{0.95}")
    lines.append("\\caption{" + caption + "}")
    lines.append("\\label{" + label + "}")
    if resize:
        lines.append("\\resizebox{\\textwidth}{!}{%")
    lines.append("\\begin{tabular}{" + col_spec + "}")
    lines.append("\\toprule")
    lines.append(" & ".join(cols) + " \\\\")
    lines.append("\\midrule")
    for _, row in df.iterrows():
        vals = []
        for c in cols:
            v = row[c]
            if pd.isna(v):
                vals.append("--")
            else:
                s = str(v)
                s = s.replace("_", "\\_")
                vals.append(s)
        lines.append(" & ".join(vals) + " \\\\")
    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    if resize:
        lines.append("}%")
    lines.append("\\end{table}")
    return "\n".join(lines) + "\n"


def export_tables(report_dir: Path, thesis_tables_dir: Path) -> list[str]:
    thesis_tables_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = thesis_tables_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)

    warnings: list[str] = []

    # Copy raw CSVs.
    for name in REQUIRED_REPORT_CSVS:
        src = report_dir / name
        if not src.exists():
            warnings.append(f"Missing report CSV: {name}")
            continue
        shutil.copy2(src, raw_dir / name)

    # Generate LaTeX tables.
    def trial_code(experiment: str, mode: str) -> str:
        if experiment == "hold":
            return {"LIN": "H1", "SMC": "H2", "SMC4": "H3"}.get(mode, f"H?_{mode}")
        if experiment == "nudge":
            return {"LIN": "N1"}.get(mode, f"N?_{mode}")
        if experiment == "tap":
            return {"LIN": "T1", "SMC": "T2", "SMC4": "T3"}.get(mode, f"T?_{mode}")
        return f"?_{mode}"

    trials = pd.read_csv(report_dir / "metrics_trials.csv")
    trials_tbl = trials[
        [
            "mode",
            "experiment",
            "outcome",
            "duration_s",
            "alpha_rms_deg",
            "alpha_max_abs_deg",
            "theta_max_abs_deg",
            "theta_drift_slope_deg_s",
            "acc_rms_steps_s2",
        ]
    ].copy()
    trials_tbl.insert(
        0,
        "Trial",
        [trial_code(e, m) for e, m in zip(trials_tbl["experiment"], trials_tbl["mode"], strict=False)],
    )
    trials_tbl["mode"] = trials_tbl["mode"].map(MODE_DISPLAY).fillna(trials_tbl["mode"])
    trials_tbl["experiment"] = trials_tbl["experiment"].map(EXPERIMENT_DISPLAY).fillna(trials_tbl["experiment"])
    trials_tbl = trials_tbl.rename(
        columns={
            "mode": "Mode",
            "experiment": "Type",
            "outcome": "Outcome",
            "duration_s": r"Duration (\si{s})",
            "alpha_rms_deg": r"$\alpha$ RMS (\si{\degree})",
            "alpha_max_abs_deg": r"max $|\alpha|$ (\si{\degree})",
            "theta_max_abs_deg": r"max $|\theta|$ (\si{\degree})",
            "theta_drift_slope_deg_s": r"$\theta$ drift (\si{\degree\per\second})",
            "acc_rms_steps_s2": r"acc RMS (steps/s$^2$)",
        }
    )
    trials_tex = df_to_tabular(
        trials_tbl.round(3),
        caption="Trials overview (thesis-facing identifiers H/N/T).",
        label="tab:trials_overview",
        resize=True,
    )
    (thesis_tables_dir / "tab_trials_overview.tex").write_text(trials_tex, encoding="utf-8")

    summ = pd.read_csv(report_dir / "metrics_summary_by_experiment_mode.csv")
    hold = summ[summ["experiment"] == "hold"].copy()
    tap = summ[summ["experiment"] == "tap"].copy()
    hold["mode"] = hold["mode"].map(MODE_DISPLAY).fillna(hold["mode"])
    tap["mode"] = tap["mode"].map(MODE_DISPLAY).fillna(tap["mode"])

    # Keep a small, defensible set of columns for main-text tables.
    summary_cols = [
        "mode",
        "duration_s_median",
        "alpha_rms_deg_median",
        "alpha_max_abs_deg_median",
        "theta_max_abs_deg_median",
        "theta_drift_slope_deg_s_median",
        "acc_rms_steps_s2_median",
        "acc_max_abs_steps_s2_median",
    ]

    summary_renames = {
        "mode": "Mode",
        "duration_s_median": r"Duration (\si{s})",
        "alpha_rms_deg_median": r"$\alpha$ RMS (\si{\degree})",
        "alpha_max_abs_deg_median": r"max $|\alpha|$ (\si{\degree})",
        "theta_max_abs_deg_median": r"max $|\theta|$ (\si{\degree})",
        "theta_drift_slope_deg_s_median": r"$\theta$ drift (\si{\degree\per\second})",
        "acc_rms_steps_s2_median": r"acc RMS (steps/s$^2$)",
        "acc_max_abs_steps_s2_median": r"max $|acc|$ (steps/s$^2$)",
    }

    hold_tex = df_to_tabular(
        hold[summary_cols].rename(columns=summary_renames).round(3),
        caption="Summary metrics by controller mode (hold experiments).",
        label="tab:summary_hold",
    )
    (thesis_tables_dir / "tab_summary_hold.tex").write_text(hold_tex, encoding="utf-8")

    tap_tex = df_to_tabular(
        tap[summary_cols].rename(columns=summary_renames).round(3),
        caption="Summary metrics by controller mode (tap experiments).",
        label="tab:summary_tap",
    )
    (thesis_tables_dir / "tab_summary_tap.tex").write_text(tap_tex, encoding="utf-8")

    nudge = pd.read_csv(report_dir / "metrics_nudge_steps.csv")
    if "step_index" in nudge.columns:
        # Avoid 0.0 / 1.0 formatting in the PDF.
        nudge["step_index"] = nudge["step_index"].astype("Int64")
    nudge_cols = [
        "step_index",
        "target_deg",
        "rise_time_s",
        "overshoot_deg",
        "settle_time_s",
        "theta_ss_err_deg",
        "alpha_max_abs_deg",
    ]
    nudge_tex = df_to_tabular(
        nudge[nudge_cols]
        .rename(
            columns={
                "step_index": "Step",
                "target_deg": r"Target (\si{\degree})",
                "rise_time_s": "Rise (s)",
                "overshoot_deg": r"Overshoot (\si{\degree})",
                "settle_time_s": "Settle (s)",
                "theta_ss_err_deg": r"$\theta$ ss err (\si{\degree})",
                "alpha_max_abs_deg": r"max $|\alpha|$ (\si{\degree})",
            }
        )
        .round(3),
        caption="Nudge/reference tracking step metrics (linear controller).",
        label="tab:nudge_steps",
    )
    (thesis_tables_dir / "tab_nudge_steps.tex").write_text(nudge_tex, encoding="utf-8")

    return warnings


def export_dataset_table(manifest_path: Path, thesis_tables_dir: Path) -> None:
    """Write a short dataset table mapping thesis-facing IDs to timestamp-coded run IDs."""
    try:
        import json

        data = json.loads(manifest_path.read_text(encoding="utf-8"))
        sessions = data.get("sessions", [])
    except Exception:
        return

    rows = []
    for s in sessions:
        label = str(s.get("label", ""))
        path = str(s.get("path", ""))
        experiment = str(s.get("experiment", ""))
        mode = "LIN"
        if "_SMC4_" in label:
            mode = "SMC4"
        elif "_SMC_" in label:
            mode = "SMC"
        elif "_LIN_" in label:
            mode = "LIN"

        # Thesis-facing IDs.
        if experiment == "hold":
            trial = {"LIN": "H1", "SMC": "H2", "SMC4": "H3"}.get(mode, "H?")
        elif experiment == "nudge":
            trial = "N1"
        elif experiment == "tap":
            trial = {"LIN": "T1", "SMC": "T2", "SMC4": "T3"}.get(mode, "T?")
        else:
            trial = "?"

        # Extract a short run ID from the manifest path.
        # Expected: logs/balance/session_YYYYMMDD_HHMMSS (but we handle other forms).
        run_dir = path.rstrip("/").split("/")[-1] if path else ""
        run_id = run_dir.replace("session_", "")
        rows.append((trial, mode, experiment, run_id))

    if not rows:
        return

    # Deterministic order.
    order = {"H1": 0, "H2": 1, "H3": 2, "N1": 3, "T1": 4, "T2": 5, "T3": 6}
    rows.sort(key=lambda r: order.get(r[0], 99))

    lines: list[str] = []
    lines.append("\\begin{table}[htbp]")
    lines.append("\\centering")
    lines.append("\\small")
    lines.append("\\setlength{\\tabcolsep}{4pt}")
    lines.append("\\caption{Dataset used for this report (manifest sessions).}")
    lines.append("\\label{tab:dataset_manifest}")
    lines.append("\\begin{tabular}{llll}")
    lines.append("\\toprule")
    lines.append("ID & Controller & Type & Run ID \\\\")
    lines.append("\\midrule")
    for trial, mode, exp, run_id in rows:
        exp_name = EXPERIMENT_DISPLAY.get(exp, exp)
        mode_name = MODE_DISPLAY.get(mode, mode)
        run_id_esc = str(run_id).replace("_", "\\_")
        lines.append(f"{trial} & {mode_name} & {exp_name} & {run_id_esc} \\\\")
    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}")
    (thesis_tables_dir / "tab_dataset_manifest.tex").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_metadata(thesis_dir: Path, manifest: str | None, report_dir: Path, repo_root: Path) -> None:
    ts = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    git_hash = get_git_hash(repo_root)
    manifest_str = Path(manifest).name if manifest else "(not provided)"
    report_str = report_dir.name

    contents = (
        "% Auto-generated by tools/export_thesis_assets.py\n"
        f"\\def\\ThesisManifest{{\\detokenize{{{manifest_str}}}}}\n"
        f"\\def\\ThesisAnalysisReport{{\\detokenize{{{report_str}}}}}\n"
        f"\\def\\ThesisAnalysisVersion{{\\detokenize{{{git_hash} {ts}}}}}\n"
        "\\def\\ThesisFirmwareVersion{\\detokenize{unknown}}\n"
    )
    (thesis_dir / "metadata.tex").write_text(contents, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Export analysis assets into thesis/ (figures, tables, metadata).")
    parser.add_argument("--report", required=True, help="Analysis report directory (reports/thesis/...)")
    parser.add_argument("--manifest", default=None, help="Manifest path used to create the report (optional)")
    parser.add_argument("--out", default="thesis", help="Thesis directory (default: thesis)")
    parser.add_argument("--prefer", choices=["pdf", "png"], default="pdf", help="Prefer PDF or PNG for figures")
    parser.add_argument(
        "--write-modelling",
        dest="write_modelling",
        action="store_true",
        default=True,
        help="Generate modelling chapter tex (default: on)",
    )
    parser.add_argument(
        "--no-write-modelling",
        dest="write_modelling",
        action="store_false",
        help="Disable modelling chapter generation",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    warn_if_not_venv(repo_root)

    report_dir = Path(args.report).resolve()
    thesis_dir = (repo_root / args.out).resolve() if not Path(args.out).is_absolute() else Path(args.out).resolve()

    report_fig_dir = report_dir / "figures"
    if not report_fig_dir.exists():
        print(f"[ERR] Missing report figures dir: {report_fig_dir}", file=sys.stderr)
        return 1

    # Ensure thesis structure exists.
    (thesis_dir / "figures").mkdir(parents=True, exist_ok=True)
    (thesis_dir / "tables").mkdir(parents=True, exist_ok=True)

    fig_warnings = export_figures(report_fig_dir, thesis_dir / "figures", args.prefer)
    table_warnings = export_tables(report_dir, thesis_dir / "tables")
    write_metadata(thesis_dir, args.manifest, report_dir, repo_root)
    if args.manifest:
        mpath = (repo_root / args.manifest) if not Path(args.manifest).is_absolute() else Path(args.manifest)
        if mpath.exists():
            export_dataset_table(mpath, thesis_dir / "tables")

    if args.write_modelling:
        try:
            subprocess.check_call(
                [
                    sys.executable,
                    str(repo_root / "tools" / "build_modelling_chapter.py"),
                    "--in",
                    "tools/modelling_complete.md",
                    "--out",
                    str(thesis_dir / "chapters" / "03_modelling_derivation_generated.tex"),
                ],
                cwd=repo_root,
            )
        except Exception as exc:
            print(f"[WARN] Failed to generate modelling chapter: {exc}")

    for w in fig_warnings + table_warnings:
        print(f"[WARN] {w}")

    print(f"[DONE] Exported thesis assets into: {thesis_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
