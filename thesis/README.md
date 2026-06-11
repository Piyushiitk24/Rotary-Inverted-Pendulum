# IIT Kanpur M.Tech Thesis

This directory is the canonical LaTeX source for:

**Modelling, Control, and Experimental Evaluation of a Stepper-Driven Rotary Inverted Pendulum**

The document uses the supplied IIT Kanpur February 2026 `Thesis` class with
A4 paper, two-sided printing, open-right chapters, IITK margins, and the IITK
front-matter sequence.

## Source Ownership

- `metadata.tex`: author, roll number, degree, department, supervisors, title,
  and submission date.
- `main.tex`: IITK front matter, seven research chapters, appendices, and
  bibliography.
- `chapters/`: thesis narrative.
- `appendices/modelling_derivation_generated.tex`: generated from
  `tools/modelling_complete.md`; do not edit directly.
- `analysis_metadata.tex`, `figures/`, and `tables/`: generated or refreshed by
  `tools/export_thesis_assets.py`.
- `Thesis.cls`, `title.tex`, `images/iitk_logo.png`, and `legacy_packages/`:
  IITK template assets with minimal compatibility repairs.

## Rebuild the Experimental Report

From the repository root:

```bash
./.venv/bin/python tools/analyze_experiments.py \
  --manifest experiments/manifest_thesis_20260209.json \
  --out reports/thesis/report_thesis_20260209
```

The analysis report must contain:

- `metrics_trials.csv`
- `metrics_summary_by_experiment_mode.csv`
- `metrics_nudge_steps.csv`
- `alignment_quality.csv`
- `figures/`

## Export Thesis Assets

```bash
./.venv/bin/python tools/export_thesis_assets.py \
  --report reports/thesis/report_thesis_20260209 \
  --manifest experiments/manifest_thesis_20260209.json \
  --out thesis
```

This refreshes the curated plots and tables, analysis metadata, and generated
modelling appendix.

## Build the Thesis

```bash
cd thesis
latexmk
```

Output:

```text
thesis/build/main.pdf
```

The local `.latexmkrc` runs pdfLaTeX and BibTeX, writes build products to
`thesis/build/`, and exposes the template's local legacy package directory.

For a clean validation build:

```bash
cd thesis
latexmk -C
latexmk
```

## Submission Checks

Before submission:

1. Confirm all identity and supervisor details in `metadata.tex`.
2. Regenerate the report and thesis assets from the pinned manifest.
3. Confirm there are no undefined references, missing citations, missing
   figures, sample template text, or unresolved hardware entries.
4. Inspect title, certificate, declaration, abstract, lists, chapter openings,
   appendices, and references in `build/main.pdf`.
5. Keep claims tied to the seven pinned trials; the dataset is qualitative and
   does not support statistical controller rankings.
