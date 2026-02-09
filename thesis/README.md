# Thesis (LaTeX) — Rotary/Furuta Inverted Pendulum

This folder is the **thesis/report source**. It is designed to build locally in VS Code (LaTeX Workshop) with `latexmk`, `biber`, and all build artifacts written to `thesis/build/`.

## One-time prerequisites (BasicTeX)

- `latexmk` and `biber` must be on PATH (you already have these).
- This project is set up to **compile even if `biblatex` is missing**, but it will use a minimal fallback bibliography.
  For real IEEE-style references via `biblatex`+`biber`, install:
  - `tlmgr install biblatex biblatex-ieee`

## Standard workflow (thesis assets come from analysis)

1) Generate the analysis report (tables + figures) from logs:

```bash
./.venv/bin/python tools/analyze_experiments.py \
  --manifest experiments/manifest_thesis_20260209.json \
  --out reports/thesis/report_thesis_20260209
```

2) Export curated assets into `thesis/` (figures/tables + metadata + modelling conversion):

```bash
./.venv/bin/python tools/export_thesis_assets.py \
  --report reports/thesis/report_thesis_20260209 \
  --manifest experiments/manifest_thesis_20260209.json \
  --out thesis
```

3) Build the PDF:

```bash
cd thesis
latexmk
```

Output:
- PDF: `thesis/build/main.pdf`

## VS Code preview (Overleaf-like)

With the LaTeX Workshop extension you can preview side-by-side:

1) Open `thesis/main.tex`
2) Run **Command Palette → “LaTeX Workshop: Build LaTeX project”**
3) Run **Command Palette → “LaTeX Workshop: View LaTeX PDF file”**

Tip: use **“View LaTeX PDF file in VS Code tab”** so the PDF can sit side-by-side with the `.tex` source.
SyncTeX (click in PDF ↔ jump to source) works when you build via `latexmk` with `-synctex=1` (already enabled).

## Notes

- The thesis **always compiles** even if figures/tables are missing (it will show boxed placeholders).
- The full modelling derivation is included in the main body by converting `tools/modelling_complete.md` into:
  - `thesis/chapters/03_modelling_derivation_generated.tex`
  Do not edit the generated file by hand; edit `tools/modelling_complete.md` instead and re-run the export.
