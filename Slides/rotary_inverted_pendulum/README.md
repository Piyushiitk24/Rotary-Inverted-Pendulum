# Rotary Inverted Pendulum Thesis Slides

This workspace keeps all rotary inverted pendulum slide content and build assets inside `Slides/rotary_inverted_pendulum/`.

## Structure

- `scripts/content_manifest.py`: single structured content source for all 38 slides
- `scripts/render_equations.py`: renders LaTeX equations to transparent PNGs with `matplotlib` + `usetex`
- `scripts/build_deck.js`: builds the slide chrome, text, tables, and native diagrams with `pptxgenjs`
- `scripts/postprocess_deck.py`: inserts rendered equations and copied PNG figures with `python-pptx`
- `assets/`: copied source figures plus generated equations
- `dist/`: generated `.pptx` output and intermediate manifest JSON

## Build

```bash
cd Slides/rotary_inverted_pendulum
../../.venv/bin/python scripts/build.py
```

The build script sets a writable `MPLCONFIGDIR`, copies the required PNG sources into `assets/figures/`, renders equations into `assets/equations/`, builds the base deck, and writes the final deck into `dist/`.
