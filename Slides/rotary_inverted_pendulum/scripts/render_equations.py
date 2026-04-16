from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path.cwd() / ".mpl-cache"))

import matplotlib

matplotlib.use("pdf")
from matplotlib import pyplot as plt
from matplotlib import rcParams
from PIL import Image


def configure_matplotlib() -> None:
    rcParams.update(
        {
            "text.usetex": True,
            "font.family": "serif",
            "font.serif": ["Computer Modern Roman"],
            "figure.dpi": 200,
            "text.latex.preamble": r"\usepackage{amsmath}\usepackage{amssymb}\usepackage{bm}",
        }
    )


def load_manifest(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def render_equation(latex: str, out_path: Path, font_size: int) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir_path = Path(tmpdir)
        pdf_path = tmpdir_path / f"{out_path.stem}.pdf"
        fig = plt.figure(figsize=(0.01, 0.01))
        fig.patch.set_alpha(0.0)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_axis_off()
        ax.patch.set_alpha(0.0)
        fig.text(
            0.0,
            0.0,
            f"${latex}$",
            fontsize=font_size,
            color="black",
        )
        fig.savefig(pdf_path, transparent=True, bbox_inches="tight", pad_inches=0.01)
        plt.close(fig)
        subprocess.run(
            ["sips", "-s", "format", "png", str(pdf_path), "--out", str(out_path)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    with Image.open(out_path) as img:
        alpha = img.getchannel("A")
        bbox = alpha.getbbox()
        if bbox:
            img.crop(bbox).save(out_path)


def main() -> int:
    if len(sys.argv) != 3:
      print("Usage: python render_equations.py <manifest.json> <workspace_root>", file=sys.stderr)
      return 2
    manifest_path = Path(sys.argv[1]).resolve()
    workspace_root = Path(sys.argv[2]).resolve()
    manifest = load_manifest(manifest_path)
    configure_matplotlib()

    for slide in manifest["slides"]:
        for element in slide.get("elements", []):
            if element.get("type") != "equation":
                continue
            out_path = workspace_root / "assets" / "equations" / f"{element['eq_id']}.png"
            render_equation(element["latex"], out_path, int(element.get("font_size", 18)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
