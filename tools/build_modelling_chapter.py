#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _strip_yaml_front_matter(md: str) -> str:
    # Only treat YAML front matter if it starts at the beginning of the file.
    if not md.startswith("---"):
        return md
    parts = md.split("---", 2)
    if len(parts) < 3:
        return md
    return parts[2]


_HEADING_RE = re.compile(r"^(#{1,6})\s+(.*)$")
_SECTION9_RE = re.compile(r"^##\s+9\.\s+")


def _extract_modelling_only(md: str) -> str:
    """Keep only modelling derivation (sections 1--8).

    tools/modelling_complete.md contains controllers and implementation notes
    starting at section 9. For the thesis structure, we keep modelling in the
    modelling chapter and cover controllers/pipeline in their own chapters.
    """
    out_lines: list[str] = []
    for line in md.splitlines():
        if _SECTION9_RE.match(line):
            break
        out_lines.append(line)
    return "\n".join(out_lines) + "\n"


def _strip_numeric_prefix_from_headings(md: str) -> str:
    out_lines: list[str] = []
    for line in md.splitlines():
        m = _HEADING_RE.match(line)
        if not m:
            out_lines.append(line)
            continue
        hashes, title = m.group(1), m.group(2).strip()
        title = re.sub(r"^\d+(?:\.\d+)*\.?\s+", "", title)
        out_lines.append(f"{hashes} {title}")
    return "\n".join(out_lines) + "\n"


def _pandoc_available() -> bool:
    return shutil.which("pandoc") is not None


def _convert_with_pandoc(md_text: str) -> str:
    with tempfile.TemporaryDirectory() as tmpdir:
        in_path = Path(tmpdir) / "modelling.md"
        in_path.write_text(md_text, encoding="utf-8")

        # We want:
        # - YAML removed already
        # - headings shifted so '##' becomes \section
        # - top-level division set to section (no \chapter inside fragment)
        cmd = [
            "pandoc",
            "--from",
            "markdown",
            "--to",
            "latex",
            "--wrap=none",
            "--top-level-division=section",
            "--shift-heading-level-by=-1",
            "--syntax-highlighting=none",
            str(in_path),
        ]
        try:
            return subprocess.check_output(cmd, text=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"pandoc conversion failed: {e}") from e


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert tools/modelling_complete.md to LaTeX chapter fragment.")
    parser.add_argument("--in", dest="in_path", required=True, help="Input markdown path (tools/modelling_complete.md)")
    parser.add_argument("--out", dest="out_path", required=True, help="Output LaTeX path (thesis/chapters/...)")
    args = parser.parse_args()

    in_path = Path(args.in_path)
    out_path = Path(args.out_path)

    md = in_path.read_text(encoding="utf-8")
    md = _strip_yaml_front_matter(md)
    md = _extract_modelling_only(md)
    md = _strip_numeric_prefix_from_headings(md)

    if not _pandoc_available():
        raise RuntimeError("pandoc is not available on PATH; install it or adjust the converter.")

    tex = _convert_with_pandoc(md)

    header = (
        "% AUTO-GENERATED FILE — DO NOT EDIT.\n"
        f"% Generated from {in_path.as_posix()} by tools/build_modelling_chapter.py\n\n"
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(header + tex, encoding="utf-8")

    print(f"[DONE] Wrote modelling chapter: {out_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[ERR] {exc}", file=sys.stderr)
        raise SystemExit(1)
