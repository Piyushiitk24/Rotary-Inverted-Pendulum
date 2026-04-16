from __future__ import annotations

from typing import Any


def text(
    x: float,
    y: float,
    w: float,
    h: float,
    value: str,
    *,
    size: int = 11,
    color: str = "3D5472",
    bold: bool = False,
    italic: bool = False,
    align: str = "left",
    valign: str = "top",
) -> dict[str, Any]:
    return {
        "type": "text",
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "text": value,
        "size": size,
        "color": color,
        "bold": bold,
        "italic": italic,
        "align": align,
        "valign": valign,
    }


def bullets(
    x: float,
    y: float,
    w: float,
    h: float,
    items: list[str],
    *,
    size: int = 10,
    color: str = "3D5472",
    gap: float = 0.18,
) -> dict[str, Any]:
    return {
        "type": "bullets",
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "items": items,
        "size": size,
        "color": color,
        "gap": gap,
    }


def card(
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    title: str | None = None,
    body: str | None = None,
    bullets_list: list[str] | None = None,
    stripe: str = "0D6EAF",
    fill: str = "FFFFFF",
    title_color: str = "1B2A4A",
    body_color: str = "3D5472",
) -> dict[str, Any]:
    return {
        "type": "card",
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "title": title,
        "body": body,
        "bullets": bullets_list,
        "stripe": stripe,
        "fill": fill,
        "title_color": title_color,
        "body_color": body_color,
    }


def image_slot(
    asset_id: str,
    src: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    fit: str = "contain",
) -> dict[str, Any]:
    return {
        "type": "image",
        "asset_id": asset_id,
        "src": src,
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "fit": fit,
    }


def equation_slot(
    eq_id: str,
    latex: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    font_size: int = 18,
) -> dict[str, Any]:
    return {
        "type": "equation",
        "eq_id": eq_id,
        "latex": latex,
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "font_size": font_size,
    }


def flow(
    x: float,
    y: float,
    w: float,
    h: float,
    nodes: list[dict[str, Any]],
    *,
    arrow_color: str = "0D6EAF",
) -> dict[str, Any]:
    return {
        "type": "flow",
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "nodes": nodes,
        "arrow_color": arrow_color,
    }


def table(
    x: float,
    y: float,
    w: float,
    h: float,
    headers: list[str],
    rows: list[list[str]],
    *,
    fill_header: str = "D6E8F5",
    fill_body: str = "FFFFFF",
) -> dict[str, Any]:
    return {
        "type": "table",
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "headers": headers,
        "rows": rows,
        "fill_header": fill_header,
        "fill_body": fill_body,
    }


def slide(
    number: int,
    kind: str,
    title: str,
    subtitle: str,
    *,
    elements: list[dict[str, Any]] | None = None,
    key_contribution: bool = False,
) -> dict[str, Any]:
    return {
        "number": number,
        "kind": kind,
        "title": title,
        "subtitle": subtitle,
        "elements": elements or [],
        "key_contribution": key_contribution,
    }
