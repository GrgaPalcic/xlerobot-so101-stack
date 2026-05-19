#!/usr/bin/env python3
"""Generate the small fixed workspace fiducial target for XLeRobot calibration.

Requires OpenCV with aruco support and Pillow:

    python3 -m pip install opencv-contrib-python-headless pillow
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


DPI = 600
MM_PER_INCH = 25.4


def mm_to_px(mm: float) -> int:
    return int(round(mm * DPI / MM_PER_INCH))


def font(size_px: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        if Path(path).exists():
            return ImageFont.truetype(path, size_px)
    return ImageFont.load_default()


def make_marker(dictionary, marker_id: int, size_px: int) -> Image.Image:
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    else:
        marker = np.zeros((size_px, size_px), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, size_px, marker, 1)
    return Image.fromarray(marker, mode="L").convert("RGB")


def draw_centered_text(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, font_obj) -> None:
    bbox = draw.textbbox((0, 0), text, font=font_obj)
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.text((xy[0] - width // 2, xy[1] - height // 2), text, fill=(0, 0, 0), font=font_obj)


def generate(args: argparse.Namespace) -> list[Path]:
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    pattern_w_mm = args.cols * args.square_mm
    pattern_h_mm = args.rows * args.square_mm
    pattern_w_px = mm_to_px(pattern_w_mm)
    pattern_h_px = mm_to_px(pattern_h_mm)
    square_px = mm_to_px(args.square_mm)
    marker_px = mm_to_px(args.marker_mm)
    inset_px = (square_px - marker_px) // 2

    pattern = Image.new("RGB", (pattern_w_px, pattern_h_px), "white")
    draw = ImageDraw.Draw(pattern)

    grid_color = (150, 150, 150)
    for row in range(args.rows):
        for col in range(args.cols):
            x0 = col * square_px
            y0 = row * square_px
            x1 = x0 + square_px
            y1 = y0 + square_px
            if (row + col) % 2 == 0:
                draw.rectangle((x0, y0, x1, y1), fill=(245, 245, 245))
            draw.rectangle((x0, y0, x1, y1), outline=grid_color, width=max(1, mm_to_px(0.10)))

    marker_id = args.start_id
    used_ids: list[int] = []
    for row in range(args.rows):
        marker_cols = range(0, args.cols, 2) if row % 2 == 0 else range(1, args.cols, 2)
        for col in marker_cols:
            x = col * square_px + inset_px
            y = row * square_px + inset_px
            marker = make_marker(dictionary, marker_id, marker_px)
            pattern.paste(marker, (x, y))
            used_ids.append(marker_id)
            marker_id += 1

    border = max(2, mm_to_px(0.35))
    draw.rectangle((0, 0, pattern_w_px - 1, pattern_h_px - 1), outline=(0, 0, 0), width=border)

    base = f"xlerobot_world_target_{args.cols}x{args.rows}_{args.square_mm:g}mm_aruco5x5_100_id{args.start_id}"
    pattern_png = out_dir / f"{base}_pattern.png"
    pattern.save(pattern_png, dpi=(DPI, DPI))

    page_w_px = mm_to_px(args.page_width_mm)
    page_h_px = mm_to_px(args.page_height_mm)
    page = Image.new("RGB", (page_w_px, page_h_px), "white")
    page_draw = ImageDraw.Draw(page)
    title_font = font(mm_to_px(4.0))
    body_font = font(mm_to_px(3.0))
    small_font = font(mm_to_px(2.4))

    x = (page_w_px - pattern_w_px) // 2
    y = mm_to_px(28.0)
    page.paste(pattern, (x, y))

    draw_centered_text(
        page_draw,
        (page_w_px // 2, mm_to_px(13.0)),
        "XLeRobot world fiducial target - print at 100%, no scaling",
        title_font,
    )
    draw_centered_text(
        page_draw,
        (page_w_px // 2, mm_to_px(20.0)),
        (
            f"DICT_5X5_100 ids {used_ids[0]}-{used_ids[-1]} | "
            f"{args.cols}x{args.rows} squares | square {args.square_mm:g} mm | marker {args.marker_mm:g} mm"
        ),
        body_font,
    )

    label_y = y + pattern_h_px + mm_to_px(8.0)
    draw_centered_text(
        page_draw,
        (page_w_px // 2, label_y),
        f"Outer pattern size: {pattern_w_mm:.1f} mm x {pattern_h_mm:.1f} mm. Touch/check the outer black rectangle corners.",
        body_font,
    )
    draw_centered_text(
        page_draw,
        (page_w_px // 2, label_y + mm_to_px(6.0)),
        "Mount this print to a rigid flat plate. Do not laminate with uneven bubbles.",
        small_font,
    )

    ruler_x = x
    ruler_y = label_y + mm_to_px(16.0)
    ruler_len = mm_to_px(100.0)
    page_draw.line((ruler_x, ruler_y, ruler_x + ruler_len, ruler_y), fill=(0, 0, 0), width=max(2, mm_to_px(0.25)))
    for tick_mm in range(0, 101, 10):
        tx = ruler_x + mm_to_px(float(tick_mm))
        tick_h = mm_to_px(4.0 if tick_mm % 50 == 0 else 2.5)
        page_draw.line((tx, ruler_y - tick_h, tx, ruler_y + tick_h), fill=(0, 0, 0), width=max(1, mm_to_px(0.18)))
    draw_centered_text(page_draw, (ruler_x + ruler_len // 2, ruler_y + mm_to_px(8.0)), "100 mm print verification ruler", small_font)

    page_png = out_dir / f"{base}_a4.png"
    page_pdf = out_dir / f"{base}_a4.pdf"
    page.save(page_png, dpi=(DPI, DPI))
    page.save(page_pdf, "PDF", resolution=DPI)

    readme = out_dir / "README_xlerobot_world_target.md"
    readme.write_text(
        "\n".join(
            [
                "# XLeRobot World Fiducial Target",
                "",
                f"- dictionary: DICT_5X5_100",
                f"- marker ids: {used_ids[0]}-{used_ids[-1]}",
                f"- squares: {args.cols} x {args.rows}",
                f"- square size: {args.square_mm:g} mm",
                f"- marker size: {args.marker_mm:g} mm",
                f"- outer pattern size: {pattern_w_mm:.1f} mm x {pattern_h_mm:.1f} mm",
                "",
                "Print the PDF at 100% / actual size. Disable fit-to-page and borderless scaling.",
                "After printing, verify the ruler is exactly 100 mm and the outer pattern is exactly 140 mm x 100 mm.",
                "Mount the print to a rigid flat plate before calibration.",
                "",
                "Use these runbook constants:",
                "",
                "```bash",
                f"export WORLD_COLS={args.cols}",
                f"export WORLD_ROWS={args.rows}",
                f"export WORLD_SQUARE_M={args.square_mm / 1000.0:.6f}",
                f"export WORLD_MARKER_M={args.marker_mm / 1000.0:.6f}",
                f"export WORLD_START_ID={args.start_id}",
                f"export WORLD_MARKER_COUNT={len(used_ids)}",
                "export WORLD_DICT=DICT_5X5_100",
                "```",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return [pattern_png, page_png, page_pdf, readme]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="docs/assets/fiducials")
    parser.add_argument("--cols", type=int, default=7)
    parser.add_argument("--rows", type=int, default=5)
    parser.add_argument("--square-mm", type=float, default=20.0)
    parser.add_argument("--marker-mm", type=float, default=14.0)
    parser.add_argument("--start-id", type=int, default=50)
    parser.add_argument("--page-width-mm", type=float, default=210.0)
    parser.add_argument("--page-height-mm", type=float, default=297.0)
    return parser.parse_args()


def main() -> int:
    for path in generate(parse_args()):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

