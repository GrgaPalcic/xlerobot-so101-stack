#!/usr/bin/env python3
"""Generate rigid workspace ChArUco fiducial targets for XLeRobot calibration.

Requires OpenCV with aruco support and Pillow:

    python3 -m pip install opencv-contrib-python-headless pillow
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


DPI = 600
MM_PER_INCH = 25.4


@dataclass(frozen=True)
class Target:
    label: str
    start_id: int
    ids: list[int]
    pattern_png: Path


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


def draw_centered_text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    font_obj: ImageFont.FreeTypeFont | ImageFont.ImageFont,
) -> None:
    bbox = draw.textbbox((0, 0), text, font=font_obj)
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.text((xy[0] - width // 2, xy[1] - height // 2), text, fill=(0, 0, 0), font=font_obj)


def board_marker_count(cols: int, rows: int) -> int:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((cols, rows), 1.0, 0.7, dictionary)
    return int(len(board.getIds()))


def make_charuco_pattern(
    *,
    cols: int,
    rows: int,
    square_mm: float,
    marker_mm: float,
    start_id: int,
    output: Path,
) -> Target:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    marker_count = board_marker_count(cols, rows)
    ids = np.arange(start_id, start_id + marker_count, dtype=np.int32)
    if ids[-1] >= 100:
        raise ValueError(f"DICT_5X5_100 only has ids 0-99, got id {int(ids[-1])}")

    board = cv2.aruco.CharucoBoard(
        (cols, rows),
        square_mm / 1000.0,
        marker_mm / 1000.0,
        dictionary,
        ids,
    )
    pattern_w_px = mm_to_px(cols * square_mm)
    pattern_h_px = mm_to_px(rows * square_mm)
    image = board.generateImage((pattern_w_px, pattern_h_px), marginSize=0, borderBits=1)
    pattern = Image.fromarray(image, mode="L").convert("RGB")
    pattern.save(output, dpi=(DPI, DPI))

    return Target(
        label="",
        start_id=start_id,
        ids=[int(value) for value in ids],
        pattern_png=output,
    )


def paginate_targets(
    *,
    targets: list[Target],
    cols: int,
    rows: int,
    square_mm: float,
    marker_mm: float,
    page_width_mm: float,
    page_height_mm: float,
    output_dir: Path,
    base: str,
) -> list[Path]:
    page_w_px = mm_to_px(page_width_mm)
    page_h_px = mm_to_px(page_height_mm)
    pattern_w_px = mm_to_px(cols * square_mm)
    pattern_h_px = mm_to_px(rows * square_mm)
    title_font = font(mm_to_px(3.0))
    label_font = font(mm_to_px(2.8))

    pages: list[Image.Image] = []
    page_pngs: list[Path] = []

    slots_per_page = 2
    for page_index in range((len(targets) + slots_per_page - 1) // slots_per_page):
        page = Image.new("RGB", (page_w_px, page_h_px), "white")
        draw = ImageDraw.Draw(page)
        draw_centered_text(
            draw,
            (page_w_px // 2, mm_to_px(10.0)),
            "XLeRobot workspace ChArUco plates - print at 100%, no scaling",
            title_font,
        )

        page_targets = targets[page_index * slots_per_page : (page_index + 1) * slots_per_page]
        y_positions = [mm_to_px(24.0), mm_to_px(156.0)]
        if len(page_targets) == 1:
            y_positions = [mm_to_px(70.0)]

        for target, y in zip(page_targets, y_positions):
            x = (page_w_px - pattern_w_px) // 2
            pattern = Image.open(target.pattern_png).convert("RGB")
            page.paste(pattern, (x, y))

            target_label = target.label or f"Plate {chr(ord('A') + targets.index(target))}"
            marker_range = f"ids {target.ids[0]}-{target.ids[-1]}"
            draw_centered_text(
                draw,
                (page_w_px // 2, y - mm_to_px(5.0)),
                (
                    f"{target_label} | DICT_5X5_100 | {marker_range} | "
                    f"{cols}x{rows} | square {square_mm:g} mm | marker {marker_mm:g} mm"
                ),
                label_font,
            )

        page_png = output_dir / f"{base}_page{page_index + 1}.png"
        page.save(page_png, dpi=(DPI, DPI))
        page_pngs.append(page_png)
        pages.append(page)

    pdf = output_dir / f"{base}_a4.pdf"
    pages[0].save(pdf, "PDF", resolution=DPI, save_all=True, append_images=pages[1:])
    return page_pngs + [pdf]


def generate(args: argparse.Namespace) -> list[Path]:
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    start_ids = [int(value) for value in args.start_ids.split(",") if value.strip()]
    if not start_ids:
        raise ValueError("--start-ids must contain at least one marker id")

    pattern_w_mm = args.cols * args.square_mm
    pattern_h_mm = args.rows * args.square_mm
    marker_count = board_marker_count(args.cols, args.rows)
    base = (
        f"xlerobot_world_targets_{args.cols}x{args.rows}_{args.square_mm:g}mm_"
        f"aruco5x5_100_ids{start_ids[0]}-{start_ids[-1] + marker_count - 1}"
    )

    targets: list[Target] = []
    outputs: list[Path] = []
    for index, start_id in enumerate(start_ids):
        label = f"Plate {chr(ord('A') + index)}"
        pattern_png = out_dir / (
            f"xlerobot_world_target_{label[-1]}_{args.cols}x{args.rows}_"
            f"{args.square_mm:g}mm_aruco5x5_100_ids{start_id}-{start_id + marker_count - 1}_pattern.png"
        )
        target = make_charuco_pattern(
            cols=args.cols,
            rows=args.rows,
            square_mm=args.square_mm,
            marker_mm=args.marker_mm,
            start_id=start_id,
            output=pattern_png,
        )
        targets.append(
            Target(
                label=label,
                start_id=target.start_id,
                ids=target.ids,
                pattern_png=target.pattern_png,
            )
        )
        outputs.append(pattern_png)

    outputs.extend(
        paginate_targets(
            targets=targets,
            cols=args.cols,
            rows=args.rows,
            square_mm=args.square_mm,
            marker_mm=args.marker_mm,
            page_width_mm=args.page_width_mm,
            page_height_mm=args.page_height_mm,
            output_dir=out_dir,
            base=base,
        )
    )

    readme = out_dir / "README_xlerobot_world_target.md"
    readme.write_text(
        "\n".join(
            [
                "# XLeRobot World Fiducial Targets",
                "",
                "These are small rigid workspace ChArUco targets for the dual-arm calibration run.",
                "",
                f"- dictionary: DICT_5X5_100",
                f"- plates: {', '.join(target.label for target in targets)}",
                f"- marker ids per plate: {', '.join(f'{target.label}={target.ids[0]}-{target.ids[-1]}' for target in targets)}",
                f"- squares: {args.cols} x {args.rows}",
                f"- ChArUco corners: {(args.cols - 1) * (args.rows - 1)}",
                f"- ArUco markers per plate: {marker_count}",
                f"- square size: {args.square_mm:g} mm",
                f"- marker size: {args.marker_mm:g} mm",
                f"- outer pattern size: {pattern_w_mm:.1f} mm x {pattern_h_mm:.1f} mm",
                "",
                "Print the PDF at 100% / actual size. Disable fit-to-page and borderless scaling.",
                "Measure the printed ChArUco squares with calipers. Each square should be 20.0 mm.",
                "Mount each selected print to a rigid flat plate before calibration.",
                "",
                "Use Plate A by default. If the print quality, mounting, glare, or detection is poor,",
                "switch to Plate B or Plate C and change only WORLD_START_ID.",
                "",
                "Print file:",
                "",
                f"```text",
                f"{base}_a4.pdf",
                f"```",
                "",
                "Use these runbook constants for Plate A:",
                "",
                "```bash",
                f"export WORLD_COLS={args.cols}",
                f"export WORLD_ROWS={args.rows}",
                f"export WORLD_SQUARE_M={args.square_mm / 1000.0:.6f}",
                f"export WORLD_MARKER_M={args.marker_mm / 1000.0:.6f}",
                f"export WORLD_START_ID={targets[0].start_id}",
                f"export WORLD_MARKER_COUNT={marker_count}",
                "export WORLD_DICT=DICT_5X5_100",
                "```",
                "",
                "Alternative start IDs:",
                "",
                "```text",
                *[f"{target.label}: WORLD_START_ID={target.start_id}" for target in targets],
                "```",
                "",
            ]
        ),
        encoding="utf-8",
    )
    outputs.append(readme)
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="docs/assets/fiducials")
    parser.add_argument("--cols", type=int, default=7)
    parser.add_argument("--rows", type=int, default=5)
    parser.add_argument("--square-mm", type=float, default=20.0)
    parser.add_argument("--marker-mm", type=float, default=14.0)
    parser.add_argument("--start-ids", default="49,66,83")
    parser.add_argument("--page-width-mm", type=float, default=210.0)
    parser.add_argument("--page-height-mm", type=float, default=297.0)
    return parser.parse_args()


def main() -> int:
    for path in generate(parse_args()):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
