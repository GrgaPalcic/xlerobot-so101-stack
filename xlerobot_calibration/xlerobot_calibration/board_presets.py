from __future__ import annotations

from copy import deepcopy
from typing import Any


BOARD_PRESETS: dict[str, dict[str, Any]] = {
    "intrinsics_a3_11x8_34_25_id2": {
        "title": "A3 caib.io intrinsics board, 11 x 8, 34/25 mm, start id 2",
        "role": "intrinsics",
        "asset": "docs/assets/fiducials/calib.io_charuco_420x297_8x11_34_25_DICT_5X5.pdf",
        "notes": (
            "Use for camera intrinsics. This is the large board from the first "
            "calibration pass."
        ),
        "config": {
            "intr_cols": 11,
            "intr_rows": 8,
            "intr_square_m": 0.034,
            "intr_marker_m": 0.025,
            "intr_start_id": 2,
            "intr_marker_count": 44,
            "intr_dict": "DICT_5X5_100",
        },
    },
    "world_plate_a_7x5_20_14_id49": {
        "title": "Workspace Plate A, 7 x 5, 20/14 mm, start id 49",
        "role": "world",
        "asset": "docs/assets/fiducials/xlerobot_world_targets_7x5_20mm_aruco5x5_100_ids49-99_a4.pdf",
        "notes": "Default smaller rigid workspace/world target.",
        "config": {
            "world_cols": 7,
            "world_rows": 5,
            "world_square_m": 0.020,
            "world_marker_m": 0.014,
            "world_start_id": 49,
            "world_marker_count": 17,
            "world_dict": "DICT_5X5_100",
        },
    },
    "world_plate_b_7x5_20_14_id66": {
        "title": "Workspace Plate B, 7 x 5, 20/14 mm, start id 66",
        "role": "world",
        "asset": "docs/assets/fiducials/xlerobot_world_targets_7x5_20mm_aruco5x5_100_ids49-99_a4.pdf",
        "notes": "Use if Plate A print, glare, mounting, or detection quality is poor.",
        "config": {
            "world_cols": 7,
            "world_rows": 5,
            "world_square_m": 0.020,
            "world_marker_m": 0.014,
            "world_start_id": 66,
            "world_marker_count": 17,
            "world_dict": "DICT_5X5_100",
        },
    },
    "world_plate_c_7x5_20_14_id83": {
        "title": "Workspace Plate C, 7 x 5, 20/14 mm, start id 83",
        "role": "world",
        "asset": "docs/assets/fiducials/xlerobot_world_targets_7x5_20mm_aruco5x5_100_ids49-99_a4.pdf",
        "notes": "Use if Plate A/B print, glare, mounting, or detection quality is poor.",
        "config": {
            "world_cols": 7,
            "world_rows": 5,
            "world_square_m": 0.020,
            "world_marker_m": 0.014,
            "world_start_id": 83,
            "world_marker_count": 17,
            "world_dict": "DICT_5X5_100",
        },
    },
}


def preset_names() -> list[str]:
    return sorted(BOARD_PRESETS)


def get_board_preset(name: str) -> dict[str, Any]:
    try:
        return deepcopy(BOARD_PRESETS[name])
    except KeyError as exc:
        valid = ", ".join(preset_names())
        raise ValueError(f"unknown board preset {name!r}; valid presets: {valid}") from exc


def apply_board_preset(state: dict[str, Any], name: str) -> dict[str, Any]:
    preset = get_board_preset(name)
    state.setdefault("config", {}).update(preset["config"])
    return preset
