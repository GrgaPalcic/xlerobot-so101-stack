import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "record_board_touch_points.py"


def load_module():
    spec = importlib.util.spec_from_file_location("record_board_touch_points", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_jog_axis_command():
    module = load_module()

    command = module.parse_jog_command("x+")

    assert command.kind == "move"
    np.testing.assert_allclose(command.delta_axis, [1.0, 0.0, 0.0])


def test_parse_jog_sample_aliases():
    module = load_module()

    assert module.parse_jog_command("").kind == "sample"
    assert module.parse_jog_command("record").kind == "sample"


def test_parse_jog_step_and_duration():
    module = load_module()

    step = module.parse_jog_command("step 0.005")
    duration = module.parse_jog_command("dur 2.5")

    assert step.kind == "step"
    assert step.value == 0.005
    assert duration.kind == "duration"
    assert duration.value == 2.5


def test_parse_joint_jog_command():
    module = load_module()

    command = module.parse_jog_command("elbow-")

    assert command.kind == "joint_move"
    assert command.joint_name == "elbow_flex"
    assert command.joint_sign == -1.0


def test_parse_joint_step_command():
    module = load_module()

    command = module.parse_jog_command("jstep 0.04")

    assert command.kind == "joint_step"
    assert command.value == 0.04


def test_parse_axis_command_with_explicit_distance():
    module = load_module()

    command = module.parse_jog_command("z+ 0.02")

    assert command.kind == "move"
    np.testing.assert_allclose(command.delta_axis, [0.0, 0.0, 1.0])
    assert command.value == 0.02


def test_parse_joint_command_with_explicit_step():
    module = load_module()

    command = module.parse_jog_command("roll+ 0.05")

    assert command.kind == "joint_move"
    assert command.joint_name == "wrist_roll"
    assert command.joint_sign == 1.0
    assert command.value == 0.05


def test_parse_reference_command():
    module = load_module()

    assert module.parse_jog_command("ref").kind == "reference"
    assert module.parse_jog_command("distance").kind == "reference"


def test_parse_sync_command():
    module = load_module()

    assert module.parse_jog_command("sync").kind == "sync"
    assert module.parse_jog_command("rebase").kind == "sync"


def test_expected_corner_distance():
    module = load_module()

    assert module.expected_corner_distance_m("top_left", "top_right", 0.14, 0.10) == 0.14
    assert module.expected_corner_distance_m("top_left", "bottom_left", 0.14, 0.10) == 0.10
    assert module.expected_corner_distance_m("top_left", "bottom_right", 0.14, 0.10) == pytest.approx(
        (0.14**2 + 0.10**2) ** 0.5
    )


def test_parse_jog_rejects_bad_command():
    module = load_module()

    with pytest.raises(ValueError):
        module.parse_jog_command("north")
