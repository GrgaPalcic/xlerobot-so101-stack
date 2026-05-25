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


def test_parse_jog_rejects_bad_command():
    module = load_module()

    with pytest.raises(ValueError):
        module.parse_jog_command("north")
