from pathlib import Path


def test_cartesian_motion_node_uses_repo_planner():
    source = (
        Path(__file__).resolve().parents[1]
        / "so101_kinematics"
        / "so101_kinematics"
        / "cartesian_motion_node.py"
    ).read_text(encoding="utf-8")

    assert "from so101_kinematics.motion_planner import MotionPlanner" in source
    assert "from robokin.motion_planner import MotionPlanner" not in source

