#!/usr/bin/env python3
"""Calibrate only the SO101 leader/follower gripper motors.

This mirrors LeRobot's SO101 calibration flow, but scopes all writes to motor
ID 6 ("gripper") so existing arm joint calibration is preserved.
"""

from __future__ import annotations

import argparse
import json
import select
import shutil
import sys
import time
from dataclasses import asdict
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


GRIPPER = "gripper"
MOTOR_ID = 6
MODEL = "sts3215"
RESOLUTION_MAX = 4095


def make_bus(port: str) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={GRIPPER: Motor(MOTOR_ID, MODEL, MotorNormMode.RANGE_0_100)},
    )


def wait_enter(prompt: str) -> None:
    print()
    input(prompt)


def read_raw_positions(buses: dict[str, FeetechMotorsBus]) -> dict[str, int]:
    return {
        name: int(bus.read("Present_Position", GRIPPER, normalize=False))
        for name, bus in buses.items()
    }


def print_live_table(positions: dict[str, int], mins: dict[str, int], maxes: dict[str, int]) -> None:
    print("\033[2J\033[H", end="")
    print("SO101 GRIPPER-ONLY RANGE CAPTURE")
    print()
    print("Move BOTH grippers through full travel: fully open, fully closed, repeat.")
    print("Press ENTER when both min/max values stop changing.")
    print()
    print(f"{'DEVICE':<10} | {'MIN':>6} | {'POS':>6} | {'MAX':>6} | {'SPAN':>6}")
    print("-" * 50)
    for name in ("leader", "follower"):
        span = maxes[name] - mins[name]
        print(f"{name:<10} | {mins[name]:>6} | {positions[name]:>6} | {maxes[name]:>6} | {span:>6}")
    print()
    print("Do not force past mechanical stops. A span under ~150 ticks is suspicious.")
    sys.stdout.flush()


def record_ranges(buses: dict[str, FeetechMotorsBus]) -> tuple[dict[str, int], dict[str, int]]:
    positions = read_raw_positions(buses)
    mins = positions.copy()
    maxes = positions.copy()

    while True:
        positions = read_raw_positions(buses)
        for name, pos in positions.items():
            mins[name] = min(mins[name], pos)
            maxes[name] = max(maxes[name], pos)

        print_live_table(positions, mins, maxes)

        ready, _, _ = select.select([sys.stdin], [], [], 0.08)
        if ready:
            sys.stdin.readline()
            break

    repeated = [name for name in mins if mins[name] == maxes[name]]
    if repeated:
        raise RuntimeError(f"No motion captured for: {', '.join(repeated)}")
    return mins, maxes


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json_with_backup(path: Path, data: dict) -> Path:
    backup = path.with_suffix(path.suffix + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
    shutil.copy2(path, backup)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
        f.write("\n")
    return backup


def update_json_calibration(path: Path, calibration: MotorCalibration) -> Path:
    data = load_json(path)
    existing = data.get(GRIPPER, {})
    data[GRIPPER] = {
        "id": int(existing.get("id", MOTOR_ID)),
        "drive_mode": int(existing.get("drive_mode", 0)),
        "homing_offset": int(calibration.homing_offset),
        "range_min": int(calibration.range_min),
        "range_max": int(calibration.range_max),
    }
    return write_json_with_backup(path, data)


def replace_yaml_value(line: str, key: str, value: int) -> str:
    indent = line[: len(line) - len(line.lstrip())]
    comment = ""
    if "#" in line:
        _, comment_part = line.split("#", 1)
        comment = " #" + comment_part.rstrip("\n")
    return f"{indent}{key}: {value}{comment}\n"


def update_yaml_gripper(path: Path, calibration: MotorCalibration) -> Path:
    backup = path.with_suffix(path.suffix + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
    shutil.copy2(path, backup)

    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    out: list[str] = []
    in_gripper = False
    gripper_indent = None
    seen = {"homing_offset": False, "range_min": False, "range_max": False}
    values = {
        "homing_offset": int(calibration.homing_offset),
        "range_min": int(calibration.range_min),
        "range_max": int(calibration.range_max),
    }

    for line in lines:
        stripped = line.lstrip()
        indent = len(line) - len(stripped)

        if stripped.startswith("gripper:"):
            in_gripper = True
            gripper_indent = indent
            out.append(line)
            continue

        if in_gripper and stripped and not stripped.startswith("#") and indent <= int(gripper_indent):
            in_gripper = False

        if in_gripper:
            key = stripped.split(":", 1)[0].strip() if ":" in stripped else ""
            if key in values:
                out.append(replace_yaml_value(line, key, values[key]))
                seen[key] = True
                continue

        out.append(line)

    missing = [key for key, found in seen.items() if not found]
    if missing:
        raise RuntimeError(f"Missing gripper keys in {path}: {', '.join(missing)}")

    path.write_text("".join(out), encoding="utf-8")
    return backup


def write_motor_calibration(
    buses: dict[str, FeetechMotorsBus],
    calibrations: dict[str, MotorCalibration],
) -> None:
    for name, bus in buses.items():
        bus.write_calibration({GRIPPER: calibrations[name]})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", default="/dev/ttyUSB1")
    parser.add_argument("--follower-port", default="/dev/ttyUSB0")
    parser.add_argument(
        "--leader-json",
        default="/home/dell/Documents/lerobot-calib/calibration/teleoperators/so_leader/my_leader.json",
    )
    parser.add_argument(
        "--follower-json",
        default="/home/dell/Documents/lerobot-calib/calibration/robots/so_follower/my_follower.json",
    )
    parser.add_argument(
        "--leader-yaml",
        default="/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/hardware/leader_joints.yaml",
    )
    parser.add_argument(
        "--follower-yaml",
        default="/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/hardware/follower_joints.yaml",
    )
    parser.add_argument(
        "--installed-leader-yaml",
        default="/home/dell/Documents/so101-ros-physical-ai/install/so101_bringup/share/so101_bringup/config/hardware/leader_joints.yaml",
    )
    parser.add_argument(
        "--installed-follower-yaml",
        default="/home/dell/Documents/so101-ros-physical-ai/install/so101_bringup/share/so101_bringup/config/hardware/follower_joints.yaml",
    )
    parser.add_argument("--no-write", action="store_true", help="Capture and print values without writing files/motors.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ports = {"leader": args.leader_port, "follower": args.follower_port}
    buses = {name: make_bus(port) for name, port in ports.items()}

    print("SO101 GRIPPER-ONLY CALIBRATION")
    print()
    print(f"Leader port:   {args.leader_port}")
    print(f"Follower port: {args.follower_port}")
    print()
    print("This will touch only motor ID 6 on each arm.")
    print("Arm joint calibration is preserved.")

    try:
        for name, bus in buses.items():
            print(f"Connecting {name} gripper bus on {ports[name]}...")
            bus.connect(handshake=True)
            bus.disable_torque(GRIPPER, num_retry=5)
            bus.write("Operating_Mode", GRIPPER, 0)

        wait_enter("Place BOTH grippers halfway open, then press ENTER to set gripper homing offsets...")
        homings = {}
        for name, bus in buses.items():
            bus.reset_calibration(GRIPPER)
            position = int(bus.read("Present_Position", GRIPPER, normalize=False))
            homing = position - int(RESOLUTION_MAX / 2)
            bus.write("Homing_Offset", GRIPPER, homing)
            homings[name] = int(homing)
            print(f"{name}: current raw position {position}, homing_offset {homing}")

        wait_enter("Now prepare to move BOTH grippers from fully open to fully closed. Press ENTER to start capture...")
        mins, maxes = record_ranges(buses)

        calibrations = {
            name: MotorCalibration(
                id=MOTOR_ID,
                drive_mode=0,
                homing_offset=homings[name],
                range_min=mins[name],
                range_max=maxes[name],
            )
            for name in buses
        }

        print("\nCaptured gripper calibration:")
        for name, calibration in calibrations.items():
            print(f"{name}: {asdict(calibration)} span={calibration.range_max - calibration.range_min}")

        if args.no_write:
            print("\n--no-write set; not writing files or motor EEPROM.")
            return 0

        print("\nWriting calibration to motor EEPROM and config files...")
        write_motor_calibration(buses, calibrations)

        backups = []
        backups.append(update_json_calibration(Path(args.leader_json), calibrations["leader"]))
        backups.append(update_json_calibration(Path(args.follower_json), calibrations["follower"]))
        backups.append(update_yaml_gripper(Path(args.leader_yaml), calibrations["leader"]))
        backups.append(update_yaml_gripper(Path(args.follower_yaml), calibrations["follower"]))

        for maybe_path, name in (
            (args.installed_leader_yaml, "leader installed YAML"),
            (args.installed_follower_yaml, "follower installed YAML"),
        ):
            path = Path(maybe_path)
            if path.exists():
                calibration = calibrations["leader"] if "leader" in name else calibrations["follower"]
                backups.append(update_yaml_gripper(path, calibration))

        print("\nDone. Backups created:")
        for backup in backups:
            print(f"  {backup}")
        print("\nNext: restart ROS teleop so the driver reloads the updated gripper calibration.")
        return 0
    finally:
        for bus in buses.values():
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
