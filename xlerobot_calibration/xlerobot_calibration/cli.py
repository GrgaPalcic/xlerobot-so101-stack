from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml

from .runner import StepError, doctor, run_step
from .state import (
    apply_config_overrides,
    create_state,
    default_out_dir,
    find_latest_state,
    load_state,
    make_run_id,
    save_state,
    step_status,
)
from .steps import STEPS, steps_by_id


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="XLeRobot dual-arm calibration runner")
    parser.add_argument("--workspace", default=str(Path.cwd()), help="SO-101 workspace root")
    parser.add_argument("--out", default="", help="Run output directory. Defaults to latest or a new field_runs/xlerobot_*")
    sub = parser.add_subparsers(dest="command", required=True)

    wizard = sub.add_parser("wizard", help="Run the interactive resumable wizard")
    add_common_options(wizard)
    wizard.add_argument("--run-id", default="", help="Run id for a new run")
    wizard.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="Override run config")
    wizard.add_argument("--yes", action="store_true", help="Assume yes for prompts")

    run = sub.add_parser("run-step", help="Run or show one step")
    add_common_options(run)
    run.add_argument("step_id", choices=[step.id for step in STEPS])
    run.add_argument("--dry-run", action="store_true")
    run.add_argument("--yes", action="store_true", help="Assume yes for prompts")
    run.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="Override run config")

    status = sub.add_parser("status", help="Show current run status")
    add_common_options(status)
    show_config = sub.add_parser("show-config", help="Print current run config")
    add_common_options(show_config)

    set_config = sub.add_parser("set-config", help="Set one config value in run_state.yaml")
    add_common_options(set_config)
    set_config.add_argument("key")
    set_config.add_argument("value")

    doctor_cmd = sub.add_parser("doctor", help="Check local tools and git signing config")
    add_common_options(doctor_cmd)
    export_report = sub.add_parser("export-report", help="Write a markdown field report draft")
    add_common_options(export_report)
    return parser


def add_common_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--workspace", default=argparse.SUPPRESS, help="SO-101 workspace root")
    parser.add_argument("--out", default=argparse.SUPPRESS, help="Run output directory")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    workspace = Path(args.workspace).resolve()

    try:
        if args.command == "doctor":
            for line in doctor(workspace):
                print(line)
            return 0

        if args.command == "wizard":
            state = resolve_or_create_state(args, workspace)
            apply_config_overrides(state, args.set)
            save_state(state)
            return wizard_loop(state, yes=args.yes)

        state = resolve_existing_state(args, workspace)

        if args.command == "status":
            print_status(state)
            return 0
        if args.command == "show-config":
            print(yaml.safe_dump(state.get("config", {}), sort_keys=True))
            return 0
        if args.command == "set-config":
            state.setdefault("config", {})[args.key] = args.value
            save_state(state)
            print(f"{args.key}={args.value}")
            return 0
        if args.command == "run-step":
            apply_config_overrides(state, args.set)
            run_step(state, args.step_id, dry_run=args.dry_run, yes=args.yes)
            return 0
        if args.command == "export-report":
            run_step(state, "export_report", yes=True)
            print(Path(state["out_dir"]) / "xlerobot_calibration_report.md")
            return 0
    except (FileNotFoundError, StepError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 1


def resolve_or_create_state(args: argparse.Namespace, workspace: Path) -> dict[str, Any]:
    if args.out:
        out = Path(args.out).resolve()
        if (out / "run_state.yaml").exists():
            return load_state(out)
        return create_state(workspace, run_id=args.run_id or make_run_id(), out_dir=out)
    latest = find_latest_state(workspace)
    if latest is not None:
        return latest
    run_id = args.run_id or make_run_id()
    return create_state(workspace, run_id=run_id, out_dir=default_out_dir(workspace, run_id))


def resolve_existing_state(args: argparse.Namespace, workspace: Path) -> dict[str, Any]:
    if args.out:
        return load_state(Path(args.out).resolve())
    latest = find_latest_state(workspace)
    if latest is None:
        raise FileNotFoundError("no run_state.yaml found; start with xlerobot-calib wizard")
    return latest


def print_status(state: dict[str, Any]) -> None:
    print(f"run_id: {state['run_id']}")
    print(f"out_dir: {state['out_dir']}")
    print("")
    for step in STEPS:
        print(f"{step_status(state, step.id):10} {step.id:28} {step.title}")


def next_pending_step(state: dict[str, Any]) -> str | None:
    for step in STEPS:
        if step_status(state, step.id) != "complete":
            return step.id
    return None


def wizard_loop(state: dict[str, Any], *, yes: bool) -> int:
    while True:
        print("")
        print_status(state)
        pending = next_pending_step(state)
        print("")
        print(f"next: {pending or 'none'}")
        print("Commands: n=run next, d=dry-run next, r=report, c=config, q=quit, or enter a step id")
        choice = input("> ").strip()
        if choice in {"q", "quit", "exit"}:
            return 0
        if choice == "c":
            print(yaml.safe_dump(state.get("config", {}), sort_keys=True))
            continue
        if choice == "r":
            run_step(state, "export_report", yes=True)
            print(Path(state["out_dir"]) / "xlerobot_calibration_report.md")
            continue
        if choice in {"n", ""}:
            if pending is None:
                print("No pending steps.")
                continue
            run_step(state, pending, yes=yes)
            continue
        if choice == "d":
            if pending is None:
                print("No pending steps.")
                continue
            run_step(state, pending, dry_run=True, yes=True)
            continue
        if choice not in steps_by_id():
            print(f"Unknown step: {choice}")
            continue
        run_step(state, choice, yes=yes)


if __name__ == "__main__":
    raise SystemExit(main())
