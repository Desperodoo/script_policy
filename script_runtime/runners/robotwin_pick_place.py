"""Run the pick-place runtime against a RoboTwin task."""

from __future__ import annotations

import argparse
import json

from script_runtime.session import build_robotwin_pick_place_session, load_runtime_config


def main() -> int:
    parser = argparse.ArgumentParser(description="Run script_runtime pick-place against RoboTwin.")
    parser.add_argument(
        "--config",
        default="script_runtime/configs/tasks/place_empty_cup_robotwin.yaml",
        help="Path to the runtime YAML config.",
    )
    parser.add_argument("--seed", type=int, default=None, help="Override RoboTwin seed.")
    parser.add_argument("--task-id", default=None, help="Override runtime task id.")
    parser.add_argument(
        "--no-artifacts",
        action="store_true",
        help="Skip summary/GIF/image export and only keep runtime result/trace.",
    )
    parser.add_argument(
        "--no-trace",
        action="store_true",
        help="Skip trace file writing.",
    )
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="Disable RoboTwin frame capture during the run.",
    )
    args = parser.parse_args()

    config = load_runtime_config(args.config)
    runtime = config.setdefault("runtime", {})
    robotwin = config.setdefault("robotwin", {})
    if args.seed is not None:
        robotwin["seed"] = int(args.seed)
    if args.task_id is not None:
        runtime["task_id"] = str(args.task_id)
    if args.no_artifacts:
        runtime["export_artifacts"] = False
    if args.no_trace:
        runtime["write_trace"] = False
    if args.no_video:
        robotwin["capture_video"] = False
    session = build_robotwin_pick_place_session(config)
    try:
        result = session.run()
        print(json.dumps({"status": result.status.value, "message": result.message, "payload": result.payload}, ensure_ascii=False, indent=2))
        if session.runtime_artifacts:
            print(json.dumps({"runtime_artifacts": session.runtime_artifacts}, ensure_ascii=False, indent=2))
    finally:
        session.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
