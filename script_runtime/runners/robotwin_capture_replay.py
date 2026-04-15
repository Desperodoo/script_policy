"""Run a lightweight RoboTwin replay pass focused on real-view keyframe export."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict

from script_runtime.session import build_robotwin_pick_place_session, load_runtime_config


def _prepare_capture_config(config: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    runtime = dict(config.get("runtime", {}))
    robotwin = dict(config.get("robotwin", {}))
    config = dict(config)

    base_trace = Path(runtime.get("trace_path", "script_runtime/artifacts/robotwin_capture_trace.jsonl"))
    base_artifact_dir = Path(runtime.get("artifact_dir", "script_runtime/artifacts/robotwin_capture"))
    runtime["trace_path"] = str(base_trace.with_name(f"{base_trace.stem}_capture{base_trace.suffix}"))
    runtime["artifact_dir"] = str(base_artifact_dir.with_name(f"{base_artifact_dir.name}_capture"))

    robotwin["capture_video"] = True
    robotwin["capture_every_n_steps"] = int(args.capture_every_n_steps)
    if args.capture_skills:
        robotwin["capture_skills"] = [part.strip() for part in args.capture_skills.split(",") if part.strip()]
    else:
        robotwin["capture_skills"] = ["GoPregrasp", "ExecuteGraspPhase", "Lift", "PlaceApproach", "PlaceRelease", "OpenGripper"]
    robotwin["capture_command_types"] = ["reset", "final"]

    config["runtime"] = runtime
    config["robotwin"] = robotwin
    return config


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a dedicated RoboTwin replay pass for real-view exports.")
    parser.add_argument(
        "--config",
        default="script_runtime/configs/tasks/place_empty_cup_robotwin.yaml",
        help="Path to the runtime YAML config.",
    )
    parser.add_argument(
        "--capture-every-n-steps",
        type=int,
        default=0,
        help="Periodic capture interval. Use 0 to disable periodic capture and only keep keyframes.",
    )
    parser.add_argument(
        "--capture-skills",
        default="",
        help="Comma-separated skill names to capture. Empty uses the default keyframe set.",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    config = load_runtime_config(args.config)
    config = _prepare_capture_config(config, args)
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
