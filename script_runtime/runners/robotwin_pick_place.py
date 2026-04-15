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
    args = parser.parse_args()

    config = load_runtime_config(args.config)
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
