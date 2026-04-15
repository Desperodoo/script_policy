#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import os
import sys
from contextlib import contextmanager
from pathlib import Path

import yaml


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def robotwin_root() -> Path:
    return repo_root() / "third_party" / "RoboTwin"


@contextmanager
def pushd(path: Path):
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def check_assets(root: Path) -> list[str]:
    required = [
        root / "assets" / "objects" / "objaverse" / "list.json",
        root / "assets" / "embodiments",
        root / "assets" / "background_texture",
    ]
    missing = [str(path) for path in required if not path.exists()]
    return missing


def load_task_config(root: Path, config_name: str) -> dict:
    config_path = root / "task_config" / f"{config_name}.yml"
    with config_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def build_demo_args(root: Path, task_name: str, task_config: str) -> dict:
    cfg = load_task_config(root, task_config)
    cfg["task_name"] = task_name
    cfg["task_config"] = task_config
    cfg["use_seed"] = True
    cfg["collect_data"] = False
    cfg["render_freq"] = 0
    cfg["save_data"] = False
    cfg["eval_mode"] = True
    cfg["seed"] = 0
    cfg["now_ep_num"] = 0
    cfg["save_path"] = str(root / "data" / "_smoke")

    embodiment_config_path = root / "task_config" / "_embodiment_config.yml"
    with embodiment_config_path.open("r", encoding="utf-8") as handle:
        embodiment_types = yaml.safe_load(handle) or {}

    embodiment = cfg.get("embodiment", ["aloha-agilex"])
    if len(embodiment) != 1:
        raise ValueError(f"Smoke test currently expects single embodiment config, got: {embodiment}")
    embodiment_name = embodiment[0]
    robot_file = embodiment_types[embodiment_name]["file_path"]
    cfg["left_robot_file"] = robot_file
    cfg["right_robot_file"] = robot_file
    cfg["dual_arm_embodied"] = True

    def load_embodiment_config(path_str: str) -> dict:
        with open(root / path_str[2:] / "config.yml", "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    cfg["left_embodiment_config"] = load_embodiment_config(robot_file)
    cfg["right_embodiment_config"] = load_embodiment_config(robot_file)
    cfg["embodiment_name"] = embodiment_name
    return cfg


def import_task_class(root: Path, task_name: str):
    sys.path.insert(0, str(root))
    with pushd(root):
        envs_module = importlib.import_module(f"envs.{task_name}")
    if not hasattr(envs_module, task_name):
        raise AttributeError(f"Task module envs.{task_name} does not expose class {task_name}")
    return getattr(envs_module, task_name)


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal RoboTwin smoke test for script_policy.")
    parser.add_argument("--task", default="place_empty_cup", help="RoboTwin task module/class name")
    parser.add_argument("--config", default="demo_clean", help="RoboTwin task config name without suffix")
    parser.add_argument(
        "--setup-demo",
        action="store_true",
        help="Instantiate the task and run setup_demo() once. Requires full assets and simulator runtime.",
    )
    args = parser.parse_args()

    root = robotwin_root()
    print(f"[smoke] RoboTwin root: {root}")

    missing_assets = check_assets(root)
    if missing_assets:
        print("[smoke] Missing assets:")
        for item in missing_assets:
            print(f"  - {item}")
    else:
        print("[smoke] Asset structure looks complete")

    try:
        task_cls = import_task_class(root, args.task)
    except Exception as exc:
        print(f"[smoke] Failed to import task class: {exc!r}")
        if missing_assets:
            print("[smoke] This is likely because RoboTwin assets are not fully downloaded yet.")
            return 2
        raise
    print(f"[smoke] Imported task class: {task_cls.__name__}")

    if not args.setup_demo:
        print("[smoke] Import-only check passed")
        return 0 if not missing_assets else 2

    if missing_assets:
        print("[smoke] Refusing to run setup_demo because assets are incomplete")
        return 2

    kwargs = build_demo_args(root, args.task, args.config)
    print(f"[smoke] Running setup_demo(task={args.task}, config={args.config})")
    with pushd(root):
        env = task_cls()
        env.setup_demo(**kwargs)
        env.close_env()
    print("[smoke] setup_demo passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
