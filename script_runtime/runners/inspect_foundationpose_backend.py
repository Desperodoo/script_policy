"""Prepare and optionally validate FoundationPose inputs from a RoboTwin snapshot."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any, Dict

import numpy as np

from script_runtime.adapters.fm_grasp_stack import FoundationPoseEstimator
from script_runtime.core import SkillContext
from script_runtime.core.skill_base import clear_pending_refresh_reason, set_pending_refresh_reason
from script_runtime.session import build_robotwin_pick_place_session, load_runtime_config
from script_runtime.skills.perception.primitives import GetObjectPose


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare FoundationPose validation package from RoboTwin.")
    parser.add_argument(
        "--config",
        default="script_runtime/configs/tasks/place_container_plate_robotwin_fm_first.yaml",
        help="Path to runtime YAML config.",
    )
    parser.add_argument("--task-id", default="foundationpose_inspect", help="Override task id.")
    parser.add_argument("--attempt-run", action="store_true", help="Attempt to run FoundationPose if dependencies look ready.")
    parser.add_argument("--python-bin", default="python", help="Python executable used for the backend subprocess.")
    parser.add_argument("--timeout-s", type=int, default=120, help="Timeout for backend subprocess.")
    parser.add_argument("--out", default="", help="Optional JSON output path.")
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    config = load_runtime_config(args.config)
    runtime = config.setdefault("runtime", {})
    runtime["task_id"] = str(args.task_id)
    session = build_robotwin_pick_place_session(config)
    try:
        sdk = session.adapters["sdk"]
        init = sdk.initialize()
        if not init.get("ok", False):
            raise RuntimeError(f"Failed to initialize RoboTwin bridge: {init}")
        if hasattr(sdk, "attach_blackboard"):
            sdk.attach_blackboard(session.blackboard)
        set_pending_refresh_reason(session.blackboard, "session_initialize")
        sdk.refresh_world(session.blackboard)
        clear_pending_refresh_reason(session.blackboard)

        context = SkillContext(
            blackboard=session.blackboard,
            adapters=session.adapters,
            task_id=str(args.task_id),
            metadata={"session_type": "foundationpose_inspect"},
        )
        object_result = GetObjectPose().run(context)
        perception = session.adapters.get("perception")
        grounding = getattr(perception, "last_grounding", None)
        snapshot = session.adapters["camera"].get_snapshot()

        repos = dict((config.get("perception_stack") or {}).get("repos") or {})
        fp_cfg = dict((config.get("perception_stack") or {}).get("foundationpose") or {})
        estimator = FoundationPoseEstimator(
            repo_path=repos.get("foundationpose"),
            python_bin=fp_cfg.get("python_bin") or str(args.python_bin),
            timeout_s=int(fp_cfg.get("timeout_s", args.timeout_s)),
        )
        readiness = estimator._readiness()
        output_root = _resolve_output_root(config=config, task_id=str(args.task_id), explicit_out=str(args.out or ""))
        export_dir = output_root.parent / f"{output_root.stem}_foundationpose_input"
        export_dir.mkdir(parents=True, exist_ok=True)

        asset_info = sdk.get_object_asset_info() if hasattr(sdk, "get_object_asset_info") else {}
        export_outputs = _export_foundationpose_input(
            export_dir=export_dir,
            snapshot=snapshot,
            grounding=grounding,
            asset_info=asset_info,
        )
        if export_outputs.get("ok", False):
            mesh_outputs = estimator._prepare_mesh_file_for_runtime(
                source_mesh_file=str(export_outputs.get("mesh_file", "")),
                export_dir=export_dir,
            )
            export_outputs["mesh_outputs"] = mesh_outputs
            if mesh_outputs.get("ok", False):
                export_outputs["source_mesh_file"] = export_outputs.get("mesh_file", "")
                export_outputs["mesh_file"] = str(mesh_outputs.get("mesh_file", ""))
        command = _build_command(
            python_bin=str(fp_cfg.get("python_bin") or args.python_bin),
            repo_path=readiness.get("resolved_repo_path", ""),
            mesh_path=str(export_outputs.get("mesh_file", "")),
            test_scene_dir=str(export_outputs.get("test_scene_dir", "")),
        )
        payload: Dict[str, Any] = {
            "task_id": str(args.task_id),
            "object_result": {
                "status": object_result.status.value,
                "message": object_result.message,
                "payload": object_result.payload,
            },
            "grounding": {
                "target_name": None if grounding is None else grounding.target_name,
                "box_xyxy": None if grounding is None else grounding.box_xyxy,
                "metadata": {} if grounding is None else dict(grounding.metadata or {}),
            },
            "foundationpose_readiness": readiness,
            "asset_info": asset_info,
            "export_outputs": export_outputs,
            "command": command,
            "attempted_run": False,
            "blockers": list(readiness.get("blockers") or []),
        }
        if args.attempt_run:
            payload["attempted_run"] = True
            payload["run_result"] = _attempt_run(command=command, readiness=readiness, timeout_s=int(args.timeout_s))

        output_root.parent.mkdir(parents=True, exist_ok=True)
        output_root.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    finally:
        session.shutdown()
    return 0


def _resolve_output_root(*, config: Dict[str, Any], task_id: str, explicit_out: str) -> Path:
    if explicit_out:
        return Path(explicit_out)
    runtime = dict(config.get("runtime") or {})
    artifact_dir = runtime.get("artifact_dir") or "script_runtime/artifacts"
    run_dir = Path(artifact_dir) / task_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir / f"{task_id}_foundationpose_inspect.json"


def _export_foundationpose_input(
    *,
    export_dir: Path,
    snapshot: Any,
    grounding: Any,
    asset_info: Dict[str, Any],
) -> Dict[str, Any]:
    try:
        import cv2
    except Exception as exc:
        return {"ok": False, "message": f"missing_cv2: {exc!r}"}

    rgb = None if snapshot is None else np.asarray(snapshot.rgb)
    depth = None if snapshot is None else np.asarray(snapshot.depth, dtype=np.float64)
    if rgb is None or depth is None:
        return {"ok": False, "message": "missing_snapshot_rgb_or_depth"}

    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    if rgb.ndim == 3 and rgb.shape[2] > 3:
        rgb = rgb[:, :, :3]

    rgb_dir = export_dir / "rgb"
    depth_dir = export_dir / "depth"
    mask_dir = export_dir / "masks"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)
    mask_dir.mkdir(parents=True, exist_ok=True)

    rgb_path = rgb_dir / "000000.png"
    depth_path = depth_dir / "000000.png"
    mask_path = mask_dir / "000000.png"
    cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(depth_path), np.clip(depth, 0, np.iinfo(np.uint16).max).astype(np.uint16))
    mask, mask_source = _resolve_grounding_mask(shape=depth.shape, grounding=grounding)
    cv2.imwrite(str(mask_path), (mask.astype(np.uint8) * 255))

    camera_params = {} if snapshot is None else dict(getattr(snapshot, "metadata", {}) or {}).get("camera_params", {}) or {}
    intrinsic = np.asarray(camera_params.get("intrinsic_cv", np.eye(3)), dtype=np.float64)
    cam_k_path = export_dir / "cam_K.txt"
    np.savetxt(cam_k_path, intrinsic.reshape(3, 3))

    preview_path = export_dir / "preview.png"
    preview = rgb.copy()
    if np.count_nonzero(mask):
        preview[mask] = (0.55 * preview[mask] + 0.45 * np.array([235, 87, 87], dtype=np.float64)).astype(np.uint8)
    if grounding is not None and grounding.box_xyxy is not None:
        x1, y1, x2, y2 = [int(round(v)) for v in list(grounding.box_xyxy)[:4]]
        cv2.rectangle(preview, (x1, y1), (x2, y2), (235, 87, 87), 3)
    cv2.imwrite(str(preview_path), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))

    mesh_file = str(asset_info.get("visual_mesh_path") or asset_info.get("collision_mesh_path") or "")
    metadata = {
        "mask_source": mask_source,
        "mesh_file": mesh_file,
        "asset_info": asset_info,
        "camera_params_available": bool(camera_params),
        "mask_foreground_pixels": int(np.count_nonzero(mask)),
    }
    metadata_path = export_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    return {
        "ok": True,
        "test_scene_dir": str(export_dir),
        "rgb_path": str(rgb_path),
        "depth_path": str(depth_path),
        "mask_path": str(mask_path),
        "cam_k_path": str(cam_k_path),
        "preview_path": str(preview_path),
        "metadata_path": str(metadata_path),
        "mesh_file": mesh_file,
        "mask_foreground_pixels": int(np.count_nonzero(mask)),
        "mask_source": mask_source,
    }


def _build_bbox_mask(*, shape: tuple[int, ...], box_xyxy: Any) -> np.ndarray:
    height, width = int(shape[0]), int(shape[1])
    mask = np.zeros((height, width), dtype=bool)
    if box_xyxy is None or len(list(box_xyxy)) < 4:
        return mask
    x1, y1, x2, y2 = [int(round(float(v))) for v in list(box_xyxy)[:4]]
    x1 = max(min(x1, width - 1), 0)
    y1 = max(min(y1, height - 1), 0)
    x2 = max(min(x2, width), x1 + 1)
    y2 = max(min(y2, height), y1 + 1)
    mask[y1:y2, x1:x2] = True
    return mask


def _resolve_grounding_mask(*, shape: tuple[int, ...], grounding: Any) -> tuple[np.ndarray, str]:
    if grounding is not None:
        mask = getattr(grounding, "mask", None)
        if mask is not None:
            mask_array = np.asarray(mask, dtype=bool)
            if mask_array.shape == tuple(shape):
                return mask_array, str(dict(getattr(grounding, "metadata", {}) or {}).get("mask_source", "grounding_mask"))
        box_xyxy = getattr(grounding, "box_xyxy", None)
    else:
        box_xyxy = None
    return _build_bbox_mask(shape=shape, box_xyxy=box_xyxy), "grounding_bbox_rect"


def _build_command(*, python_bin: str, repo_path: str, mesh_path: str, test_scene_dir: str) -> Dict[str, Any]:
    repo_root = Path(str(repo_path or "")).expanduser().resolve()
    mesh_root = Path(str(mesh_path or "")).expanduser().resolve()
    scene_root = Path(str(test_scene_dir or "")).expanduser().resolve()
    debug_dir = str(scene_root / "foundationpose_debug")
    cmd = [
        str(Path(str(python_bin)).expanduser()),
        str(repo_root / "run_demo.py"),
        "--mesh_file",
        str(mesh_root),
        "--test_scene_dir",
        str(scene_root),
        "--debug",
        "0",
        "--debug_dir",
        debug_dir,
    ]
    return {
        "cwd": str(repo_root),
        "argv": cmd,
        "shell_preview": " ".join(cmd),
        "debug_dir": debug_dir,
    }


def _attempt_run(*, command: Dict[str, Any], readiness: Dict[str, Any], timeout_s: int) -> Dict[str, Any]:
    blockers = list(readiness.get("blockers", []) or [])
    hard_blockers = {"repo_missing", "weights_missing", "missing_refiner_weights", "missing_scorer_weights"}
    if any(blocker in hard_blockers for blocker in blockers):
        return {"ok": False, "blocked": True, "blockers": blockers}
    try:
        result = subprocess.run(
            command["argv"],
            cwd=command["cwd"] or None,
            capture_output=True,
            text=True,
            timeout=max(int(timeout_s), 1),
            check=False,
        )
    except Exception as exc:
        return {"ok": False, "blocked": False, "message": repr(exc)}
    return {
        "ok": result.returncode == 0,
        "blocked": False,
        "returncode": int(result.returncode),
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
    }


if __name__ == "__main__":
    raise SystemExit(main())
