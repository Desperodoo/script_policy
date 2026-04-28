"""Headless GraspGen runner for exported RoboTwin snapshots."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

import numpy as np


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run GraspGen headlessly on an exported NPZ snapshot.")
    parser.add_argument("--repo-path", required=True, help="Path to third_party/GraspGen.")
    parser.add_argument("--npz-path", required=True, help="Input NPZ exported from script_runtime.")
    parser.add_argument("--gripper-config", required=True, help="Model gripper config from GraspGenModels/checkpoints.")
    parser.add_argument("--out-dir", required=True, help="Output directory for summaries and raw predictions.")
    parser.add_argument("--num-grasps", type=int, default=200, help="Number of grasps to sample per attempt.")
    parser.add_argument("--topk-num-grasps", type=int, default=100, help="How many top grasps to keep from inference.")
    parser.add_argument("--max-grasps", type=int, default=60, help="Maximum number of grasps to export.")
    parser.add_argument("--grasp-threshold", type=float, default=-1.0, help="Confidence threshold.")
    parser.add_argument("--min-grasps", type=int, default=40, help="Minimum grasps before stopping retries.")
    parser.add_argument("--max-tries", type=int, default=6, help="Maximum inference retries.")
    parser.add_argument("--max-points", type=int, default=8192, help="Maximum segmented object points to keep.")
    parser.add_argument("--no-remove-outliers", action="store_true", help="Disable GraspGen point-cloud outlier removal.")
    return parser


def _prepare_imports(repo_path: Path) -> None:
    sys.path.insert(0, str(repo_path))


def _patch_huggingface_hub_compat() -> None:
    """Keep old diffusers-based GraspGen code working with newer hub releases."""

    try:
        import huggingface_hub  # type: ignore
    except Exception:
        return
    if hasattr(huggingface_hub, "cached_download"):
        return
    hf_hub_download = getattr(huggingface_hub, "hf_hub_download", None)
    if hf_hub_download is not None:
        setattr(huggingface_hub, "cached_download", hf_hub_download)


def _load_export(npz_path: Path) -> Dict[str, Any]:
    payload = np.load(npz_path)
    rgb = np.asarray(payload["rgb"]) if "rgb" in payload.files else None
    depth = np.asarray(payload["depth"], dtype=np.float32) if "depth" in payload.files else None
    cam_k = np.asarray(payload["K"], dtype=np.float32) if "K" in payload.files else None
    segmap = np.asarray(payload["segmap"], dtype=np.uint8) if "segmap" in payload.files else None
    return {"rgb": rgb, "depth": depth, "cam_k": cam_k, "segmap": segmap}


def _segmented_object_cloud(
    *,
    depth: np.ndarray | None,
    cam_k: np.ndarray | None,
    segmap: np.ndarray | None,
    max_points: int,
) -> np.ndarray:
    if depth is None or cam_k is None or segmap is None:
        raise RuntimeError("input_missing_depth_intrinsics_or_segmap")
    mask = (segmap > 0) & (depth > 0)
    if not np.any(mask):
        raise RuntimeError("segmap_empty")
    ys, xs = np.nonzero(mask)
    z = depth[ys, xs].astype(np.float32)
    x = (xs.astype(np.float32) - float(cam_k[0, 2])) * z / float(cam_k[0, 0])
    y = (ys.astype(np.float32) - float(cam_k[1, 2])) * z / float(cam_k[1, 1])
    points = np.stack([x, y, z], axis=1).astype(np.float32)
    if points.shape[0] > max(int(max_points), 128):
        indices = np.random.choice(points.shape[0], int(max_points), replace=False)
        points = points[indices]
    return points


def _render_overlay(*, out_dir: Path, rgb: np.ndarray | None, cam_k: np.ndarray | None, points: np.ndarray) -> str:
    if rgb is None or cam_k is None or points.size == 0:
        return ""
    try:
        import cv2
    except Exception:
        return ""

    canvas = rgb.copy()
    if canvas.dtype != np.uint8:
        canvas = np.clip(canvas, 0, 255).astype(np.uint8)
    height, width = canvas.shape[:2]
    for rank, point in enumerate(points[: min(points.shape[0], 24)]):
        z = float(point[2])
        if z <= 1e-6:
            continue
        u = float(cam_k[0, 0] * point[0] / z + cam_k[0, 2])
        v = float(cam_k[1, 1] * point[1] / z + cam_k[1, 2])
        if not (0 <= u < width and 0 <= v < height):
            continue
        color = (0, int(max(255 - rank * 8, 64)), int(min(rank * 10, 255)))
        radius = 6 if rank == 0 else 4
        cv2.circle(canvas, (int(round(u)), int(round(v))), radius, color, thickness=-1)
    overlay_path = out_dir / "graspgen_overlay.png"
    cv2.imwrite(str(overlay_path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
    return str(overlay_path)


def main() -> int:
    args = build_argparser().parse_args()
    repo_path = Path(args.repo_path).expanduser().resolve()
    npz_path = Path(args.npz_path).expanduser().resolve()
    gripper_config = Path(args.gripper_config).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    _prepare_imports(repo_path)
    _patch_huggingface_hub_compat()

    from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg

    exported = _load_export(npz_path)
    object_pc = _segmented_object_cloud(
        depth=exported["depth"],
        cam_k=exported["cam_k"],
        segmap=exported["segmap"],
        max_points=int(args.max_points),
    )
    grasp_cfg = load_grasp_cfg(str(gripper_config))
    sampler = GraspGenSampler(grasp_cfg)
    grasps, grasp_conf = GraspGenSampler.run_inference(
        object_pc,
        sampler,
        grasp_threshold=float(args.grasp_threshold),
        num_grasps=int(args.num_grasps),
        topk_num_grasps=int(args.topk_num_grasps),
        min_grasps=int(args.min_grasps),
        max_tries=int(args.max_tries),
        remove_outliers=not bool(args.no_remove_outliers),
    )
    if hasattr(grasps, "detach"):
        grasps = grasps.detach()
    if hasattr(grasps, "cpu"):
        grasps = grasps.cpu()
    if hasattr(grasps, "numpy"):
        grasps = grasps.numpy()
    if hasattr(grasp_conf, "detach"):
        grasp_conf = grasp_conf.detach()
    if hasattr(grasp_conf, "cpu"):
        grasp_conf = grasp_conf.cpu()
    if hasattr(grasp_conf, "numpy"):
        grasp_conf = grasp_conf.numpy()

    grasp_mats = np.asarray(grasps, dtype=np.float32).reshape(-1, 4, 4)
    scores = np.asarray(grasp_conf, dtype=np.float32).reshape(-1)
    grasp_count = int(min(grasp_mats.shape[0], scores.shape[0] if scores.size else grasp_mats.shape[0], max(int(args.max_grasps), 1)))
    grasp_mats = grasp_mats[:grasp_count]
    scores = scores[:grasp_count]
    contact_pts = grasp_mats[:, :3, 3] if grasp_count > 0 else np.zeros((0, 3), dtype=np.float32)

    np.savez_compressed(
        out_dir / "segment_0_grasps.npz",
        pred_grasps_cam=grasp_mats,
        scores=scores,
        contact_pts=contact_pts,
    )
    overlay_path = _render_overlay(
        out_dir=out_dir,
        rgb=exported["rgb"],
        cam_k=exported["cam_k"],
        points=contact_pts,
    )
    summary = {
        "ok": True,
        "repo_path": str(repo_path),
        "npz_path": str(npz_path),
        "gripper_config": str(gripper_config),
        "num_grasps": int(args.num_grasps),
        "topk_num_grasps": int(args.topk_num_grasps),
        "grasp_threshold": float(args.grasp_threshold),
        "segmented_object_points": int(object_pc.shape[0]),
        "grasp_group_count": 1 if grasp_count > 0 else 0,
        "grasp_total": int(grasp_count),
        "score_max": None if scores.size == 0 else float(np.max(scores)),
        "score_mean": None if scores.size == 0 else float(np.mean(scores)),
        "overlay_path": overlay_path,
        "remove_outliers": not bool(args.no_remove_outliers),
    }
    summary_path = out_dir / "graspgen_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
