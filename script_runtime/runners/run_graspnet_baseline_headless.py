"""Headless GraspNet Baseline runner for exported RoboTwin snapshots."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run GraspNet Baseline headlessly on an exported NPZ snapshot.")
    parser.add_argument("--repo-path", required=True, help="Path to third_party/graspnet-baseline.")
    parser.add_argument("--npz-path", required=True, help="Input NPZ exported from script_runtime.")
    parser.add_argument("--checkpoint-path", required=True, help="Checkpoint tar path for GraspNet Baseline.")
    parser.add_argument("--out-dir", required=True, help="Output directory for summaries and raw predictions.")
    parser.add_argument("--num-point", type=int, default=20000, help="Number of masked scene points to sample.")
    parser.add_argument("--max-grasps", type=int, default=60, help="Maximum grasps to keep after ranking.")
    parser.add_argument("--collision-thresh", type=float, default=-1.0, help="Collision threshold. Negative disables collision filtering.")
    parser.add_argument("--voxel-size", type=float, default=0.01, help="Voxel size for collision filtering.")
    return parser


def _prepare_imports(repo_path: Path) -> None:
    sys.path.insert(0, str(repo_path))
    sys.path.insert(0, str(repo_path / "models"))
    sys.path.insert(0, str(repo_path / "dataset"))
    sys.path.insert(0, str(repo_path / "utils"))


def _load_export(npz_path: Path) -> Dict[str, Any]:
    payload = np.load(npz_path)
    rgb = np.asarray(payload["rgb"]) if "rgb" in payload.files else None
    depth = np.asarray(payload["depth"], dtype=np.float32) if "depth" in payload.files else None
    cam_k = np.asarray(payload["K"], dtype=np.float32) if "K" in payload.files else None
    segmap = np.asarray(payload["segmap"], dtype=np.uint8) if "segmap" in payload.files else None
    return {"rgb": rgb, "depth": depth, "cam_k": cam_k, "segmap": segmap}


def _build_masked_cloud(
    *,
    rgb: np.ndarray | None,
    depth: np.ndarray | None,
    cam_k: np.ndarray | None,
    segmap: np.ndarray | None,
    num_point: int,
) -> Tuple[Dict[str, Any], np.ndarray]:
    import torch
    from data_utils import CameraInfo, create_point_cloud_from_depth_image

    if rgb is None or depth is None or cam_k is None or segmap is None:
        raise RuntimeError("input_missing_rgb_depth_intrinsics_or_segmap")

    mask = (segmap > 0) & (depth > 0)
    if not np.any(mask):
        raise RuntimeError("segmap_empty")

    color = rgb.astype(np.float32) / 255.0
    camera = CameraInfo(
        width=float(depth.shape[1]),
        height=float(depth.shape[0]),
        fx=float(cam_k[0, 0]),
        fy=float(cam_k[1, 1]),
        cx=float(cam_k[0, 2]),
        cy=float(cam_k[1, 2]),
        scale=1.0,
    )
    cloud = create_point_cloud_from_depth_image(depth.astype(np.float32), camera, organized=True)
    cloud_masked = cloud[mask]
    color_masked = color[mask]
    if cloud_masked.shape[0] == 0:
        raise RuntimeError("masked_cloud_empty")

    if cloud_masked.shape[0] >= num_point:
        indices = np.random.choice(cloud_masked.shape[0], num_point, replace=False)
    else:
        indices_a = np.arange(cloud_masked.shape[0])
        indices_b = np.random.choice(cloud_masked.shape[0], num_point - cloud_masked.shape[0], replace=True)
        indices = np.concatenate([indices_a, indices_b], axis=0)

    cloud_sampled = torch.from_numpy(cloud_masked[indices][np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    end_points = {
        "point_clouds": cloud_sampled.to(device),
        "cloud_colors": color_masked[indices],
    }
    return end_points, cloud_masked.astype(np.float32)


def _load_network(checkpoint_path: Path):
    import torch
    from graspnet import GraspNet

    net = GraspNet(
        input_feature_dim=0,
        num_view=300,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    )
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    net.load_state_dict(state_dict)
    net.eval()
    return net


def _grasp_group_to_payload(gg: Any, *, max_grasps: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if hasattr(gg, "nms"):
        gg.nms()
    if hasattr(gg, "sort_by_score"):
        gg.sort_by_score()
    gg = gg[: max(int(max_grasps), 1)]

    rotations = np.asarray(getattr(gg, "rotation_matrices", []), dtype=np.float32).reshape(-1, 3, 3)
    translations = np.asarray(getattr(gg, "translations", []), dtype=np.float32).reshape(-1, 3)
    scores = np.asarray(getattr(gg, "scores", []), dtype=np.float32).reshape(-1)
    if rotations.size == 0 or translations.size == 0:
        grasp_array = np.asarray(
            getattr(gg, "grasp_group_array", getattr(gg, "grasp_array", [])),
            dtype=np.float32,
        )
        if grasp_array.size:
            grasp_array = grasp_array.reshape(-1, grasp_array.shape[-1])
            scores = grasp_array[:, 0]
            rotations = grasp_array[:, 4:13].reshape(-1, 3, 3)
            translations = grasp_array[:, 13:16]

    grasp_count = int(min(rotations.shape[0], translations.shape[0], scores.shape[0] if scores.size else rotations.shape[0]))
    grasp_mats = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], grasp_count, axis=0)
    if grasp_count > 0:
        grasp_mats[:, :3, :3] = rotations[:grasp_count]
        grasp_mats[:, :3, 3] = translations[:grasp_count]
    return grasp_mats, scores[:grasp_count], translations[:grasp_count]


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
    overlay_path = out_dir / "graspnet_baseline_overlay.png"
    cv2.imwrite(str(overlay_path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
    return str(overlay_path)


def main() -> int:
    args = build_argparser().parse_args()
    repo_path = Path(args.repo_path).expanduser().resolve()
    npz_path = Path(args.npz_path).expanduser().resolve()
    checkpoint_path = Path(args.checkpoint_path).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    _prepare_imports(repo_path)

    import torch
    from graspnet import pred_decode
    from graspnetAPI import GraspGroup
    from collision_detector import ModelFreeCollisionDetector

    exported = _load_export(npz_path)
    end_points, scene_cloud = _build_masked_cloud(
        rgb=exported["rgb"],
        depth=exported["depth"],
        cam_k=exported["cam_k"],
        segmap=exported["segmap"],
        num_point=max(int(args.num_point), 1024),
    )
    net = _load_network(checkpoint_path)
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg = GraspGroup(grasp_preds[0].detach().cpu().numpy())
    collision_enabled = float(args.collision_thresh) >= 0.0
    if collision_enabled:
        detector = ModelFreeCollisionDetector(scene_cloud, voxel_size=float(args.voxel_size))
        collision_mask = detector.detect(gg, approach_dist=0.05, collision_thresh=float(args.collision_thresh))
        gg = gg[~collision_mask]

    grasp_mats, scores, contact_pts = _grasp_group_to_payload(gg, max_grasps=int(args.max_grasps))
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
        "checkpoint_path": str(checkpoint_path),
        "num_point": int(args.num_point),
        "collision_thresh": float(args.collision_thresh),
        "voxel_size": float(args.voxel_size),
        "masked_scene_points": int(scene_cloud.shape[0]),
        "grasp_group_count": 1 if grasp_mats.shape[0] > 0 else 0,
        "grasp_total": int(grasp_mats.shape[0]),
        "score_max": None if scores.size == 0 else float(np.max(scores)),
        "score_mean": None if scores.size == 0 else float(np.mean(scores)),
        "overlay_path": overlay_path,
    }
    summary_path = out_dir / "graspnet_baseline_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
