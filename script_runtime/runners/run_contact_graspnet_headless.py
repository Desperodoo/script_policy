"""Headless Contact-GraspNet runner for exported RoboTwin snapshots.

This script is meant to be executed inside a TensorFlow-capable environment.
It bypasses the official GUI visualization entrypoint and directly calls the
core inference stack, saving compact diagnostics and raw prediction files.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run Contact-GraspNet headlessly on an exported NPZ snapshot.")
    parser.add_argument("--repo-path", required=True, help="Path to third_party/contact_graspnet.")
    parser.add_argument("--npz-path", required=True, help="Input NPZ exported from script_runtime.")
    parser.add_argument("--ckpt-dir", required=True, help="Checkpoint directory with config.yaml and model.ckpt-* files.")
    parser.add_argument("--out-dir", required=True, help="Output directory for summaries and raw predictions.")
    parser.add_argument("--forward-passes", type=int, default=1, help="Number of forward passes.")
    parser.add_argument("--z-range", default="[0.2,1.8]", help="Depth crop range in meters.")
    parser.add_argument("--local-regions", action="store_true", help="Crop local object regions before inference.")
    parser.add_argument("--filter-grasps", action="store_true", help="Filter grasps to segmented object surfaces.")
    parser.add_argument("--skip-border-objects", action="store_true", help="Skip segments touching image border.")
    parser.add_argument("--segmap-id", type=int, default=0, help="Optional single object id.")
    parser.add_argument("--arg-configs", nargs="*", default=[], help="Optional config overrides.")
    return parser


def _prepare_imports(repo_path: Path) -> None:
    contact_dir = repo_path / "contact_graspnet"
    sys.path.insert(0, str(contact_dir))
    sys.path.insert(0, str(repo_path))
    sys.path.insert(0, str(repo_path / "pointnet2" / "utils"))
    sys.path.insert(0, str(repo_path / "pointnet2" / "tf_ops" / "grouping"))


def _summarize_group(*, segment_id: Any, grasps: Any, scores: Any, contact_pts: Any, gripper_openings: Any) -> Dict[str, Any]:
    grasp_array = np.asarray(grasps)
    score_array = np.asarray(scores).reshape(-1)
    contact_array = np.asarray(contact_pts)
    opening_array = np.asarray(gripper_openings).reshape(-1)
    grasp_count = int(grasp_array.shape[0]) if grasp_array.ndim >= 3 else 0
    first_translation: List[float] = []
    first_contact: List[float] = []
    if grasp_count > 0:
        first_translation = [float(v) for v in grasp_array[0, :3, 3].tolist()]
    if contact_array.size >= 3:
        contact_array = np.asarray(contact_array).reshape(-1, 3)
        first_contact = [float(v) for v in contact_array[0].tolist()]
    return {
        "segment_id": int(segment_id),
        "grasp_count": grasp_count,
        "score_max": None if score_array.size == 0 else float(np.max(score_array)),
        "score_mean": None if score_array.size == 0 else float(np.mean(score_array)),
        "opening_mean": None if opening_array.size == 0 else float(np.mean(opening_array)),
        "first_grasp_translation": first_translation,
        "first_contact_point": first_contact,
    }


def _render_contact_overlay(*, out_dir: Path, rgb: Any, cam_k: Any, contact_pts: Any, scores: Any, segmap: Any) -> str:
    if rgb is None or cam_k is None or contact_pts is None:
        return ""
    try:
        import cv2
    except Exception:
        return ""

    rgb_array = np.asarray(rgb)
    if rgb_array.ndim != 3:
        return ""
    if rgb_array.dtype != np.uint8:
        rgb_array = np.clip(rgb_array, 0, 255).astype(np.uint8)
    canvas = rgb_array.copy()
    contact_array = np.asarray(contact_pts, dtype=np.float32).reshape(-1, 3)
    score_array = np.asarray(scores, dtype=np.float32).reshape(-1)
    if contact_array.size == 0:
        return ""
    cam_k = np.asarray(cam_k, dtype=np.float32).reshape(3, 3)
    order = np.argsort(score_array)[::-1]
    top_order = order[: min(len(order), 24)]
    if segmap is not None:
        seg = np.asarray(segmap)
        mask = seg > 0
        if mask.any():
            canvas[mask] = (0.75 * canvas[mask] + 0.25 * np.array([66, 133, 244], dtype=np.float32)).astype(np.uint8)
    height, width = canvas.shape[:2]
    for rank, idx in enumerate(top_order):
        point = contact_array[idx]
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
        if rank < 5:
            cv2.putText(
                canvas,
                f"{rank}:{score_array[idx]:.2f}",
                (int(round(u)) + 4, int(round(v)) - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                color,
                1,
                cv2.LINE_AA,
            )
    overlay_path = out_dir / "contact_graspnet_overlay.png"
    cv2.imwrite(str(overlay_path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
    return str(overlay_path)


def main() -> int:
    args = build_argparser().parse_args()
    repo_path = Path(args.repo_path).expanduser().resolve()
    npz_path = Path(args.npz_path).expanduser().resolve()
    ckpt_dir = Path(args.ckpt_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    _prepare_imports(repo_path)

    import tensorflow.compat.v1 as tf

    tf.disable_eager_execution()
    try:
        physical_devices = tf.config.experimental.list_physical_devices("GPU")
        if physical_devices:
            tf.config.experimental.set_memory_growth(physical_devices[0], True)
    except Exception:
        pass

    import config_utils
    from contact_grasp_estimator import GraspEstimator
    from data import load_available_input_data

    global_config = config_utils.load_config(
        str(ckpt_dir),
        batch_size=max(int(args.forward_passes), 1),
        arg_configs=list(args.arg_configs or []),
    )
    grasp_estimator = GraspEstimator(global_config)
    grasp_estimator.build_network()
    saver = tf.train.Saver(save_relative_paths=True)

    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    tf_config.allow_soft_placement = True
    sess = tf.Session(config=tf_config)
    grasp_estimator.load_weights(sess, saver, str(ckpt_dir), mode="test")

    segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(str(npz_path))
    if pc_full is None:
        if depth is None or cam_K is None:
            raise RuntimeError("input_missing_depth_or_intrinsics")
        pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
            depth,
            cam_K,
            segmap=segmap,
            rgb=rgb,
            z_range=eval(str(args.z_range)),
            segmap_id=int(args.segmap_id),
            skip_border_objects=bool(args.skip_border_objects),
        )
    else:
        pc_segments = {}

    pred_grasps_cam, scores, contact_pts, gripper_openings = grasp_estimator.predict_scene_grasps(
        sess,
        pc_full,
        pc_segments=pc_segments,
        local_regions=bool(args.local_regions),
        filter_grasps=bool(args.filter_grasps),
        forward_passes=max(int(args.forward_passes), 1),
    )

    group_summaries: List[Dict[str, Any]] = []
    raw_outputs: Dict[str, Any] = {}
    for segment_id in sorted(pred_grasps_cam.keys()):
        group_summaries.append(
            _summarize_group(
                segment_id=segment_id,
                grasps=pred_grasps_cam.get(segment_id),
                scores=scores.get(segment_id),
                contact_pts=contact_pts.get(segment_id),
                gripper_openings=gripper_openings.get(segment_id),
            )
        )
        raw_outputs[str(segment_id)] = {
            "pred_grasps_cam": np.asarray(pred_grasps_cam.get(segment_id)),
            "scores": np.asarray(scores.get(segment_id)),
            "contact_pts": np.asarray(contact_pts.get(segment_id)),
            "gripper_openings": np.asarray(gripper_openings.get(segment_id)),
        }

    for segment_key, payload in raw_outputs.items():
        np.savez_compressed(
            out_dir / f"segment_{segment_key}_grasps.npz",
            **payload,
        )

    overlay_path = ""
    if group_summaries:
        first_key = sorted(pred_grasps_cam.keys())[0]
        overlay_path = _render_contact_overlay(
            out_dir=out_dir,
            rgb=rgb,
            cam_k=cam_K,
            contact_pts=contact_pts.get(first_key),
            scores=scores.get(first_key),
            segmap=segmap,
        )

    summary = {
        "ok": True,
        "repo_path": str(repo_path),
        "npz_path": str(npz_path),
        "ckpt_dir": str(ckpt_dir),
        "forward_passes": max(int(args.forward_passes), 1),
        "local_regions": bool(args.local_regions),
        "filter_grasps": bool(args.filter_grasps),
        "pc_full_points": int(np.asarray(pc_full).shape[0]) if pc_full is not None else 0,
        "pc_segment_ids": [int(key) for key in sorted(pc_segments.keys())],
        "pc_segment_sizes": {str(key): int(np.asarray(value).shape[0]) for key, value in pc_segments.items()},
        "grasp_group_count": len(group_summaries),
        "grasp_total": int(sum(item["grasp_count"] for item in group_summaries)),
        "groups": group_summaries,
        "overlay_path": overlay_path,
    }
    summary_path = out_dir / "contact_graspnet_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
