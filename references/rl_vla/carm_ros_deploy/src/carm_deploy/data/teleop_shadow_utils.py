#!/usr/bin/env python3
"""
Utilities for teleop shadow validation.
"""

from __future__ import annotations

import math
import importlib.util
import os
from typing import Any, Dict, Optional

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CARM_DEPLOY_ROOT = os.path.dirname(_THIS_DIR)
_RL_VLA_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_CARM_DEPLOY_ROOT)))
_POSE_UTILS_PATH = os.path.join(_RL_VLA_ROOT, "rlft", "utils", "pose_utils.py")
_POSE_UTILS_SPEC = importlib.util.spec_from_file_location("teleop_shadow_pose_utils", _POSE_UTILS_PATH)
if _POSE_UTILS_SPEC is None or _POSE_UTILS_SPEC.loader is None:
    raise ImportError(f"Unable to load pose_utils from {_POSE_UTILS_PATH}")
_POSE_UTILS_MODULE = importlib.util.module_from_spec(_POSE_UTILS_SPEC)
_POSE_UTILS_SPEC.loader.exec_module(_POSE_UTILS_MODULE)

apply_relative_transform = _POSE_UTILS_MODULE.apply_relative_transform
compute_relative_pose_transform = _POSE_UTILS_MODULE.compute_relative_pose_transform


def build_hold_target_chunk(
    target_pose_abs: np.ndarray,
    pred_horizon: int,
) -> np.ndarray:
    """Build a constant absolute target chunk with fixed horizon."""
    if pred_horizon <= 0:
        raise ValueError("pred_horizon must be positive")
    target_pose_abs = np.asarray(target_pose_abs, dtype=np.float64)
    if target_pose_abs.shape[-1] != 8:
        raise ValueError("target_pose_abs must have shape (8,)")
    return np.repeat(target_pose_abs[None, :], pred_horizon, axis=0)


def compute_learning_level_chunk(
    ref_pose: np.ndarray,
    target_pose_abs: np.ndarray,
    pred_horizon: int,
) -> Dict[str, Any]:
    """
    Build learning-level human chunks from an absolute teleop target.

    Returns a dict with:
    - human_chunk_abs: [pred_horizon, 8]
    - human_chunk_rel: [pred_horizon, 7]
    - reconstructed_target_abs: [pred_horizon, 8]
    - abs_reconstruction_pos_error
    - abs_reconstruction_rot_error
    """
    ref_pose = np.asarray(ref_pose, dtype=np.float64)
    if ref_pose.shape[-1] != 7:
        raise ValueError("ref_pose must have shape (7,)")

    target_pose_abs = np.asarray(target_pose_abs, dtype=np.float64)
    if target_pose_abs.shape[-1] != 8:
        raise ValueError("target_pose_abs must have shape (8,)")

    human_chunk_abs = build_hold_target_chunk(target_pose_abs, pred_horizon)
    human_chunk_rel = np.zeros((pred_horizon, 7), dtype=np.float64)
    reconstructed = np.zeros((pred_horizon, 8), dtype=np.float64)

    for i in range(pred_horizon):
        pose_abs = human_chunk_abs[i]
        rel = compute_relative_pose_transform(ref_pose, pose_abs[:7])
        human_chunk_rel[i] = rel
        reconstructed[i] = apply_relative_transform(rel, ref_pose, pose_abs[7])

    pos_error = float(np.linalg.norm(reconstructed[0, :3] - target_pose_abs[:3]))
    rot_error = quaternion_angle_distance(reconstructed[0, 3:7], target_pose_abs[3:7])

    return {
        "human_chunk_abs": human_chunk_abs,
        "human_chunk_rel": human_chunk_rel,
        "reconstructed_target_abs": reconstructed,
        "abs_reconstruction_pos_error": pos_error,
        "abs_reconstruction_rot_error": rot_error,
    }


def quaternion_angle_distance(q0: np.ndarray, q1: np.ndarray) -> float:
    """Angular distance between two quaternions in radians."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.clip(np.abs(np.dot(q0, q1)), -1.0, 1.0))
    return float(2.0 * math.acos(dot))


def extract_processed_target_abs(teleop_state_v2: Optional[dict]) -> Optional[np.ndarray]:
    """Extract [x,y,z,qx,qy,qz,qw,gripper] from v2 payload if active."""
    if not teleop_state_v2:
        return None
    processed = teleop_state_v2.get("processed") or {}
    if not processed.get("active"):
        return None
    target_pose_abs = processed.get("target_pose_abs")
    if target_pose_abs is None:
        return None
    gripper_pose = processed.get("gripper_pose", 0.0) or 0.0
    return np.asarray(list(target_pose_abs) + [gripper_pose], dtype=np.float64)


def percentile(values, q: float) -> Optional[float]:
    """Return percentile for non-empty numeric sequences."""
    if not values:
        return None
    arr = np.asarray(values, dtype=np.float64)
    return float(np.percentile(arr, q))
