"""Reusable candidate-family helpers for grasp, approach, and release planning.

This module is the first step away from task-local fallback lists and toward
shared planning utilities that multiple tasks can reuse.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class CandidateVariantSpec:
    label: str
    pose: Optional[List[float]]


def quat_normalize(quat: List[float] | np.ndarray) -> List[float]:
    q = np.asarray(quat, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    q = q / norm
    return [float(v) for v in q.tolist()]


def quat_mul(q1: List[float] | np.ndarray, q2: List[float] | np.ndarray) -> List[float]:
    x1, y1, z1, w1 = [float(v) for v in q1]
    x2, y2, z2, w2 = [float(v) for v in q2]
    return quat_normalize(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_from_axis_angle(axis: List[float] | np.ndarray, angle_rad: float) -> List[float]:
    axis_arr = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(axis_arr))
    if norm < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    axis_arr = axis_arr / norm
    half = 0.5 * float(angle_rad)
    sin_half = float(np.sin(half))
    return quat_normalize(
        [
            float(axis_arr[0] * sin_half),
            float(axis_arr[1] * sin_half),
            float(axis_arr[2] * sin_half),
            float(np.cos(half)),
        ]
    )


def build_orientation_variant_specs(arm: str) -> List[tuple[str, List[float], float]]:
    """Build a coarse orientation bank for grasp-family expansion.

    The design is intentionally coarse-first:
    - keep a small always-on local bank
    - add arm-aware larger rotations to escape local orientation minima

    This follows the same spirit as proposal-based grasp systems like
    Contact-GraspNet / GraspNet: generate diverse coarse hypotheses first,
    then let downstream feasibility/ranking prune them.
    """

    specs: List[tuple[str, List[float], float]] = [
        ("ori_base", [0.0, 0.0, 0.0, 1.0], 0.0),
        ("yaw_pos_20", quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(20.0)), -0.03),
        ("yaw_neg_20", quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(-20.0)), -0.03),
        ("pitch_pos_15", quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(15.0)), -0.045),
        ("pitch_neg_15", quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(-15.0)), -0.045),
    ]

    if arm == "left":
        specs.extend(
            [
                ("left_yaw_pos_45", quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(45.0)), -0.075),
                ("left_yaw_neg_45", quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(-45.0)), -0.075),
                ("left_pitch_pos_30", quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(30.0)), -0.085),
                ("left_pitch_neg_30", quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(-30.0)), -0.085),
                ("left_roll_in_35", quat_from_axis_angle([1.0, 0.0, 0.0], np.deg2rad(35.0)), -0.08),
                ("left_roll_out_35", quat_from_axis_angle([1.0, 0.0, 0.0], np.deg2rad(-35.0)), -0.08),
                (
                    "left_yaw_pos_35_pitch_neg_20",
                    quat_mul(
                        quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(35.0)),
                        quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(-20.0)),
                    ),
                    -0.1,
                ),
                (
                    "left_yaw_neg_35_pitch_pos_20",
                    quat_mul(
                        quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(-35.0)),
                        quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(20.0)),
                    ),
                    -0.1,
                ),
            ]
        )
    elif arm == "right":
        specs.extend(
            [
                ("right_yaw_pos_45", quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(45.0)), -0.075),
                ("right_yaw_neg_45", quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(-45.0)), -0.075),
                ("right_pitch_pos_30", quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(30.0)), -0.085),
                ("right_pitch_neg_30", quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(-30.0)), -0.085),
                ("right_roll_in_35", quat_from_axis_angle([1.0, 0.0, 0.0], np.deg2rad(-35.0)), -0.08),
                ("right_roll_out_35", quat_from_axis_angle([1.0, 0.0, 0.0], np.deg2rad(35.0)), -0.08),
                (
                    "right_yaw_pos_35_pitch_pos_20",
                    quat_mul(
                        quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(35.0)),
                        quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(20.0)),
                    ),
                    -0.1,
                ),
                (
                    "right_yaw_neg_35_pitch_neg_20",
                    quat_mul(
                        quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(-35.0)),
                        quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(-20.0)),
                    ),
                    -0.1,
                ),
            ]
        )
    return specs


def _offset_pose(pose: Optional[List[float]], dz: float = 0.0, dx: float = 0.0, dy: float = 0.0) -> Optional[List[float]]:
    if pose is None or len(pose) < 7:
        return None
    shifted = list(pose)
    shifted[0] += float(dx)
    shifted[1] += float(dy)
    shifted[2] += float(dz)
    return shifted


def _blend_pose(
    pose_a: Optional[List[float]],
    pose_b: Optional[List[float]],
    alpha: float,
    extra_dz: float = 0.0,
) -> Optional[List[float]]:
    if pose_a is None or pose_b is None or len(pose_a) < 7 or len(pose_b) < 7:
        return None
    alpha = float(alpha)
    blended = list(pose_a)
    for axis in range(3):
        blended[axis] = float((1.0 - alpha) * float(pose_a[axis]) + alpha * float(pose_b[axis]))
    blended[2] += float(extra_dz)
    blended[3:7] = list(pose_b[3:7])
    return blended


def build_arm_aware_release_candidates(
    *,
    base_target: Optional[List[float]],
    approach_pose: Optional[List[float]],
    active_arm: str,
) -> List[CandidateVariantSpec]:
    arm_sign = -1.0 if active_arm == "left" else 1.0
    return [
        CandidateVariantSpec("primary", base_target),
        CandidateVariantSpec("softened_release", _offset_pose(base_target, dz=0.02)),
        CandidateVariantSpec("midpoint", _offset_pose(base_target, dz=0.01)),
        CandidateVariantSpec("approach_to_release_35", _blend_pose(approach_pose, base_target, 0.35, extra_dz=0.05)),
        CandidateVariantSpec("approach_to_release_60", _blend_pose(approach_pose, base_target, 0.60, extra_dz=0.03)),
        CandidateVariantSpec("target_high_clearance", _offset_pose(base_target, dz=0.06)),
        CandidateVariantSpec(
            "approach_xy_target_z",
            None if approach_pose is None or base_target is None else [approach_pose[0], approach_pose[1], base_target[2] + 0.04, *list(base_target[3:7])],
        ),
        CandidateVariantSpec("arm_side_clear", _offset_pose(base_target, dx=0.03 * arm_sign, dy=0.04 * arm_sign, dz=0.05)),
        CandidateVariantSpec("arm_side_high", _offset_pose(base_target, dx=0.05 * arm_sign, dy=0.06 * arm_sign, dz=0.09)),
        CandidateVariantSpec(
            "release_side_backoff",
            _offset_pose(base_target, dx=0.025 * arm_sign, dy=0.05 * arm_sign, dz=0.11),
        ),
        CandidateVariantSpec("approach_hold", approach_pose),
    ]


def build_blended_release_candidates(
    *,
    base_target: Optional[List[float]],
    release_pose: Optional[List[float]],
    approach_pose: Optional[List[float]],
    active_arm: str,
) -> List[CandidateVariantSpec]:
    arm_sign = -1.0 if active_arm == "left" else 1.0
    return [
        CandidateVariantSpec("primary", base_target),
        CandidateVariantSpec("high_clearance", _offset_pose(base_target, dz=0.04)),
        CandidateVariantSpec("release_backoff", _offset_pose(release_pose, dz=0.08) if release_pose else None),
        CandidateVariantSpec("release_high_clearance", _offset_pose(release_pose, dz=0.12) if release_pose else None),
        CandidateVariantSpec("arm_side_clear", _offset_pose(base_target, dx=0.03 * arm_sign, dy=0.04 * arm_sign, dz=0.05)),
        CandidateVariantSpec("arm_side_high", _offset_pose(base_target, dx=0.05 * arm_sign, dy=0.06 * arm_sign, dz=0.09)),
        CandidateVariantSpec(
            "release_side_backoff",
            _offset_pose(release_pose, dx=0.025 * arm_sign, dy=0.05 * arm_sign, dz=0.11) if release_pose else None,
        ),
        CandidateVariantSpec("approach_hold", approach_pose),
    ]


def build_grasp_candidate_variants(
    *,
    candidate: Dict[str, Any],
    object_pose: Optional[List[float]] = None,
    current_eef: Optional[List[float]] = None,
) -> List[Dict[str, Any]]:
    pose = list(candidate.get("pose") or [])
    if len(pose) < 7:
        return []
    pregrasp = list(candidate.get("pregrasp_pose") or [])
    if len(pregrasp) < 7:
        pregrasp = list(pose)
        pregrasp[2] += 0.08

    grasp_xyz = np.asarray(pose[:3], dtype=np.float64)
    pregrasp_xyz = np.asarray(pregrasp[:3], dtype=np.float64)
    approach_vec = pregrasp_xyz - grasp_xyz
    approach_norm = float(np.linalg.norm(approach_vec))
    if approach_norm < 1e-6:
        approach_dir = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        approach_dir = approach_vec / approach_norm
    lateral = np.cross(approach_dir, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm < 1e-6:
        lateral = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        lateral = lateral / lateral_norm

    arm = str(candidate.get("arm") or "right")
    specs = [("base", np.zeros(3), np.zeros(3), 1.0)]
    if current_eef is not None and len(current_eef) >= 3:
        current_xyz = np.asarray(current_eef[:3], dtype=np.float64)
        reach_shift = np.clip((current_xyz - pregrasp_xyz) * 0.28, -0.04, 0.04)
        specs.append(("reach_relief", np.zeros(3), reach_shift + np.asarray([0.0, 0.0, 0.015]), 0.985))
    if object_pose is not None and len(object_pose) >= 3:
        object_xyz = np.asarray(object_pose[:3], dtype=np.float64)
        recenter = np.clip(object_xyz - grasp_xyz, -0.025, 0.025)
        specs.append(("vision_recenter", recenter, recenter + np.asarray([0.0, 0.0, 0.02]), 0.965))
        object_grasp_height = float(object_xyz[2]) + 0.085
        object_grasp_center = np.asarray([float(object_xyz[0]), float(object_xyz[1]), object_grasp_height], dtype=np.float64)
        if current_eef is not None and len(current_eef) >= 3:
            current_xyz = np.asarray(current_eef[:3], dtype=np.float64)
            object_to_current = current_xyz - object_grasp_center
            horizontal = np.asarray([object_to_current[0], object_to_current[1], 0.0], dtype=np.float64)
            horizontal_norm = float(np.linalg.norm(horizontal))
            if horizontal_norm < 1e-6:
                arm_sign = -1.0 if arm == "left" else 1.0
                horizontal_dir = np.asarray([-0.6, -0.8 * arm_sign, 0.0], dtype=np.float64)
            else:
                horizontal_dir = horizontal / horizontal_norm
            tangent = np.cross(np.asarray([0.0, 0.0, 1.0], dtype=np.float64), horizontal_dir)
            tangent_norm = float(np.linalg.norm(tangent))
            if tangent_norm < 1e-6:
                tangent = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
            else:
                tangent = tangent / tangent_norm
            arm_sign = -1.0 if arm == "left" else 1.0
            inside_sign = -arm_sign

            def _object_anchor(grasp_offset, pregrasp_offset, score):
                grasp_target = object_grasp_center + grasp_offset
                pregrasp_target = object_grasp_center + pregrasp_offset
                return (
                    np.clip(grasp_target - grasp_xyz, -0.08, 0.08),
                    np.clip(pregrasp_target - pregrasp_xyz, -0.16, 0.16),
                    score,
                )

            center_grasp, center_pre, center_score = _object_anchor(
                horizontal_dir * 0.012,
                horizontal_dir * 0.085 + tangent * (0.01 * inside_sign) + np.asarray([0.0, 0.0, 0.1]),
                0.972,
            )
            specs.append(("object_current_lane", center_grasp, center_pre, center_score))

            sweep_grasp, sweep_pre, sweep_score = _object_anchor(
                horizontal_dir * 0.008 + tangent * (0.014 * inside_sign),
                horizontal_dir * 0.072 + tangent * (0.038 * inside_sign) + np.asarray([0.0, 0.0, 0.11]),
                0.962,
            )
            specs.append(("object_inside_sweep", sweep_grasp, sweep_pre, sweep_score))

            arc_grasp, arc_pre, arc_score = _object_anchor(
                horizontal_dir * 0.014 - tangent * (0.012 * inside_sign),
                horizontal_dir * 0.06 - tangent * (0.055 * inside_sign) + np.asarray([0.0, 0.0, 0.12]),
                0.956,
            )
            specs.append(("object_arc_entry", arc_grasp, arc_pre, arc_score))
    specs.extend(
        [
            ("pregrasp_high", np.zeros(3), np.asarray([0.0, 0.0, 0.04]), 0.95),
            ("lateral_pos", lateral * 0.012, lateral * 0.025 + np.asarray([0.0, 0.0, 0.02]), 0.92),
            ("lateral_neg", -lateral * 0.012, -lateral * 0.025 + np.asarray([0.0, 0.0, 0.02]), 0.91),
            ("pregrasp_backoff", np.zeros(3), approach_dir * 0.035 + np.asarray([0.0, 0.0, 0.015]), 0.88),
        ]
    )
    if arm == "left":
        specs.extend(
            [
                ("left_short_backoff", np.zeros(3), approach_dir * 0.015 + np.asarray([0.025, 0.0, 0.03]), 0.975),
                ("left_cross_y_pos", np.asarray([0.012, 0.018, 0.0]), np.asarray([0.03, 0.05, 0.035]), 0.955),
                ("left_cross_y_neg", np.asarray([0.012, -0.018, 0.0]), np.asarray([0.03, -0.05, 0.035]), 0.95),
                ("left_vertical_clear", np.asarray([0.01, 0.0, 0.0]), np.asarray([0.035, 0.0, 0.07]), 0.945),
            ]
        )
    elif arm == "right":
        specs.extend(
            [
                ("right_short_backoff", np.zeros(3), approach_dir * 0.015 + np.asarray([-0.025, 0.0, 0.03]), 0.975),
                ("right_cross_y_pos", np.asarray([-0.012, 0.018, 0.0]), np.asarray([-0.03, 0.05, 0.035]), 0.955),
                ("right_cross_y_neg", np.asarray([-0.012, -0.018, 0.0]), np.asarray([-0.03, -0.05, 0.035]), 0.95),
                ("right_vertical_clear", np.asarray([-0.01, 0.0, 0.0]), np.asarray([-0.035, 0.0, 0.07]), 0.945),
            ]
        )

    orientation_specs = build_orientation_variant_specs(arm)

    variants: List[Dict[str, Any]] = []
    for label, grasp_delta, pregrasp_delta, base_score in specs:
        for ori_label, ori_delta, ori_penalty in orientation_specs:
            variant = dict(candidate)
            variant_pose = list(pose)
            variant_pregrasp = list(pregrasp)
            for axis in range(3):
                variant_pose[axis] = float(grasp_xyz[axis] + grasp_delta[axis])
                variant_pregrasp[axis] = float(pregrasp_xyz[axis] + pregrasp_delta[axis])
            variant_pose[3:7] = quat_mul(variant_pose[3:7], ori_delta)
            variant_pregrasp[3:7] = quat_mul(variant_pregrasp[3:7], ori_delta)
            variant["pose"] = variant_pose
            variant["pregrasp_pose"] = variant_pregrasp
            variant["variant_label"] = label if ori_label == "ori_base" else f"{label}_{ori_label}"
            variant["score"] = float(base_score + ori_penalty)
            if current_eef is not None and len(current_eef) >= 3:
                dist = float(
                    np.linalg.norm(
                        np.asarray(current_eef[:3], dtype=np.float64)
                        - np.asarray(variant_pregrasp[:3], dtype=np.float64)
                    )
                )
                variant["score"] -= min(dist, 1.0) * 0.03
            variants.append(variant)
    return variants


def build_synthesized_grasp_candidates(
    *,
    object_pose: Optional[List[float]],
    arm: str,
    orientation: Optional[List[float]] = None,
) -> List[Dict[str, Any]]:
    if object_pose is None or len(object_pose) < 3:
        return []
    obj = np.asarray(object_pose[:3], dtype=np.float64)
    base_height = float(obj[2]) + 0.085
    pre_height = base_height + 0.055
    orientation = quat_normalize(orientation or [0.9949492141138409, 0.0001428575322812901, 0.10037948350189996, 1.4760360441018511e-05])
    orientation_bank = [
        ("ori_base", orientation, 0.0),
        ("ori_yaw_pos_25", quat_mul(orientation, quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(25.0))), -0.025),
        ("ori_yaw_neg_25", quat_mul(orientation, quat_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(-25.0))), -0.025),
        ("ori_pitch_pos_18", quat_mul(orientation, quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(18.0))), -0.04),
        ("ori_pitch_neg_18", quat_mul(orientation, quat_from_axis_angle([0.0, 1.0, 0.0], np.deg2rad(-18.0))), -0.04),
    ]
    offsets = [
        ("synth_center", np.asarray([0.0, 0.0, 0.0])),
        ("synth_back", np.asarray([-0.02, 0.0, 0.0])),
        ("synth_left", np.asarray([0.0, -0.02, 0.0])),
        ("synth_right", np.asarray([0.0, 0.02, 0.0])),
        ("synth_high", np.asarray([0.0, 0.0, 0.02])),
    ]
    if arm == "left":
        offsets.extend(
            [
                ("synth_left_arm_cross", np.asarray([0.015, 0.045, 0.0])),
                ("synth_left_arm_short", np.asarray([0.03, 0.0, 0.015])),
            ]
        )
    elif arm == "right":
        offsets.extend(
            [
                ("synth_right_arm_cross", np.asarray([-0.015, -0.045, 0.0])),
                ("synth_right_arm_short", np.asarray([-0.03, 0.0, 0.015])),
            ]
        )

    candidates: List[Dict[str, Any]] = []
    rank = 0
    for label, offset in offsets:
        grasp_xyz = obj + offset
        for ori_label, ori_quat, ori_bonus in orientation_bank:
            grasp_shift = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
            pregrasp_shift = np.asarray([-0.09, 0.0, 0.05], dtype=np.float64)
            if arm == "left":
                pregrasp_shift = np.asarray([-0.06, 0.0, 0.05], dtype=np.float64)
            elif arm == "right":
                pregrasp_shift = np.asarray([0.06, 0.0, 0.05], dtype=np.float64)
            if "pitch_pos" in ori_label:
                pregrasp_shift = pregrasp_shift + np.asarray([0.015, -0.01, 0.025], dtype=np.float64)
            elif "pitch_neg" in ori_label:
                pregrasp_shift = pregrasp_shift + np.asarray([-0.02, 0.01, -0.02], dtype=np.float64)
            elif "yaw_pos" in ori_label:
                grasp_shift = np.asarray([0.0, -0.01 if arm == "left" else 0.01, 0.0], dtype=np.float64)
                pregrasp_shift = pregrasp_shift + np.asarray([0.0, -0.03 if arm == "left" else 0.03, 0.0], dtype=np.float64)
            elif "yaw_neg" in ori_label:
                grasp_shift = np.asarray([0.0, 0.01 if arm == "left" else -0.01, 0.0], dtype=np.float64)
                pregrasp_shift = pregrasp_shift + np.asarray([0.0, 0.03 if arm == "left" else -0.03, 0.0], dtype=np.float64)

            grasp_xyz_variant = grasp_xyz + grasp_shift
            pregrasp_xyz = grasp_xyz_variant + pregrasp_shift
            pose = [float(grasp_xyz_variant[0]), float(grasp_xyz_variant[1]), float(base_height + offset[2]), *ori_quat]
            pregrasp = [float(pregrasp_xyz[0]), float(pregrasp_xyz[1]), float(pre_height + offset[2]), *ori_quat]
            candidates.append(
                {
                    "pose": pose,
                    "pregrasp_pose": pregrasp,
                    "arm": arm,
                    "score": 0.76 - rank * 0.015 + ori_bonus,
                    "variant_label": f"{label}_{ori_label}",
                    "rank": rank,
                    "source_index": -1,
                }
            )
            rank += 1
    return candidates
