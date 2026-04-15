"""Perception providers for oracle-assisted and non-oracle runtime integration."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class PerceptionObservation:
    """Minimal perception bundle shared across grounding / pose / grasp providers."""

    rgb: Optional[Any] = None
    depth: Optional[Any] = None
    task_goal: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)


class PerceptionAdapter:
    """Abstract provider for target grounding, pose, place target, and grasp proposals."""

    def get_object_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        return None

    def get_place_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        return None

    def get_place_release_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        return None

    def get_retreat_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        return None

    def get_grasp_candidates(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> Optional[List[Dict[str, Any]]]:
        return None


class NullPerceptionAdapter(PerceptionAdapter):
    """Explicit no-op provider used when no external perception is configured."""


class OraclePerceptionAdapter(PerceptionAdapter):
    """Thin wrapper around simulator / SDK oracles.

    This keeps current runtime behavior unchanged while making oracle usage explicit
    and swappable. Future visual providers should implement the same contract.
    """

    def __init__(self, oracle_backend: Any):
        self.oracle_backend = oracle_backend

    def get_object_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if hasattr(self.oracle_backend, "get_object_pose"):
            return self.oracle_backend.get_object_pose()
        return None

    def get_place_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if hasattr(self.oracle_backend, "get_place_pose"):
            return self.oracle_backend.get_place_pose()
        return None

    def get_place_release_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if hasattr(self.oracle_backend, "get_place_release_pose"):
            return self.oracle_backend.get_place_release_pose()
        return None

    def get_retreat_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if hasattr(self.oracle_backend, "get_retreat_pose"):
            return self.oracle_backend.get_retreat_pose()
        return None

    def get_grasp_candidates(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> Optional[List[Dict[str, Any]]]:
        if hasattr(self.oracle_backend, "get_grasp_candidates"):
            return self.oracle_backend.get_grasp_candidates()
        return None


class RoboTwinDepthPoseProvider(PerceptionAdapter):
    """First non-oracle pose provider using RoboTwin head-camera RGB/depth.

    This provider intentionally starts narrow:
    - estimate object translation from head-camera depth foreground clustering
    - keep orientation simple / unknown
    - optionally fall back to oracle pose when depth inference is unavailable

    It is not the final real-world perception stack, but it moves `GetObjectPose`
    away from directly depending on `actor.get_pose()`.
    """

    def __init__(
        self,
        oracle_backend: Any | None = None,
        use_oracle_fallback: bool = True,
        foreground_offset_mm: float = 18.0,
        min_component_area: int = 120,
    ):
        self.oracle_backend = oracle_backend
        self.use_oracle_fallback = bool(use_oracle_fallback)
        self.foreground_offset_mm = float(foreground_offset_mm)
        self.min_component_area = int(min_component_area)
        self.last_pose_source = "none"
        self.last_grasp_source = "none"
        self.last_component_diagnostics: List[Dict[str, Any]] = []

    def get_object_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        pose = self._estimate_from_depth(observation)
        if pose is not None:
            self.last_pose_source = "depth_centroid"
            return pose
        if self.use_oracle_fallback and self.oracle_backend is not None and hasattr(self.oracle_backend, "get_object_pose"):
            self.last_pose_source = "oracle_fallback"
            return self.oracle_backend.get_object_pose()
        self.last_pose_source = "none"
        return None

    def get_grasp_candidates(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> Optional[List[Dict[str, Any]]]:
        base_candidates = None
        if self.oracle_backend is not None and hasattr(self.oracle_backend, "get_grasp_candidates"):
            base_candidates = self.oracle_backend.get_grasp_candidates()
        object_pose = self._estimate_from_depth(observation)
        if not base_candidates:
            synthesized = self._synthesize_grasp_candidates_from_pose(object_pose=object_pose, context=context)
            if not synthesized:
                self.last_grasp_source = "none"
                return None
            expanded = self._rank_variants_with_reachability(synthesized, context=context)
            self.last_grasp_source = "depth_synthesized"
            return expanded

        expanded = self._expand_grasp_candidates(base_candidates, object_pose=object_pose, context=context)
        self.last_grasp_source = "oracle_augmented" if len(expanded) > len(base_candidates) else "oracle_base"
        return expanded

    def _estimate_from_depth(self, observation: PerceptionObservation) -> Optional[List[float]]:
        depth = observation.depth
        metadata = observation.metadata or {}
        camera_params = metadata.get("camera_params")
        if depth is None or camera_params is None:
            return None

        depth = np.asarray(depth, dtype=np.float64)
        if depth.ndim != 2:
            return None
        valid = np.isfinite(depth) & (depth > 1.0)
        if valid.sum() < self.min_component_area:
            return None

        table_depth = float(np.percentile(depth[valid], 68.0))
        foreground = self._build_foreground_mask(depth, valid, table_depth)
        foreground = self._clean_foreground_mask(foreground)
        if foreground.sum() < self.min_component_area:
            return None

        diagnostics = self._compute_component_diagnostics(
            foreground,
            depth,
            table_depth=table_depth,
            camera_params=camera_params,
        )
        self.last_component_diagnostics = diagnostics
        component_mask = None
        for row in diagnostics:
            if row.get("selected", False):
                component_mask = row.get("mask")
                break
        if component_mask is None:
            return None

        world_point = self._mask_centroid_to_world(component_mask, depth, camera_params)
        if world_point is None:
            return None
        return [float(world_point[0]), float(world_point[1]), float(world_point[2]), 0.0, 0.0, 0.0, 1.0]

    def _clean_foreground_mask(self, foreground: np.ndarray) -> np.ndarray:
        import cv2

        mask = foreground.astype(np.uint8) * 255
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask > 0

    def _build_foreground_mask(
        self,
        depth: np.ndarray,
        valid: np.ndarray,
        table_depth: float,
    ) -> np.ndarray:
        height, width = depth.shape
        row_baseline = np.full(height, table_depth, dtype=np.float64)
        for row in range(height):
            row_values = depth[row][valid[row]]
            if row_values.size >= 24:
                row_baseline[row] = float(np.percentile(row_values, 92.0))

        kernel = np.ones(9, dtype=np.float64) / 9.0
        row_baseline = np.convolve(np.pad(row_baseline, (4, 4), mode="edge"), kernel, mode="valid")
        baseline_map = np.repeat(row_baseline[:, None], width, axis=1)

        foreground = valid & (depth < baseline_map - self.foreground_offset_mm)
        # Suppress strong border-connected clutter that tends to fuse with the table.
        border_margin = max(width // 8, 18)
        foreground[:, :border_margin] = False
        foreground[: max(height // 10, 12), :] = False
        foreground[-max(height // 8, 18):, :] = False
        return foreground

    def _compute_component_diagnostics(
        self,
        foreground: np.ndarray,
        depth: np.ndarray,
        table_depth: float,
        camera_params: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        import cv2

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(foreground.astype(np.uint8), connectivity=8)
        best_score = -1e9
        best_index = -1
        rows: List[Dict[str, Any]] = []
        height, width = depth.shape
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_component_area:
                continue
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            mask = labels == label
            component_depth = depth[mask]
            if component_depth.size == 0:
                continue
            median_depth = float(np.median(component_depth))
            depth_gain = max(table_depth - median_depth, 0.0)
            fill_ratio = float(area) / max(float(w * h), 1.0)
            center_x = x + 0.5 * w
            center_y = y + 0.5 * h
            center_dx = abs(center_x - width * 0.52) / max(width, 1.0)
            center_dy = abs(center_y - height * 0.58) / max(height, 1.0)
            large_area_penalty = max(area - 1500, 0) * 0.22
            flat_fill_penalty = max(fill_ratio - 0.72, 0.0) * 420.0
            border_penalty = 0.0
            if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
                border_penalty = 250.0
            world_point = self._mask_centroid_to_world(mask, depth, camera_params)
            workspace_bonus = 0.0
            if world_point is not None:
                if 0.0 <= float(world_point[2]) <= 1.1:
                    workspace_bonus += 35.0
                if -0.8 <= float(world_point[0]) <= 0.8 and -0.8 <= float(world_point[1]) <= 0.8:
                    workspace_bonus += 25.0
            score = (
                depth_gain * 11.0
                + min(area, 1200) * 0.28
                + workspace_bonus
                - large_area_penalty
                - flat_fill_penalty
                - center_dx * 220.0
                - center_dy * 120.0
                - border_penalty
            )
            row = {
                "label": int(label),
                "area": area,
                "bbox": [x, y, w, h],
                "median_depth_mm": median_depth,
                "depth_gain_mm": depth_gain,
                "fill_ratio": fill_ratio,
                "center_dx": center_dx,
                "center_dy": center_dy,
                "workspace_bonus": workspace_bonus,
                "score": score,
                "world_centroid": None if world_point is None else [float(v) for v in world_point.tolist()],
                "mask": mask,
                "selected": False,
            }
            rows.append(row)
            if score > best_score:
                best_score = score
                best_index = len(rows) - 1

        if 0 <= best_index < len(rows):
            rows[best_index]["selected"] = True
        return rows

    def _expand_grasp_candidates(
        self,
        base_candidates: List[Dict[str, Any]],
        object_pose: Optional[List[float]] = None,
        context: Any | None = None,
    ) -> List[Dict[str, Any]]:
        expanded: List[Dict[str, Any]] = []
        seen = set()
        for index, candidate in enumerate(base_candidates):
            variants = self._candidate_variants(candidate, object_pose=object_pose, context=context)
            for variant in variants:
                key = self._candidate_key(variant)
                if key in seen:
                    continue
                seen.add(key)
                variant.setdefault("score", max(1.0 - 0.05 * len(expanded), 0.0))
                variant.setdefault("rank", len(expanded))
                variant.setdefault("source_index", index)
                expanded.append(variant)
        return self._rank_variants_with_reachability(expanded, context=context)

    def _candidate_variants(
        self,
        candidate: Dict[str, Any],
        object_pose: Optional[List[float]] = None,
        context: Any | None = None,
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

        current_eef = None
        if context is not None and hasattr(context, "world_state"):
            current_eef = getattr(context.world_state.robot, "eef_pose", None)

        specs = [("base", np.zeros(3), np.zeros(3), 1.0)]
        if current_eef is not None and len(current_eef) >= 3:
            current_xyz = np.asarray(current_eef[:3], dtype=np.float64)
            reach_shift = np.clip((current_xyz - pregrasp_xyz) * 0.28, -0.04, 0.04)
            specs.append(("reach_relief", np.zeros(3), reach_shift + np.asarray([0.0, 0.0, 0.015]), 0.985))
        if object_pose is not None and len(object_pose) >= 3:
            object_xyz = np.asarray(object_pose[:3], dtype=np.float64)
            recenter = np.clip(object_xyz - grasp_xyz, -0.025, 0.025)
            specs.append(("vision_recenter", recenter, recenter + np.asarray([0.0, 0.0, 0.02]), 0.965))
        specs.extend(
            [
                ("pregrasp_high", np.zeros(3), np.asarray([0.0, 0.0, 0.04]), 0.95),
                ("lateral_pos", lateral * 0.012, lateral * 0.025 + np.asarray([0.0, 0.0, 0.02]), 0.92),
                ("lateral_neg", -lateral * 0.012, -lateral * 0.025 + np.asarray([0.0, 0.0, 0.02]), 0.91),
                ("pregrasp_backoff", np.zeros(3), approach_dir * 0.035 + np.asarray([0.0, 0.0, 0.015]), 0.88),
            ]
        )

        variants: List[Dict[str, Any]] = []
        for label, grasp_delta, pregrasp_delta, base_score in specs:
            variant = dict(candidate)
            variant_pose = list(pose)
            variant_pregrasp = list(pregrasp)
            for axis in range(3):
                variant_pose[axis] = float(grasp_xyz[axis] + grasp_delta[axis])
                variant_pregrasp[axis] = float(pregrasp_xyz[axis] + pregrasp_delta[axis])
            variant["pose"] = variant_pose
            variant["pregrasp_pose"] = variant_pregrasp
            variant["variant_label"] = label
            variant["score"] = float(base_score)
            if current_eef is not None and len(current_eef) >= 3:
                dist = float(np.linalg.norm(np.asarray(current_eef[:3], dtype=np.float64) - np.asarray(variant_pregrasp[:3], dtype=np.float64)))
                variant["score"] -= min(dist, 1.0) * 0.03
            variants.append(variant)
        return variants

    @staticmethod
    def _candidate_key(candidate: Dict[str, Any]) -> tuple:
        pose = tuple(round(float(v), 4) for v in list(candidate.get("pose") or [])[:7])
        pregrasp = tuple(round(float(v), 4) for v in list(candidate.get("pregrasp_pose") or [])[:7])
        return pose, pregrasp

    def _rank_variants_with_reachability(
        self,
        candidates: List[Dict[str, Any]],
        context: Any | None = None,
    ) -> List[Dict[str, Any]]:
        if not candidates or context is None:
            return candidates
        adapters = getattr(context, "adapters", {}) or {}
        sdk = adapters.get("sdk")
        if sdk is None or not hasattr(sdk, "evaluate_pose_candidates"):
            return sorted(candidates, key=lambda item: item.get("score", 0.0), reverse=True)

        pregrasp_poses = [list(candidate.get("pregrasp_pose") or []) for candidate in candidates]
        evaluations = sdk.evaluate_pose_candidates(pregrasp_poses, kind="pregrasp")
        for candidate, evaluation in zip(candidates, evaluations):
            candidate["planner_status"] = evaluation.get("status", "Unknown")
            if evaluation.get("waypoint_count") is not None:
                candidate["planner_waypoint_count"] = int(evaluation["waypoint_count"])

            score = float(candidate.get("score", 0.0))
            if candidate["planner_status"] == "Success":
                score += 2.0
                score -= min(float(candidate.get("planner_waypoint_count", 0)), 400.0) / 1000.0
            elif candidate["planner_status"] in {"Failure", "Fail"}:
                score -= 1.0
            else:
                score -= 0.15
            candidate["score"] = score
        return sorted(candidates, key=lambda item: item.get("score", 0.0), reverse=True)

    def _synthesize_grasp_candidates_from_pose(
        self,
        object_pose: Optional[List[float]] = None,
        context: Any | None = None,
    ) -> List[Dict[str, Any]]:
        if object_pose is None or len(object_pose) < 3:
            return []
        orientation = [0.9949492141138409, 0.0001428575322812901, 0.10037948350189996, 1.4760360441018511e-05]
        if context is not None and hasattr(context, "world_state"):
            eef_pose = getattr(context.world_state.robot, "eef_pose", None)
            if eef_pose and len(eef_pose) >= 7:
                orientation = [float(v) for v in eef_pose[3:7]]

        obj = np.asarray(object_pose[:3], dtype=np.float64)
        base_height = float(obj[2]) + 0.085
        pre_height = base_height + 0.055
        offsets = [
            ("synth_center", np.asarray([0.0, 0.0, 0.0])),
            ("synth_back", np.asarray([-0.02, 0.0, 0.0])),
            ("synth_left", np.asarray([0.0, -0.02, 0.0])),
            ("synth_right", np.asarray([0.0, 0.02, 0.0])),
            ("synth_high", np.asarray([0.0, 0.0, 0.02])),
        ]
        candidates: List[Dict[str, Any]] = []
        for rank, (label, offset) in enumerate(offsets):
            grasp_xyz = obj + offset
            pregrasp_xyz = grasp_xyz + np.asarray([-0.09, 0.0, 0.05])
            pose = [float(grasp_xyz[0]), float(grasp_xyz[1]), float(base_height + offset[2]), *orientation]
            pregrasp = [float(pregrasp_xyz[0]), float(pregrasp_xyz[1]), float(pre_height + offset[2]), *orientation]
            candidates.append(
                {
                    "pose": pose,
                    "pregrasp_pose": pregrasp,
                    "arm": "left",
                    "score": 0.72 - rank * 0.03,
                    "variant_label": label,
                    "rank": rank,
                    "source_index": -1,
                }
            )
        return candidates

    def export_component_diagnostics(self, observation: PerceptionObservation) -> Dict[str, Any]:
        depth = observation.depth
        metadata = observation.metadata or {}
        camera_params = metadata.get("camera_params")
        if depth is None or camera_params is None:
            return {"ok": False, "message": "Missing depth or camera_params"}

        depth = np.asarray(depth, dtype=np.float64)
        if depth.ndim != 2:
            return {"ok": False, "message": "Depth must be HxW"}

        valid = np.isfinite(depth) & (depth > 1.0)
        if valid.sum() < self.min_component_area:
            return {"ok": False, "message": "Not enough valid depth pixels"}

        table_depth = float(np.percentile(depth[valid], 68.0))
        foreground = self._build_foreground_mask(depth, valid, table_depth)
        foreground = self._clean_foreground_mask(foreground)
        diagnostics = self._compute_component_diagnostics(
            foreground,
            depth,
            table_depth=table_depth,
            camera_params=camera_params,
        )
        self.last_component_diagnostics = diagnostics
        return {
            "ok": True,
            "table_depth_mm": table_depth,
            "foreground_pixel_count": int(np.count_nonzero(foreground)),
            "components": [self._sanitize_component_row(row) for row in diagnostics],
            "foreground_mask": foreground,
        }

    @staticmethod
    def _sanitize_component_row(row: Dict[str, Any]) -> Dict[str, Any]:
        sanitized = dict(row)
        sanitized.pop("mask", None)
        return sanitized

    def _mask_centroid_to_world(
        self,
        mask: np.ndarray,
        depth_mm: np.ndarray,
        camera_params: Dict[str, Any],
    ) -> Optional[np.ndarray]:
        try:
            intrinsic = self._to_numpy(camera_params["intrinsic_cv"])
            extrinsic = self._resolve_camera_to_world(camera_params)
        except Exception:
            return None

        ys, xs = np.where(mask)
        if xs.size == 0:
            return None
        z = depth_mm[ys, xs] / 1000.0
        fx = float(intrinsic[0, 0])
        fy = float(intrinsic[1, 1])
        cx = float(intrinsic[0, 2])
        cy = float(intrinsic[1, 2])
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        points_cam = np.stack([x, y, z, np.ones_like(z)], axis=1)
        cam_to_world = extrinsic
        points_world = (cam_to_world @ points_cam.T).T[:, :3]
        centroid = np.median(points_world, axis=0)
        return centroid

    def _resolve_camera_to_world(self, camera_params: Dict[str, Any]) -> np.ndarray:
        # Prefer `extrinsic_cv` because our back-projection follows cv pixel
        # conventions. RoboTwin's `cam2world_gl` lives in a GL camera frame and
        # produces large pose errors if used directly with cv intrinsics.
        extrinsic = self._to_numpy(camera_params["extrinsic_cv"])
        if extrinsic.shape == (4, 4):
            return np.linalg.inv(extrinsic)
        if extrinsic.shape == (3, 4):
            lifted = np.eye(4, dtype=np.float64)
            lifted[:3, :] = extrinsic
            return np.linalg.inv(lifted)
        if "cam2world_gl" in camera_params:
            cam_to_world = self._to_numpy(camera_params["cam2world_gl"])
            if cam_to_world.shape == (4, 4):
                return cam_to_world
        raise ValueError(f"Unsupported extrinsic shape: {extrinsic.shape}")

    @staticmethod
    def _to_numpy(value: Any) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            value = value.numpy()
        return np.asarray(value, dtype=np.float64)
