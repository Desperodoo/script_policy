"""Perception providers for oracle-assisted and non-oracle runtime integration."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np
from script_runtime.planning import (
    build_grasp_candidate_variants,
    build_synthesized_grasp_candidates,
    quat_normalize,
)


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
        backend_candidate_reads: int = 2,
    ):
        self.oracle_backend = oracle_backend
        self.use_oracle_fallback = bool(use_oracle_fallback)
        self.foreground_offset_mm = float(foreground_offset_mm)
        self.min_component_area = int(min_component_area)
        self.backend_candidate_reads = max(int(backend_candidate_reads), 1)
        self.last_pose_source = "none"
        self.last_grasp_source = "none"
        self.last_component_diagnostics: List[Dict[str, Any]] = []
        self.last_grasp_diagnostics: List[Dict[str, Any]] = []

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
        base_candidates, feasible_backend = self._collect_backend_grasp_candidates(context=context)
        object_pose = self._estimate_from_depth(observation)
        if not base_candidates:
            synthesized = self._synthesize_grasp_candidates_from_pose(object_pose=object_pose, context=context)
            if not synthesized:
                self.last_grasp_source = "none"
                return None
            expanded = self._rank_variants_with_reachability(synthesized, context=context)
            self.last_grasp_source = "depth_synthesized"
            return expanded

        if feasible_backend:
            self.last_grasp_source = "oracle_feasibility_first"
            return base_candidates

        expanded = self._expand_grasp_candidates(base_candidates, object_pose=object_pose, context=context)
        self.last_grasp_source = "oracle_augmented" if len(expanded) > len(base_candidates) else "oracle_base"
        return expanded

    def _collect_backend_grasp_candidates(
        self,
        context: Any | None = None,
    ) -> tuple[Optional[List[Dict[str, Any]]], bool]:
        candidate_sets: List[List[Dict[str, Any]]] = []

        if self.oracle_backend is not None and hasattr(self.oracle_backend, "get_grasp_candidates"):
            for _ in range(self.backend_candidate_reads):
                batch = self.oracle_backend.get_grasp_candidates()
                if batch:
                    candidate_sets.append(batch)

        cached = self._cached_grasp_candidates(context=context)
        if cached:
            candidate_sets.append(cached)

        merged = self._merge_backend_candidate_sets(candidate_sets)
        if not merged:
            return None, False
        if self._has_backend_feasible_candidate(merged):
            return self._prefer_backend_feasible_candidates(merged), True
        return merged, False

    def _merge_backend_candidate_sets(
        self,
        candidate_sets: List[List[Dict[str, Any]]],
    ) -> List[Dict[str, Any]]:
        merged: Dict[tuple, Dict[str, Any]] = {}
        order: List[tuple] = []
        for read_index, candidates in enumerate(candidate_sets):
            for candidate_index, candidate in enumerate(candidates or []):
                item = dict(candidate)
                item.setdefault("backend_read_index", read_index)
                item.setdefault("backend_candidate_index", candidate_index)
                key = self._backend_candidate_identity(item)
                existing = merged.get(key)
                if existing is None:
                    merged[key] = item
                    order.append(key)
                    continue
                merged[key] = self._merge_backend_candidate(existing, item)
        return [merged[key] for key in order]

    @classmethod
    def _merge_backend_candidate(
        cls,
        existing: Dict[str, Any],
        incoming: Dict[str, Any],
    ) -> Dict[str, Any]:
        left = dict(existing)
        right = dict(incoming)
        winner = left
        loser = right
        if cls._candidate_merge_preference(right) > cls._candidate_merge_preference(left):
            winner = right
            loser = left
        merged = dict(winner)
        for key, value in loser.items():
            if key not in merged or merged.get(key) in (None, "", [], {}):
                merged[key] = value
        read_indices = []
        for source in (left, right):
            if source.get("backend_read_indices"):
                read_indices.extend(list(source["backend_read_indices"]))
            elif source.get("backend_read_index") is not None:
                read_indices.append(int(source["backend_read_index"]))
        if read_indices:
            merged["backend_read_indices"] = sorted(set(read_indices))
        return merged

    @staticmethod
    def _candidate_merge_preference(candidate: Dict[str, Any]) -> tuple:
        status_rank = RoboTwinDepthPoseProvider._planner_status_rank(candidate.get("planner_status"))
        score = float(candidate.get("score", 0.0))
        waypoint_count = candidate.get("planner_waypoint_count")
        waypoint_sort = -10**9
        if waypoint_count is not None:
            waypoint_sort = -int(waypoint_count)
        return status_rank, score, waypoint_sort

    @classmethod
    def _backend_candidate_identity(cls, candidate: Dict[str, Any]) -> tuple:
        contact = candidate.get("contact_point_id")
        arm = candidate.get("arm")
        pose_key, pregrasp_key = cls._candidate_key(candidate)
        if contact is not None:
            return ("contact", int(contact), str(arm or ""))
        return ("pose", str(arm or ""), pose_key, pregrasp_key)

    @staticmethod
    def _cached_grasp_candidates(context: Any | None = None) -> Optional[List[Dict[str, Any]]]:
        if context is None:
            return None
        blackboard = getattr(context, "blackboard", None)
        if blackboard is not None and hasattr(blackboard, "get"):
            cached = blackboard.get("grasp_candidates")
            if cached:
                return [dict(candidate) for candidate in cached]
        world_state = getattr(context, "world_state", None)
        learned = None if world_state is None else getattr(world_state, "learned", None)
        cached = None if learned is None else getattr(learned, "grasp_candidates", None)
        if cached:
            return [dict(candidate) for candidate in cached]
        return None

    @staticmethod
    def _has_backend_feasible_candidate(base_candidates: Optional[List[Dict[str, Any]]]) -> bool:
        if not base_candidates:
            return False
        return any(str(candidate.get("planner_status", "Unknown")) == "Success" for candidate in base_candidates)

    def _prefer_backend_feasible_candidates(
        self,
        base_candidates: List[Dict[str, Any]],
    ) -> Optional[List[Dict[str, Any]]]:
        if not self._has_backend_feasible_candidate(base_candidates):
            return None
        ranked = sorted(
            [dict(candidate) for candidate in base_candidates],
            key=lambda item: (
                -self._planner_status_rank(item.get("planner_status")),
                -float(item.get("score", 0.0)),
                int(item.get("planner_waypoint_count") or 10**9),
            ),
        )
        self.last_grasp_diagnostics = [
            {
                "variant_label": str(candidate.get("variant_label", "")),
                "score": float(candidate.get("score", 0.0)),
                "planner_status": str(candidate.get("planner_status", "Unknown")),
                "planner_waypoint_count": candidate.get("planner_waypoint_count"),
                "pose_xyz": [float(v) for v in list(candidate.get("pose") or [])[:3]],
                "pregrasp_xyz": [float(v) for v in list(candidate.get("pregrasp_pose") or [])[:3]],
                "planner_debug": candidate.get("planner_debug"),
            }
            for candidate in ranked[:12]
        ]
        return ranked

    @staticmethod
    def _planner_status_rank(status: Any) -> int:
        normalized = str(status or "Unknown")
        if normalized == "Success":
            return 3
        if normalized in {"Partial", "Fallback"}:
            return 2
        if normalized in {"Failure", "Fail"}:
            return 0
        return 1

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
        current_eef = None
        if context is not None and hasattr(context, "world_state"):
            current_eef = getattr(context.world_state.robot, "eef_pose", None)
        for index, candidate in enumerate(base_candidates):
            variants = build_grasp_candidate_variants(
                candidate=candidate,
                object_pose=object_pose,
                current_eef=current_eef,
            )
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
            if evaluation.get("planner_debug") is not None:
                candidate["planner_debug"] = evaluation.get("planner_debug")
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
        ranked = sorted(candidates, key=lambda item: item.get("score", 0.0), reverse=True)
        self.last_grasp_diagnostics = [
            {
                "variant_label": str(candidate.get("variant_label", "")),
                "score": float(candidate.get("score", 0.0)),
                "planner_status": str(candidate.get("planner_status", "Unknown")),
                "planner_waypoint_count": candidate.get("planner_waypoint_count"),
                "pose_xyz": [float(v) for v in list(candidate.get("pose") or [])[:3]],
                "pregrasp_xyz": [float(v) for v in list(candidate.get("pregrasp_pose") or [])[:3]],
                "planner_debug": candidate.get("planner_debug"),
            }
            for candidate in ranked[:12]
        ]
        return ranked

    def _synthesize_grasp_candidates_from_pose(
        self,
        object_pose: Optional[List[float]] = None,
        context: Any | None = None,
    ) -> List[Dict[str, Any]]:
        if object_pose is None or len(object_pose) < 3:
            return []
        orientation = [0.9949492141138409, 0.0001428575322812901, 0.10037948350189996, 1.4760360441018511e-05]
        arm = "left"
        if context is not None and hasattr(context, "world_state"):
            eef_pose = getattr(context.world_state.robot, "eef_pose", None)
            if eef_pose and len(eef_pose) >= 7:
                orientation = [float(v) for v in eef_pose[3:7]]
        if context is not None:
            adapters = getattr(context, "adapters", {}) or {}
            sdk = adapters.get("sdk")
            if sdk is not None and hasattr(sdk, "_active_arm"):
                try:
                    arm = str(sdk._active_arm())
                except Exception:
                    arm = "left"
        return build_synthesized_grasp_candidates(
            object_pose=object_pose,
            arm=arm,
            orientation=quat_normalize(orientation),
        )

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
