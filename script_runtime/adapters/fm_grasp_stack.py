"""FM-first grasp stack adapters.

This module introduces an explicit multi-stage perception + grasp pipeline:

1. target grounding
2. object pose estimation
3. grasp proposal generation
4. optional task-aware reranking

The first shipped version is intentionally interface-first:
- it can already run with delegate/oracle baselines
- it records backend comparison diagnostics
- it exposes stable integration points for external open-source FM / grasp repos

The goal is to stop baking all upstream grasp reasoning into a single monolithic
``PerceptionAdapter`` and instead make backend comparison a first-class feature.
"""

from __future__ import annotations

import importlib
import importlib.util
import inspect
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

from .perception_adapter import PerceptionAdapter, PerceptionObservation, RoboTwinDepthPoseProvider
from script_runtime.planning import (
    annotate_grasp_candidates,
    quat_normalize,
    resolve_runtime_task_name,
    sort_grasp_candidates_by_semantics,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _resolve_repo_path(repo_path: str | None) -> Path | None:
    if not repo_path:
        return None
    path = Path(str(repo_path)).expanduser()
    if not path.is_absolute():
        path = _repo_root() / path
    return path.resolve()


def _import_available(module_name: str) -> bool:
    try:
        return importlib.util.find_spec(module_name) is not None
    except Exception:
        return False


def _resolve_python_bin(python_bin: str | None) -> str:
    if python_bin:
        return str(Path(str(python_bin)).expanduser())
    return str(Path(sys.executable).resolve())


def _python_import_status(module_names: Sequence[str], python_bin: str | None = None) -> Dict[str, Any]:
    names = [str(name) for name in list(module_names or []) if str(name)]
    if python_bin is None:
        return {
            "python_bin": str(Path(sys.executable).resolve()),
            "python_bin_exists": True,
            "module_status": {name: bool(_import_available(name)) for name in names},
            "module_errors": {name: "missing" for name in names if not _import_available(name)},
        }
    resolved_python = _resolve_python_bin(python_bin)
    python_path = Path(resolved_python)
    status: Dict[str, Any] = {
        "python_bin": resolved_python,
        "python_bin_exists": python_path.exists(),
        "module_status": {},
        "module_errors": {},
    }
    if not names:
        return status
    if not python_path.exists():
        for name in names:
            status["module_status"][name] = False
            status["module_errors"][name] = "python_bin_missing"
        return status
    script = """
import importlib.util
import json
import sys

names = json.loads(sys.argv[1])
payload = {"module_status": {}, "module_errors": {}}
for name in names:
    try:
        payload["module_status"][name] = importlib.util.find_spec(name) is not None
        if not payload["module_status"][name]:
            payload["module_errors"][name] = "missing"
    except Exception as exc:
        payload["module_status"][name] = False
        payload["module_errors"][name] = f"{type(exc).__name__}:{exc}"
print(json.dumps(payload))
""".strip()
    try:
        result = subprocess.run(
            [resolved_python, "-c", script, json.dumps(names)],
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
    except Exception as exc:
        for name in names:
            status["module_status"][name] = False
            status["module_errors"][name] = f"subprocess_error:{type(exc).__name__}:{exc}"
        return status
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()[-4000:]
        for name in names:
            status["module_status"][name] = False
            status["module_errors"][name] = f"subprocess_nonzero:{stderr}"
        return status
    try:
        payload = json.loads((result.stdout or "").strip() or "{}")
    except Exception as exc:
        for name in names:
            status["module_status"][name] = False
            status["module_errors"][name] = f"bad_json:{type(exc).__name__}:{exc}"
        return status
    status["module_status"] = dict(payload.get("module_status") or {})
    status["module_errors"] = dict(payload.get("module_errors") or {})
    for name in names:
        status["module_status"].setdefault(name, False)
        if not status["module_status"][name]:
            status["module_errors"].setdefault(name, "missing")
    return status


def _context_task_id(context: Any | None) -> str:
    task_id = "" if context is None else str(getattr(context, "task_id", "") or "")
    if task_id:
        return task_id
    world_state = None if context is None else getattr(context, "world_state", None)
    execution = None if world_state is None else getattr(world_state, "execution", None)
    return str("" if execution is None else getattr(execution, "task_id", "") or "")


def _runtime_artifact_root(context: Any | None) -> Path:
    blackboard = None if context is None else getattr(context, "blackboard", None)
    artifact_dir = None if blackboard is None else blackboard.get("runtime_artifact_dir")
    if artifact_dir:
        return Path(str(artifact_dir)).expanduser()
    return _repo_root() / "script_runtime" / "artifacts"


def _backend_run_dir(context: Any | None, backend_name: str) -> Path:
    task_id = _context_task_id(context) or backend_name
    run_dir = _runtime_artifact_root(context) / task_id / "fm_runtime" / backend_name
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def _rotation_matrix_to_quat_wxyz(rotation: Any) -> List[float]:
    import numpy as np

    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s
    return quat_normalize([float(qw), float(qx), float(qy), float(qz)])


def _quat_wxyz_to_matrix(quat: Any):
    import numpy as np

    q = np.asarray(quat, dtype=np.float64).reshape(-1)
    if q.shape[0] < 4:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = [float(v) for v in q[:4]]
    norm = float(np.linalg.norm([w, x, y, z]))
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = [value / norm for value in (w, x, y, z)]
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _to_numpy(value: Any):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    import numpy as np

    return np.asarray(value, dtype=np.float64)


def _resolve_camera_to_world(camera_params: Dict[str, Any]):
    import numpy as np

    if "extrinsic_cv" in camera_params:
        extrinsic = np.asarray(camera_params["extrinsic_cv"], dtype=np.float64)
        if extrinsic.shape == (4, 4):
            return np.linalg.inv(extrinsic)
        if extrinsic.shape == (3, 4):
            lifted = np.eye(4, dtype=np.float64)
            lifted[:3, :] = extrinsic
            return np.linalg.inv(lifted)
    if "cam2world_cv" in camera_params:
        cam_to_world = np.asarray(camera_params["cam2world_cv"], dtype=np.float64)
        if cam_to_world.shape == (4, 4):
            return cam_to_world
    if "cam2world_gl" in camera_params:
        cam_to_world = np.asarray(camera_params["cam2world_gl"], dtype=np.float64)
        if cam_to_world.shape == (4, 4):
            return cam_to_world
    return np.eye(4, dtype=np.float64)


def _find_matching_files(root: Path | None, patterns: Sequence[str]) -> List[str]:
    if root is None or not root.exists():
        return []
    matches: List[str] = []
    for pattern in patterns:
        for path in root.glob(pattern):
            if path.is_file():
                matches.append(str(path))
    return sorted(set(matches))


def _find_foundationpose_weight_dirs(root: Path | None) -> Dict[str, str]:
    if root is None or not root.exists():
        return {}
    weights_root = root / "weights"
    if not weights_root.exists():
        return {}
    candidates: Dict[str, str] = {}
    for child in weights_root.rglob("*"):
        if not child.is_dir():
            continue
        config_path = child / "config.yml"
        model_path = child / "model_best.pth"
        if not config_path.is_file() or not model_path.is_file():
            continue
        candidates[child.name] = str(child.resolve())
    return dict(sorted(candidates.items()))


def _find_contact_graspnet_checkpoint_dirs(root: Path | None) -> Dict[str, str]:
    if root is None or not root.exists():
        return {}
    checkpoints_root = root / "checkpoints"
    if not checkpoints_root.exists():
        return {}
    candidates: Dict[str, str] = {}
    for child in checkpoints_root.rglob("*"):
        if not child.is_dir():
            continue
        config_path = child / "config.yaml"
        if not config_path.is_file():
            continue
        has_checkpoint = any(child.glob("model.ckpt-*.index")) or any(child.glob("model.ckpt-*.data-*"))
        if not has_checkpoint:
            continue
        candidates[child.name] = str(child.resolve())
    return dict(sorted(candidates.items()))


def _sort_key_for_candidate(candidate: Dict[str, Any]) -> tuple[int, float, int]:
    status = str(candidate.get("planner_status", "Unknown") or "Unknown")
    status_rank = 0 if status == "Success" else 1
    score = -float(candidate.get("score", 0.0) or 0.0)
    waypoint_count = int(candidate.get("planner_waypoint_count") or 10**9)
    return status_rank, score, waypoint_count


def _candidate_identity(candidate: Dict[str, Any]) -> tuple:
    contact = candidate.get("contact_point_id")
    arm = str(candidate.get("arm", "") or "")
    if contact is not None:
        return ("contact", int(contact), arm)
    pose = tuple(round(float(value), 5) for value in list(candidate.get("pose") or [])[:7])
    pregrasp = tuple(round(float(value), 5) for value in list(candidate.get("pregrasp_pose") or [])[:7])
    return ("pose", arm, pose, pregrasp)


def _merge_candidate_lists(candidate_groups: Sequence[Sequence[Dict[str, Any]]]) -> List[Dict[str, Any]]:
    merged: Dict[tuple, Dict[str, Any]] = {}
    order: List[tuple] = []
    for group in candidate_groups:
        for candidate in group:
            item = dict(candidate)
            key = _candidate_identity(item)
            existing = merged.get(key)
            if existing is None:
                merged[key] = item
                order.append(key)
                continue
            winner = existing
            challenger = item
            if _sort_key_for_candidate(challenger) < _sort_key_for_candidate(winner):
                winner, challenger = challenger, winner
            updated = dict(winner)
            sources = []
            for source_item in (existing, item):
                for source_name in list(source_item.get("proposal_sources", []) or []):
                    if source_name not in sources:
                        sources.append(source_name)
                backend_name = source_item.get("proposal_backend")
                if backend_name and backend_name not in sources:
                    sources.append(str(backend_name))
            if sources:
                updated["proposal_sources"] = sources
            for key_name, value in challenger.items():
                if key_name not in updated or updated.get(key_name) in (None, "", [], {}):
                    updated[key_name] = value
            merged[key] = updated
    ranked = [merged[key] for key in order]
    ranked.sort(key=_sort_key_for_candidate)
    return ranked


def _candidate_runtime_reason(candidate: Dict[str, Any]) -> str:
    planner_status = str(candidate.get("planner_status", "Unknown") or "Unknown")
    if planner_status == "Success":
        return "planner_success"
    notes = str(dict(candidate.get("affordance") or {}).get("notes") or "")
    if "reference_contact_unavailable_in_current_instance" in notes:
        return "reference_contact_unavailable"
    debug = dict(candidate.get("planner_debug") or {})
    if debug:
        return "planner_failed"
    if candidate.get("contact_point") in (None, [], ()):
        return "missing_contact_point"
    return "candidate_rejected"


def _guided_family_label(candidate: Dict[str, Any]) -> str:
    label = str(candidate.get("variant_label") or "")
    match = re.match(r"^(contact_graspnet_guided_c\d+)", label)
    if match:
        return match.group(1)
    template_id = candidate.get("template_contact_point_id")
    if template_id is not None:
        return f"contact_graspnet_guided_c{template_id}"
    return ""


def _guided_stage_summary(
    *,
    reranked: Sequence[Dict[str, Any]],
    diagnostics: Sequence[Dict[str, Any]],
    template_debug: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    all_candidates = [dict(row or {}) for row in list(reranked or [])]
    guided_candidates = [row for row in all_candidates if _guided_family_label(row)]
    guided_feasible = [
        _guided_family_label(row)
        for row in guided_candidates
        if str(row.get("planner_status", "Unknown") or "Unknown") == "Success"
    ]
    unique_guided_feasible = sorted({label for label in guided_feasible if label})
    raw_candidates = [
        row
        for row in all_candidates
        if str(row.get("variant_label") or "").startswith("contact_graspnet_seg")
    ]
    template_candidates = [
        row
        for row in all_candidates
        if str(row.get("variant_label") or "").startswith("contact_graspnet_template_")
    ]
    summary: Dict[str, Any] = {
        "guided_candidate_count": len(guided_candidates),
        "guided_feasible_families": unique_guided_feasible,
        "has_guided_feasible_family": bool(unique_guided_feasible),
        "raw_contact_candidate_count": len(raw_candidates),
        "template_transfer_candidate_count": len(template_candidates),
        "guided_merge_coverage_ratio": 0.0,
        "guided_rejection_reasons": [],
    }
    if raw_candidates:
        summary["guided_merge_coverage_ratio"] = round(float(len(guided_candidates)) / float(len(raw_candidates)), 4)
    rejection_rows = []
    for row in guided_candidates:
        label = _guided_family_label(row)
        if not label or str(row.get("planner_status", "Unknown") or "Unknown") == "Success":
            continue
        rejection_rows.append(
            {
                "variant_label": label,
                "reason": _candidate_runtime_reason(row),
                "planner_status": str(row.get("planner_status", "Unknown") or "Unknown"),
                "task_compatibility": str(row.get("task_compatibility") or ""),
                "semantic_source": str(row.get("semantic_source") or ""),
            }
        )
    summary["guided_rejection_reasons"] = rejection_rows
    contact_runtime = next(
        (dict(row or {}) for row in diagnostics if str(row.get("backend_name") or "") == "contact_graspnet"),
        {},
    )
    contact_diag = dict(contact_runtime.get("diagnostics") or {})
    contact_summary = dict(contact_diag.get("summary") or {})
    summary["contact_graspnet_summary"] = {
        "grasp_total": int(contact_summary.get("grasp_total") or 0),
        "grasp_group_count": int(contact_summary.get("grasp_group_count") or 0),
        "summary_path": str(contact_diag.get("summary_path") or ""),
        "output_dir": str(contact_diag.get("output_dir") or ""),
    }
    summary["template_source_debug"] = dict(template_debug or {})
    return summary


def _contact_graspnet_variant_kind(candidate: Dict[str, Any]) -> str:
    label = str(candidate.get("variant_label") or "")
    if label.startswith("contact_graspnet_guided_"):
        return "guided"
    if label.startswith("contact_graspnet_template_"):
        return "template"
    if label.startswith("contact_graspnet_seg"):
        return "raw"
    return "other"


@dataclass
class GroundingResult:
    target_name: str
    source: str
    score: float = 0.0
    box_xyxy: Optional[List[float]] = None
    mask: Any = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class BackendResult:
    backend_name: str
    available: bool
    ok: bool
    message: str = ""
    payload: Any = None
    diagnostics: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "backend_name": self.backend_name,
            "available": bool(self.available),
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "diagnostics": dict(self.diagnostics or {}),
        }


class TargetGrounder:
    name = "target_grounder"

    def is_available(self) -> bool:
        return True

    def ground(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> BackendResult:
        return BackendResult(backend_name=self.name, available=self.is_available(), ok=False, message="not_implemented")


class ObjectPoseEstimator:
    name = "object_pose_estimator"

    def is_available(self) -> bool:
        return True

    def estimate_pose(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
    ) -> BackendResult:
        return BackendResult(backend_name=self.name, available=self.is_available(), ok=False, message="not_implemented")


class GraspProposalBackend:
    name = "grasp_proposal_backend"

    def is_available(self) -> bool:
        return True

    def propose_grasps(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> BackendResult:
        return BackendResult(backend_name=self.name, available=self.is_available(), ok=False, message="not_implemented")


class TaskAwareGraspReranker:
    name = "grasp_reranker"

    def rerank(
        self,
        candidates: Sequence[Dict[str, Any]],
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> Sequence[Dict[str, Any]]:
        return list(candidates)


class TaskGoalTargetGrounder(TargetGrounder):
    name = "task_goal_prompt"

    def ground(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> BackendResult:
        target_name = str(
            observation.task_goal.get("target_object")
            or observation.task_goal.get("object")
            or observation.task_goal.get("object_name")
            or ""
        )
        if not target_name:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="missing_target_object_in_task_goal",
            )
        result = GroundingResult(
            target_name=target_name,
            source=self.name,
            score=1.0,
            metadata={"task_goal": dict(observation.task_goal or {})},
        )
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=result,
            diagnostics={"target_name": target_name},
        )


class GroundedSAM2Grounder(TargetGrounder):
    name = "grounded_sam2"

    def __init__(
        self,
        repo_path: str | None = None,
        *,
        model_id: str = "IDEA-Research/grounding-dino-tiny",
        sam2_config: str = "configs/sam2.1/sam2.1_hiera_s.yaml",
        sam2_checkpoint: str = "checkpoints/sam2.1_hiera_small.pt",
        sam2_hf_model_id: str = "facebook/sam2.1-hiera-small",
        box_threshold: float = 0.35,
        text_threshold: float = 0.25,
        max_detections: int = 8,
        device: str | None = None,
    ):
        self.repo_path = str(repo_path or "")
        self.model_id = str(model_id or "IDEA-Research/grounding-dino-tiny")
        self.sam2_config = str(sam2_config or "configs/sam2.1/sam2.1_hiera_s.yaml")
        self.sam2_checkpoint = str(sam2_checkpoint or "checkpoints/sam2.1_hiera_small.pt")
        self.sam2_hf_model_id = str(sam2_hf_model_id or "facebook/sam2.1-hiera-small")
        self.box_threshold = float(box_threshold)
        self.text_threshold = float(text_threshold)
        self.max_detections = max(int(max_detections), 1)
        self.device = str(device or "")
        self._hf_processor = None
        self._hf_model = None
        self._sam2_predictor = None
        self._last_ranked_avoid_candidates: List[Dict[str, Any]] = []

    def is_available(self) -> bool:
        readiness = self._readiness()
        return bool(readiness.get("available", False))

    def ground(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> BackendResult:
        target_name = self._resolve_target_name(observation)
        readiness = self._readiness()
        diagnostics = {
            **readiness,
            "repo_path": self.repo_path,
            "model_id": self.model_id,
            "sam2_config": self.sam2_config,
            "sam2_checkpoint": self.sam2_checkpoint,
            "sam2_hf_model_id": self.sam2_hf_model_id,
            "target_name": target_name,
        }
        if not target_name:
            return BackendResult(
                backend_name=self.name,
                available=bool(readiness.get("available", False)),
                ok=False,
                message="missing_target_object_in_task_goal",
                diagnostics=diagnostics,
            )
        if observation.rgb is None:
            return BackendResult(
                backend_name=self.name,
                available=bool(readiness.get("available", False)),
                ok=False,
                message="missing_rgb_observation",
                diagnostics=diagnostics,
            )
        if not readiness.get("available", False):
            return BackendResult(
                backend_name=self.name,
                available=False,
                ok=False,
                message=str(readiness.get("message", "backend_unavailable")),
                diagnostics=diagnostics,
            )

        try:
            processor, model, device = self._ensure_hf_detector()
        except Exception as exc:
            diagnostics["load_error"] = repr(exc)
            return BackendResult(
                backend_name=self.name,
                available=False,
                ok=False,
                message="hf_grounding_load_failed",
                diagnostics=diagnostics,
            )

        try:
            import numpy as np
            import torch
            from PIL import Image
        except Exception as exc:
            diagnostics["runtime_import_error"] = repr(exc)
            return BackendResult(
                backend_name=self.name,
                available=False,
                ok=False,
                message="missing_runtime_dependencies",
                diagnostics=diagnostics,
            )

        image = self._to_pil_image(observation.rgb, Image=Image, np=np)
        prompt = self._format_prompt(target_name)
        results = self._run_grounding_inference(
            processor=processor,
            model=model,
            device=device,
            image=image,
            prompt=prompt,
        )
        if not results:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="no_grounding_results",
                diagnostics={**diagnostics, "prompt": prompt},
            )
        candidates = self._parse_grounding_candidates(results[0], target_name=target_name)
        if not candidates:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="no_candidate_passed_threshold",
                diagnostics={**diagnostics, "prompt": prompt},
            )
        avoid_target_name = self._resolve_avoid_target_name(observation=observation, target_name=target_name)
        avoid_candidates: List[Dict[str, Any]] = []
        if avoid_target_name:
            try:
                avoid_results = self._run_grounding_inference(
                    processor=processor,
                    model=model,
                    device=device,
                    image=image,
                    prompt=self._format_prompt(avoid_target_name),
                )
            except Exception:
                avoid_results = []
            if avoid_results:
                avoid_candidates = self._parse_grounding_candidates(avoid_results[0], target_name=avoid_target_name)

        candidates = self._rerank_candidates(
            candidates,
            observation=observation,
            target_name=target_name,
            avoid_candidates=avoid_candidates,
            avoid_target_name=avoid_target_name,
        )
        ranked_avoid_candidates = list(getattr(self, "_last_ranked_avoid_candidates", []) or [])
        selected = candidates[0]
        payload = GroundingResult(
            target_name=target_name,
            source=self.name,
            score=float(selected["score"]),
            box_xyxy=[float(v) for v in selected["box_xyxy"]],
            mask=selected.get("mask"),
            metadata={
                "prompt": prompt,
                "label": selected["label"],
                "selected_index": int(selected["index"]),
                "candidate_count": len(candidates),
                "geometry_score": float(selected.get("geometry_score", 0.0)),
                "overall_score": float(selected.get("overall_score", selected["score"])),
                "semantic_profile": str(selected.get("semantic_profile", "")),
                "mask_source": str(selected.get("mask_source", "bbox_rect")),
                "mask_foreground_pixels": int(selected.get("mask_foreground_pixels", 0)),
                "mask_foreground_ratio": float(selected.get("mask_foreground_ratio", 0.0)),
                "surface_overlap_ratio": float(selected.get("surface_overlap_ratio", 0.0)),
                "avoid_target_name": str(avoid_target_name or ""),
            },
        )
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=payload,
            diagnostics={
                **diagnostics,
                "prompt": prompt,
                "device": device,
                "candidate_count": len(candidates),
                "avoid_target_name": avoid_target_name,
                "top_candidates": [
                    self._sanitize_candidate_for_diagnostics(row)
                    for row in candidates[: min(len(candidates), self.max_detections)]
                ],
                "avoid_candidates": [
                    self._sanitize_candidate_for_diagnostics(row)
                    for row in ranked_avoid_candidates[: min(len(ranked_avoid_candidates), self.max_detections)]
                ],
            },
        )

    def _run_grounding_inference(self, *, processor: Any, model: Any, device: str, image: Any, prompt: str):
        with __import__("torch").no_grad():
            inputs = processor(images=image, text=prompt, return_tensors="pt").to(device)
            outputs = model(**inputs)
        return processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            target_sizes=[image.size[::-1]],
        )

    def _readiness(self) -> Dict[str, Any]:
        repo_path = _resolve_repo_path(self.repo_path)
        repo_exists = bool(repo_path and repo_path.exists())
        transformers_available = _import_available("transformers")
        torch_available = _import_available("torch")
        pil_available = _import_available("PIL")
        hydra_available = _import_available("hydra")
        iopath_available = _import_available("iopath")
        sam2_checkpoint_path = None if repo_path is None else (repo_path / self.sam2_checkpoint)
        sam2_checkpoint_exists = bool(sam2_checkpoint_path and sam2_checkpoint_path.exists())
        sam2_import_ready = repo_exists and hydra_available and iopath_available and torch_available
        mask_refinement_available = sam2_import_ready and (sam2_checkpoint_exists or bool(self.sam2_hf_model_id))
        available = repo_exists and transformers_available and torch_available and pil_available
        if not repo_exists:
            message = "repo_missing"
        elif not transformers_available:
            message = "missing_dependency_transformers"
        elif not torch_available:
            message = "missing_dependency_torch"
        elif not pil_available:
            message = "missing_dependency_pillow"
        else:
            message = ""
        return {
            "available": available,
            "message": message,
            "repo_exists": repo_exists,
            "resolved_repo_path": None if repo_path is None else str(repo_path),
            "transformers_available": transformers_available,
            "torch_available": torch_available,
            "pil_available": pil_available,
            "hydra_available": hydra_available,
            "iopath_available": iopath_available,
            "sam2_checkpoint_path": None if sam2_checkpoint_path is None else str(sam2_checkpoint_path),
            "sam2_checkpoint_exists": sam2_checkpoint_exists,
            "sam2_import_ready": sam2_import_ready,
            "mask_refinement_available": mask_refinement_available,
        }

    def _ensure_hf_detector(self):
        if self._hf_processor is not None and self._hf_model is not None:
            return self._hf_processor, self._hf_model, self._resolve_device_name()

        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        device = self._resolve_device_name()
        self._hf_processor = AutoProcessor.from_pretrained(self.model_id)
        self._hf_model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(device)
        return self._hf_processor, self._hf_model, device

    def _ensure_sam2_predictor(self):
        if self._sam2_predictor is not None:
            return self._sam2_predictor

        repo_path = _resolve_repo_path(self.repo_path)
        if repo_path is None or not repo_path.exists():
            raise RuntimeError("sam2_repo_missing")
        if str(repo_path) not in sys.path:
            sys.path.insert(0, str(repo_path))

        from sam2.build_sam import build_sam2, build_sam2_hf
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        device = self._resolve_device_name()
        checkpoint_path = repo_path / self.sam2_checkpoint
        if checkpoint_path.exists():
            sam_model = build_sam2(self.sam2_config, str(checkpoint_path), device=device)
        elif self.sam2_hf_model_id:
            sam_model = build_sam2_hf(self.sam2_hf_model_id, device=device)
        else:
            raise RuntimeError("sam2_checkpoint_missing")
        self._sam2_predictor = SAM2ImagePredictor(sam_model)
        return self._sam2_predictor

    def _resolve_device_name(self) -> str:
        if self.device:
            return self.device
        try:
            import torch

            return "cuda" if torch.cuda.is_available() else "cpu"
        except Exception:
            return "cpu"

    @staticmethod
    def _resolve_target_name(observation: PerceptionObservation) -> str:
        task_goal = dict(observation.task_goal or {})
        return str(
            task_goal.get("target_object")
            or task_goal.get("object")
            or task_goal.get("object_name")
            or ""
        ).strip()

    @staticmethod
    def _format_prompt(target_name: str) -> str:
        text = str(target_name or "").strip().lower()
        if text and not text.endswith("."):
            text += "."
        return text

    @staticmethod
    def _to_pil_image(rgb: Any, *, Image: Any, np: Any):
        image = np.asarray(rgb)
        if image.ndim != 3:
            raise ValueError("rgb observation must be HxWxC")
        if image.shape[2] > 3:
            image = image[:, :, :3]
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        return Image.fromarray(image, mode="RGB")

    def _parse_grounding_candidates(
        self,
        result: Dict[str, Any],
        *,
        target_name: str,
    ) -> List[Dict[str, Any]]:
        boxes = result.get("boxes")
        scores = result.get("scores")
        labels = result.get("labels")
        candidates: List[Dict[str, Any]] = []
        if boxes is None or scores is None or labels is None:
            return candidates
        target_norm = str(target_name or "").strip().lower()
        for index, (box, score, label) in enumerate(zip(boxes, scores, labels)):
            if index >= self.max_detections:
                break
            box_list = [float(v) for v in list(getattr(box, "tolist", lambda: list(box))())[:4]]
            label_text = str(label or "")
            label_norm = label_text.strip().lower()
            exact_match = int(label_norm == target_norm)
            substring_match = int(target_norm in label_norm or label_norm in target_norm)
            candidates.append(
                {
                    "index": index,
                    "label": label_text,
                    "score": float(getattr(score, "item", lambda: score)()),
                    "box_xyxy": box_list,
                    "exact_match": exact_match,
                    "substring_match": substring_match,
                }
            )
        candidates.sort(
            key=lambda item: (
                -int(item["exact_match"]),
                -int(item["substring_match"]),
                -float(item["score"]),
            )
        )
        return candidates

    def _rerank_candidates(
        self,
        candidates: Sequence[Dict[str, Any]],
        *,
        observation: PerceptionObservation,
        target_name: str,
        avoid_candidates: Sequence[Dict[str, Any]] | None = None,
        avoid_target_name: str = "",
    ) -> List[Dict[str, Any]]:
        ranked: List[Dict[str, Any]] = []
        profile = self._semantic_profile(target_name=target_name, task_goal=observation.task_goal)
        import numpy as np

        image = None if observation.rgb is None else np.asarray(observation.rgb)
        image_shape = None if image is None or image.ndim < 2 else image.shape[:2]
        avoid_profile = self._semantic_profile(
            target_name=str(avoid_target_name or self._infer_candidate_target_name(avoid_candidates)).strip(),
            task_goal=observation.task_goal,
        )
        prepared_candidates = self._prepare_candidates_for_rerank(
            candidates=candidates,
            observation=observation,
            image_shape=image_shape,
            profile=profile,
        )
        prepared_avoid_candidates = self._prepare_candidates_for_rerank(
            candidates=avoid_candidates or [],
            observation=observation,
            image_shape=image_shape,
            profile=avoid_profile,
        )
        self._last_ranked_avoid_candidates = [dict(item) for item in prepared_avoid_candidates]
        avoid_boxes = [
            list(row.get("box_xyxy") or [])
            for row in prepared_avoid_candidates
            if row.get("box_xyxy")
        ]
        avoid_masks = [row.get("mask") for row in prepared_avoid_candidates]
        for item in prepared_candidates:
            geometry = dict(item.get("geometry") or {})
            geometry_score = self._score_candidate_geometry(
                candidate=item,
                geometry=geometry,
                profile=profile,
            )
            surface_overlap_ratio = self._max_overlap_ratio(
                box_xyxy=list(item.get("box_xyxy") or []),
                other_boxes=avoid_boxes,
                mask=item.get("mask"),
                other_masks=avoid_masks,
            )
            overlap_penalty = 0.0
            if str(profile.get("kind", "generic")) == "upright_container":
                overlap_penalty = min(surface_overlap_ratio, 1.0) * 1.35
            elif str(profile.get("kind", "generic")) == "flat_surface":
                overlap_penalty = -0.35 * min(surface_overlap_ratio, 1.0)
            item["geometry"] = geometry
            item["semantic_profile"] = str(profile.get("kind", "generic"))
            item["geometry_score"] = float(geometry_score)
            item["surface_overlap_ratio"] = float(surface_overlap_ratio)
            item["overall_score"] = (
                float(item.get("score", 0.0))
                + float(geometry_score)
                - float(overlap_penalty)
            )
            ranked.append(item)
        ranked.sort(
            key=lambda item: (
                -float(item.get("overall_score", item.get("score", 0.0))),
                -int(item.get("exact_match", 0)),
                -int(item.get("substring_match", 0)),
                -float(item.get("score", 0.0)),
            )
        )
        for index, item in enumerate(ranked):
            item["rerank_rank"] = int(index)
        return ranked

    def _prepare_candidates_for_rerank(
        self,
        *,
        candidates: Sequence[Dict[str, Any]],
        observation: PerceptionObservation,
        image_shape: tuple[int, int] | None,
        profile: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        prepared: List[Dict[str, Any]] = []
        sam2_masks = self._predict_sam2_masks(observation=observation, candidates=candidates)
        for row in candidates:
            item = dict(row)
            mask_summary = self._refine_candidate_mask(
                observation=observation,
                box_xyxy=list(item.get("box_xyxy") or []),
                profile=profile,
                sam2_mask=sam2_masks.get(int(item.get("index", -1))),
            )
            geometry = self._compute_candidate_geometry(
                observation=observation,
                box_xyxy=list(item.get("box_xyxy") or []),
                image_shape=image_shape,
                refined_mask=mask_summary.get("mask"),
            )
            item["geometry"] = geometry
            item["semantic_profile"] = str(profile.get("kind", "generic"))
            item["mask_source"] = str(mask_summary.get("mask_source", "bbox_rect"))
            item["mask_foreground_pixels"] = int(mask_summary.get("foreground_pixels", 0))
            item["mask_foreground_ratio"] = float(mask_summary.get("foreground_ratio", 0.0))
            item["mask_component_count"] = int(mask_summary.get("component_count", 0))
            item["mask_bbox_xyxy"] = list(mask_summary.get("mask_bbox_xyxy") or [])
            item["mask"] = mask_summary.get("mask")
            prepared.append(item)
        return prepared

    def _compute_candidate_geometry(
        self,
        *,
        observation: PerceptionObservation,
        box_xyxy: Sequence[float],
        image_shape: tuple[int, int] | None,
        refined_mask: Any | None = None,
    ) -> Dict[str, Any]:
        import numpy as np

        geometry: Dict[str, Any] = {}
        if image_shape is None or len(box_xyxy) < 4:
            return geometry
        height, width = int(image_shape[0]), int(image_shape[1])
        x1, y1, x2, y2 = [float(v) for v in list(box_xyxy)[:4]]
        x1i = int(np.clip(np.floor(x1), 0, width - 1))
        y1i = int(np.clip(np.floor(y1), 0, height - 1))
        x2i = int(np.clip(np.ceil(x2), x1i + 1, width))
        y2i = int(np.clip(np.ceil(y2), y1i + 1, height))
        box_w = max(x2i - x1i, 1)
        box_h = max(y2i - y1i, 1)
        box_area = float(box_w * box_h)
        geometry.update(
            {
                "bbox_width_px": box_w,
                "bbox_height_px": box_h,
                "bbox_area_px": box_area,
                "bbox_area_ratio": box_area / max(float(height * width), 1.0),
                "bbox_aspect_hw": float(box_h) / max(float(box_w), 1.0),
                "touches_border": bool(x1i <= 1 or y1i <= 1 or x2i >= width - 1 or y2i >= height - 1),
                "border_margin_px": float(min(x1i, y1i, width - x2i, height - y2i)),
            }
        )
        depth = observation.depth
        if depth is None:
            return geometry
        depth = np.asarray(depth, dtype=np.float64)
        if depth.ndim != 2:
            return geometry

        mask_values = None
        mask = None
        if refined_mask is not None:
            mask = np.asarray(refined_mask, dtype=bool)
            if mask.shape == depth.shape:
                mask = mask & np.isfinite(depth) & (depth > 1.0)
                geometry["mask_foreground_ratio"] = float(np.count_nonzero(mask)) / max(float(box_area), 1.0)
                if mask.any():
                    mask_values = depth[mask]

        if mask_values is None:
            shrink_w = max(int(box_w * 0.1), 0)
            shrink_h = max(int(box_h * 0.1), 0)
            xs = min(max(x1i + shrink_w, 0), width - 1)
            ys = min(max(y1i + shrink_h, 0), height - 1)
            xe = max(min(x2i - shrink_w, width), xs + 1)
            ye = max(min(y2i - shrink_h, height), ys + 1)
            patch = depth[ys:ye, xs:xe]
            valid = np.isfinite(patch) & (patch > 1.0)
            geometry["depth_valid_ratio"] = float(np.count_nonzero(valid)) / max(float(patch.size), 1.0)
            if not valid.any():
                return geometry
            values = patch[valid]
        else:
            geometry["depth_valid_ratio"] = 1.0
            values = mask_values
        scene_valid = np.isfinite(depth) & (depth > 1.0)
        scene_reference = float(np.percentile(depth[scene_valid], 70.0)) if scene_valid.any() else float(np.median(values))
        patch_median = float(np.median(values))
        patch_near = float(np.percentile(values, 20.0))
        patch_far = float(np.percentile(values, 80.0))
        geometry.update(
            {
                "depth_median_mm": patch_median,
                "depth_relief_mm": max(scene_reference - patch_near, 0.0),
                "depth_std_mm": float(np.std(values)),
                "depth_range_mm": max(patch_far - patch_near, 0.0),
            }
        )

        camera_params = dict(observation.metadata or {}).get("camera_params")
        if camera_params is None:
            return geometry
        if mask is not None and mask.any():
            world_stats = self._estimate_world_extent_from_mask(
                depth_full=depth,
                valid_mask=mask,
                camera_params=camera_params,
            )
        else:
            world_stats = self._estimate_world_extent(
                depth_patch=patch,
                valid_mask=valid,
                x_offset=xs,
                y_offset=ys,
                camera_params=camera_params,
            )
        geometry.update(world_stats)
        return geometry

    def _estimate_world_extent(
        self,
        *,
        depth_patch: Any,
        valid_mask: Any,
        x_offset: int,
        y_offset: int,
        camera_params: Dict[str, Any],
    ) -> Dict[str, Any]:
        import numpy as np

        try:
            intrinsic = self._to_numpy(camera_params["intrinsic_cv"])
            cam_to_world = self._resolve_camera_to_world(camera_params)
        except Exception:
            return {}

        ys, xs = np.where(valid_mask)
        if xs.size < 12:
            return {}
        z = np.asarray(depth_patch[ys, xs], dtype=np.float64) / 1000.0
        fx = float(intrinsic[0, 0])
        fy = float(intrinsic[1, 1])
        cx = float(intrinsic[0, 2])
        cy = float(intrinsic[1, 2])
        px = xs.astype(np.float64) + float(x_offset)
        py = ys.astype(np.float64) + float(y_offset)
        x = (px - cx) * z / fx
        y = (py - cy) * z / fy
        points_cam = np.stack([x, y, z, np.ones_like(z)], axis=1)
        points_world = (cam_to_world @ points_cam.T).T[:, :3]
        if points_world.shape[0] > 2000:
            stride = max(points_world.shape[0] // 2000, 1)
            points_world = points_world[::stride]
        lo = np.percentile(points_world, 10.0, axis=0)
        hi = np.percentile(points_world, 90.0, axis=0)
        extents = np.maximum(hi - lo, 0.0)
        lateral_extent = float(max(extents[0], extents[1]))
        vertical_extent = float(extents[2])
        return {
            "world_centroid": [float(v) for v in np.median(points_world, axis=0).tolist()],
            "world_extent_xyz_m": [float(v) for v in extents.tolist()],
            "world_lateral_extent_m": lateral_extent,
            "world_vertical_extent_m": vertical_extent,
            "world_slenderness": vertical_extent / max(lateral_extent, 1e-4),
            "world_flatness": lateral_extent / max(vertical_extent, 1e-4),
        }

    def _estimate_world_extent_from_mask(
        self,
        *,
        depth_full: Any,
        valid_mask: Any,
        camera_params: Dict[str, Any],
    ) -> Dict[str, Any]:
        import numpy as np

        try:
            intrinsic = self._to_numpy(camera_params["intrinsic_cv"])
            cam_to_world = self._resolve_camera_to_world(camera_params)
        except Exception:
            return {}

        ys, xs = np.where(valid_mask)
        if xs.size < 12:
            return {}
        z = np.asarray(depth_full[ys, xs], dtype=np.float64) / 1000.0
        fx = float(intrinsic[0, 0])
        fy = float(intrinsic[1, 1])
        cx = float(intrinsic[0, 2])
        cy = float(intrinsic[1, 2])
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        points_cam = np.stack([x, y, z, np.ones_like(z)], axis=1)
        points_world = (cam_to_world @ points_cam.T).T[:, :3]
        if points_world.shape[0] > 2000:
            stride = max(points_world.shape[0] // 2000, 1)
            points_world = points_world[::stride]
        lo = np.percentile(points_world, 10.0, axis=0)
        hi = np.percentile(points_world, 90.0, axis=0)
        extents = np.maximum(hi - lo, 0.0)
        lateral_extent = float(max(extents[0], extents[1]))
        vertical_extent = float(extents[2])
        return {
            "world_centroid": [float(v) for v in np.median(points_world, axis=0).tolist()],
            "world_extent_xyz_m": [float(v) for v in extents.tolist()],
            "world_lateral_extent_m": lateral_extent,
            "world_vertical_extent_m": vertical_extent,
            "world_slenderness": vertical_extent / max(lateral_extent, 1e-4),
            "world_flatness": lateral_extent / max(vertical_extent, 1e-4),
        }

    def _score_candidate_geometry(
        self,
        *,
        candidate: Dict[str, Any],
        geometry: Dict[str, Any],
        profile: Dict[str, Any],
    ) -> float:
        area_ratio = float(geometry.get("bbox_area_ratio", 0.0))
        depth_relief_norm = min(float(geometry.get("depth_relief_mm", 0.0)) / 120.0, 1.5)
        depth_std_norm = min(float(geometry.get("depth_std_mm", 0.0)) / 60.0, 1.5)
        aspect_hw = float(geometry.get("bbox_aspect_hw", 1.0))
        border_penalty = 0.8 if geometry.get("touches_border", False) else 0.0
        mask_foreground_ratio = float(geometry.get("mask_foreground_ratio", 0.0))
        exact_bonus = 0.25 * float(candidate.get("exact_match", 0))
        substring_bonus = 0.1 * float(candidate.get("substring_match", 0))
        world_slenderness = float(geometry.get("world_slenderness", 0.0))
        world_flatness = float(geometry.get("world_flatness", 0.0))

        kind = str(profile.get("kind", "generic"))
        score = exact_bonus + substring_bonus - border_penalty
        if kind == "upright_container":
            score += 0.85 * depth_relief_norm
            score += 0.45 * depth_std_norm
            score += 0.9 * min(world_slenderness, 2.5) if world_slenderness > 0.0 else 0.35 * min(aspect_hw, 2.0)
            score -= 1.15 * area_ratio
            score += 0.35 * min(mask_foreground_ratio / 0.42, 1.0)
            score -= 0.35 * max(world_flatness - 1.2, 0.0)
        elif kind == "flat_surface":
            score += 0.85 * min(world_flatness, 3.0) if world_flatness > 0.0 else 0.25 / max(aspect_hw, 0.25)
            score += 0.8 * min(area_ratio / 0.12, 1.5)
            score -= 0.55 * depth_relief_norm
            score -= 0.35 * min(world_slenderness, 2.0)
        else:
            score += 0.35 * depth_relief_norm
            score += 0.15 * depth_std_norm
            score -= 0.25 * area_ratio
        return float(score)

    @staticmethod
    def _semantic_profile(target_name: str, task_goal: Dict[str, Any] | None = None) -> Dict[str, Any]:
        target_norm = str(target_name or "").strip().lower()
        task_goal = dict(task_goal or {})
        upright_keywords = {
            "cup",
            "mug",
            "container",
            "bottle",
            "can",
            "jar",
            "vase",
            "thermos",
            "bin",
            "trash",
        }
        flat_keywords = {
            "plate",
            "dish",
            "tray",
            "coaster",
            "mouse_pad",
            "mat",
            "board",
            "pad",
        }
        if any(keyword in target_norm for keyword in upright_keywords):
            return {"kind": "upright_container", "target_name": target_norm}
        if any(keyword in target_norm for keyword in flat_keywords):
            return {"kind": "flat_surface", "target_name": target_norm}
        target_surface = str(task_goal.get("target_surface") or "").strip().lower()
        if target_surface and target_surface != target_norm and any(keyword in target_surface for keyword in flat_keywords):
            return {"kind": "upright_container", "target_name": target_norm}
        return {"kind": "generic", "target_name": target_norm}

    @staticmethod
    def _resolve_avoid_target_name(observation: PerceptionObservation, target_name: str) -> str:
        task_goal = dict(observation.task_goal or {})
        target_surface = str(task_goal.get("target_surface") or "").strip().lower()
        target_norm = str(target_name or "").strip().lower()
        if target_surface and target_surface != target_norm:
            return target_surface
        return ""

    def _predict_sam2_masks(
        self,
        *,
        observation: PerceptionObservation,
        candidates: Sequence[Dict[str, Any]],
    ) -> Dict[int, Any]:
        import numpy as np

        rgb = observation.rgb
        if rgb is None:
            return {}
        boxes: List[List[float]] = []
        candidate_indices: List[int] = []
        for row in candidates:
            box_xyxy = list(row.get("box_xyxy") or [])
            if len(box_xyxy) < 4:
                continue
            candidate_indices.append(int(row.get("index", len(candidate_indices))))
            boxes.append([float(v) for v in box_xyxy[:4]])
        if not boxes:
            return {}
        try:
            predictor = self._ensure_sam2_predictor()
        except Exception:
            return {}
        image = np.asarray(rgb)
        if image.ndim != 3:
            return {}
        if image.shape[2] > 3:
            image = image[:, :, :3]
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        predictor.set_image(image)
        masks, _, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=np.asarray(boxes, dtype=np.float32),
            multimask_output=False,
            return_logits=False,
        )
        mask_array = np.asarray(masks)
        if mask_array.ndim == 4:
            mask_array = mask_array.squeeze(1)
        outputs: Dict[int, Any] = {}
        if mask_array.ndim != 3:
            return outputs
        for candidate_index, mask in zip(candidate_indices, mask_array):
            outputs[int(candidate_index)] = np.asarray(mask, dtype=bool)
        return outputs

    def _refine_candidate_mask(
        self,
        *,
        observation: PerceptionObservation,
        box_xyxy: Sequence[float],
        profile: Dict[str, Any],
        sam2_mask: Any | None = None,
    ) -> Dict[str, Any]:
        import numpy as np

        depth = observation.depth
        shape = None
        if depth is not None:
            depth = np.asarray(depth, dtype=np.float64)
            shape = tuple(depth.shape[:2]) if depth.ndim >= 2 else None
        rect_mask = self._build_rect_mask(shape=shape, box_xyxy=box_xyxy)

        if sam2_mask is not None and shape is not None:
            sam2_summary = self._summarize_sam2_mask(
                mask=sam2_mask,
                shape=shape,
                depth=depth,
                rect_mask=rect_mask,
                box_xyxy=box_xyxy,
                profile=profile,
            )
            if sam2_summary is not None:
                return sam2_summary

        if depth is None:
            return self._mask_summary_from_box(shape=shape, box_xyxy=box_xyxy, mask=rect_mask, source="bbox_rect")
        if depth.ndim != 2:
            return self._mask_summary_from_box(shape=depth.shape, box_xyxy=box_xyxy, mask=rect_mask, source="bbox_rect")
        valid = rect_mask & np.isfinite(depth) & (depth > 1.0)
        if np.count_nonzero(valid) < 16:
            return self._mask_summary_from_box(shape=depth.shape, box_xyxy=box_xyxy, mask=rect_mask, source="bbox_rect")

        local_values = depth[valid]
        near = float(np.percentile(local_values, 15.0))
        far = float(np.percentile(local_values, 85.0))
        cutoff = near + max((far - near) * 0.38, 25.0)
        local_foreground = valid & (depth <= cutoff)
        refined = self._clean_mask(local_foreground)
        component_rows = self._rank_mask_components(mask=refined, depth=depth, box_xyxy=box_xyxy, profile=profile)
        if not component_rows:
            return self._mask_summary_from_box(shape=depth.shape, box_xyxy=box_xyxy, mask=rect_mask, source="bbox_rect")
        selected = component_rows[0]
        selected_mask = selected.get("mask")
        if selected_mask is None or np.count_nonzero(selected_mask) < 12:
            return self._mask_summary_from_box(shape=depth.shape, box_xyxy=box_xyxy, mask=rect_mask, source="bbox_rect")
        summary = self._mask_summary_from_box(
            shape=depth.shape,
            box_xyxy=box_xyxy,
            mask=np.asarray(selected_mask, dtype=bool),
            source="depth_refined_component",
        )
        summary["component_count"] = len(component_rows)
        return summary

    def _summarize_sam2_mask(
        self,
        *,
        mask: Any,
        shape: tuple[int, int],
        depth: Any | None,
        rect_mask: Any,
        box_xyxy: Sequence[float],
        profile: Dict[str, Any],
    ) -> Dict[str, Any] | None:
        import numpy as np

        sam2_mask = np.asarray(mask, dtype=bool)
        if sam2_mask.ndim == 3:
            sam2_mask = np.squeeze(sam2_mask, axis=0)
        if sam2_mask.shape != shape:
            return None
        sam2_mask = sam2_mask & np.asarray(rect_mask, dtype=bool)
        if depth is not None and np.asarray(depth).shape == shape:
            valid = np.isfinite(depth) & (depth > 1.0)
            sam2_mask = sam2_mask & valid
        if np.count_nonzero(sam2_mask) < 12:
            return None
        refined = self._clean_mask(sam2_mask)
        component_rows = []
        if depth is not None and np.asarray(depth).shape == shape:
            component_rows = self._rank_mask_components(
                mask=refined,
                depth=np.asarray(depth, dtype=np.float64),
                box_xyxy=box_xyxy,
                profile=profile,
            )
        if component_rows:
            selected_mask = np.asarray(component_rows[0].get("mask"), dtype=bool)
            source = "sam2_instance_mask"
            component_count = len(component_rows)
        else:
            selected_mask = np.asarray(refined, dtype=bool)
            source = "sam2_instance_mask"
            component_count = 1
        summary = self._mask_summary_from_box(
            shape=shape,
            box_xyxy=box_xyxy,
            mask=selected_mask,
            source=source,
        )
        summary["component_count"] = int(component_count)
        return summary

    def _rank_mask_components(
        self,
        *,
        mask: Any,
        depth: Any,
        box_xyxy: Sequence[float],
        profile: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        import cv2
        import numpy as np

        mask_uint8 = np.asarray(mask, dtype=np.uint8)
        if mask_uint8.ndim != 2 or np.count_nonzero(mask_uint8) == 0:
            return []
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_uint8, connectivity=8)
        if num_labels <= 1:
            return []
        x1, y1, x2, y2 = [float(v) for v in list(box_xyxy)[:4]]
        box_cx = 0.5 * (x1 + x2)
        box_cy = 0.5 * (y1 + y2)
        box_area = max((x2 - x1) * (y2 - y1), 1.0)
        kind = str(profile.get("kind", "generic"))
        rows: List[Dict[str, Any]] = []
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 12:
                continue
            left = int(stats[label, cv2.CC_STAT_LEFT])
            top = int(stats[label, cv2.CC_STAT_TOP])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            component_mask = labels == label
            values = depth[component_mask]
            if values.size == 0:
                continue
            median_depth = float(np.median(values))
            area_ratio = float(area) / max(float(box_area), 1.0)
            center_x = left + width * 0.5
            center_y = top + height * 0.5
            center_penalty = abs(center_x - box_cx) / max(float(x2 - x1), 1.0)
            center_penalty += 0.75 * abs(center_y - box_cy) / max(float(y2 - y1), 1.0)
            border_penalty = 0.0
            if left <= int(x1) + 1 or top <= int(y1) + 1 or left + width >= int(x2) - 1 or top + height >= int(y2) - 1:
                border_penalty = 0.35
            score = (1.0 - median_depth / max(float(np.percentile(depth[np.isfinite(depth) & (depth > 1.0)], 95.0)), 1.0)) * 2.0
            score += min(area_ratio / 0.28, 1.2)
            score -= center_penalty
            score -= border_penalty
            if kind == "upright_container":
                aspect_hw = float(height) / max(float(width), 1.0)
                score += 0.4 * min(aspect_hw, 2.0)
                score -= 0.7 * max(area_ratio - 0.72, 0.0)
            rows.append(
                {
                    "label": int(label),
                    "score": float(score),
                    "area": area,
                    "bbox": [left, top, width, height],
                    "mask": component_mask,
                }
            )
        rows.sort(key=lambda item: (-float(item.get("score", 0.0)), -int(item.get("area", 0))))
        return rows

    @staticmethod
    def _clean_mask(mask: Any):
        import cv2
        import numpy as np

        mask_uint8 = np.asarray(mask, dtype=np.uint8) * 255
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask_uint8 = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_uint8 = cv2.morphologyEx(mask_uint8, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask_uint8 > 0

    @staticmethod
    def _build_rect_mask(*, shape: tuple[int, int] | None, box_xyxy: Sequence[float]) -> Any:
        if shape is None:
            return None
        import numpy as np

        height, width = int(shape[0]), int(shape[1])
        mask = np.zeros((height, width), dtype=bool)
        if len(list(box_xyxy)) < 4:
            return mask
        x1, y1, x2, y2 = [int(round(float(v))) for v in list(box_xyxy)[:4]]
        x1 = max(min(x1, width - 1), 0)
        y1 = max(min(y1, height - 1), 0)
        x2 = max(min(x2, width), x1 + 1)
        y2 = max(min(y2, height), y1 + 1)
        mask[y1:y2, x1:x2] = True
        return mask

    def _mask_summary_from_box(
        self,
        *,
        shape: tuple[int, int] | None,
        box_xyxy: Sequence[float],
        mask: Any,
        source: str,
    ) -> Dict[str, Any]:
        import numpy as np

        actual_mask = mask
        if actual_mask is None and shape is not None:
            actual_mask = self._build_rect_mask(shape=shape, box_xyxy=box_xyxy)
        foreground_pixels = int(np.count_nonzero(actual_mask)) if actual_mask is not None else 0
        bbox_area = 0.0
        if len(list(box_xyxy)) >= 4:
            x1, y1, x2, y2 = [float(v) for v in list(box_xyxy)[:4]]
            bbox_area = max((x2 - x1) * (y2 - y1), 1.0)
        mask_bbox_xyxy: List[int] = []
        if actual_mask is not None and foreground_pixels > 0:
            ys, xs = np.where(actual_mask)
            mask_bbox_xyxy = [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]
        return {
            "mask": actual_mask,
            "mask_source": str(source),
            "foreground_pixels": foreground_pixels,
            "foreground_ratio": float(foreground_pixels) / max(float(bbox_area), 1.0),
            "mask_bbox_xyxy": mask_bbox_xyxy,
            "component_count": 0,
        }

    @staticmethod
    def _max_overlap_ratio(
        *,
        box_xyxy: Sequence[float],
        other_boxes: Sequence[Sequence[float]],
        mask: Any | None = None,
        other_masks: Sequence[Any | None] | None = None,
    ) -> float:
        import numpy as np

        base_mask = None if mask is None else np.asarray(mask, dtype=bool)
        use_mask = base_mask is not None and base_mask.ndim == 2 and np.count_nonzero(base_mask) > 0
        x1, y1, x2, y2 = [float(v) for v in list(box_xyxy)[:4]]
        base_area = max((x2 - x1) * (y2 - y1), 1.0)
        best = 0.0
        other_masks = list(other_masks or [])
        for index, other in enumerate(other_boxes):
            other_mask = other_masks[index] if index < len(other_masks) else None
            if use_mask and other_mask is not None:
                other_mask_array = np.asarray(other_mask, dtype=bool)
                if other_mask_array.shape == base_mask.shape and np.count_nonzero(other_mask_array) > 0:
                    inter = int(np.count_nonzero(base_mask & other_mask_array))
                    best = max(best, inter / max(float(np.count_nonzero(base_mask)), 1.0))
                    continue
            if len(list(other)) < 4:
                continue
            ox1, oy1, ox2, oy2 = [float(v) for v in list(other)[:4]]
            ix1 = max(x1, ox1)
            iy1 = max(y1, oy1)
            ix2 = min(x2, ox2)
            iy2 = min(y2, oy2)
            if ix2 <= ix1 or iy2 <= iy1:
                continue
            inter = (ix2 - ix1) * (iy2 - iy1)
            best = max(best, inter / base_area)
        return float(best)

    @staticmethod
    def _sanitize_candidate_for_diagnostics(candidate: Dict[str, Any]) -> Dict[str, Any]:
        sanitized = dict(candidate)
        mask = sanitized.get("mask")
        if mask is not None:
            sanitized["mask_outline_xy"] = GroundedSAM2Grounder._mask_outline_points(mask)
        sanitized.pop("mask", None)
        return sanitized

    @staticmethod
    def _mask_outline_points(mask: Any, *, max_points: int = 64) -> List[List[int]]:
        import numpy as np

        mask_array = np.asarray(mask, dtype=np.uint8)
        if mask_array.ndim != 2 or np.count_nonzero(mask_array) == 0:
            return []
        try:
            import cv2

            contours, _ = cv2.findContours(mask_array, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return []
            contour = max(contours, key=cv2.contourArea)
            epsilon = max(1.5, 0.01 * cv2.arcLength(contour, True))
            simplified = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
            if simplified.shape[0] > max_points:
                stride = max(int(np.ceil(simplified.shape[0] / max_points)), 1)
                simplified = simplified[::stride]
            return [[int(point[0]), int(point[1])] for point in simplified.tolist()]
        except Exception:
            ys, xs = np.where(mask_array > 0)
            if xs.size == 0:
                return []
            return [
                [int(xs.min()), int(ys.min())],
                [int(xs.max()), int(ys.min())],
                [int(xs.max()), int(ys.max())],
                [int(xs.min()), int(ys.max())],
            ]

    @staticmethod
    def _infer_candidate_target_name(candidates: Sequence[Dict[str, Any]] | None) -> str:
        for row in list(candidates or []):
            label = str(row.get("label") or "").strip().lower()
            if label:
                return label
        return ""

    @staticmethod
    def _to_numpy(value: Any):
        return _to_numpy(value)

    @classmethod
    def _resolve_camera_to_world(cls, camera_params: Dict[str, Any]):
        return _resolve_camera_to_world(camera_params)


class FoundationPoseEstimator(ObjectPoseEstimator):
    name = "foundationpose"

    def __init__(self, repo_path: str | None = None, python_bin: str | None = None, timeout_s: int = 180):
        self.repo_path = str(repo_path or "")
        self.python_bin = _resolve_python_bin(python_bin)
        self.timeout_s = max(int(timeout_s), 1)

    def is_available(self) -> bool:
        readiness = self._readiness()
        return bool(readiness.get("available", False))

    def estimate_pose(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
    ) -> BackendResult:
        readiness = self._readiness()
        if not readiness.get("available", False):
            return BackendResult(
                backend_name=self.name,
                available=False,
                ok=False,
                message=str(readiness.get("message", "integration_not_implemented")),
                diagnostics={
                    **readiness,
                    "repo_path": self.repo_path,
                    "python_bin": self.python_bin,
                    "target_name": None if grounding is None else grounding.target_name,
                },
            )
        export_dir = _backend_run_dir(context, self.name)
        export_outputs = self._export_runtime_input(
            export_dir=export_dir,
            observation=observation,
            context=context,
            grounding=grounding,
        )
        if not export_outputs.get("ok", False):
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message=str(export_outputs.get("message", "export_failed")),
                diagnostics={**readiness, "export_outputs": export_outputs, "python_bin": self.python_bin},
            )
        command = self._build_command(
            repo_path=str(readiness.get("resolved_repo_path") or ""),
            mesh_path=str(export_outputs.get("mesh_file") or ""),
            test_scene_dir=str(export_outputs.get("test_scene_dir") or ""),
            debug_dir=str(export_outputs.get("debug_dir") or ""),
        )
        env = {
            "CUDA_HOME": str(Path("/usr/local/cuda").resolve()) if Path("/usr/local/cuda").exists() else "",
        }
        try:
            result = subprocess.run(
                command,
                cwd=str(readiness.get("resolved_repo_path") or "") or None,
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
                check=False,
                env={**os.environ, **{k: v for k, v in env.items() if v}},
            )
        except Exception as exc:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="subprocess_error",
                diagnostics={
                    **readiness,
                    "python_bin": self.python_bin,
                    "export_outputs": export_outputs,
                    "command": command,
                    "exception": repr(exc),
                },
            )
        pose_path = export_outputs["debug_dir"] / "ob_in_cam" / "000000.txt"
        if result.returncode != 0 or not pose_path.is_file():
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="runtime_failed" if result.returncode != 0 else "pose_output_missing",
                diagnostics={
                    **readiness,
                    "python_bin": self.python_bin,
                    "export_outputs": {
                        **export_outputs,
                        "debug_dir": str(export_outputs["debug_dir"]),
                        "test_scene_dir": str(export_outputs["test_scene_dir"]),
                    },
                    "command": command,
                    "returncode": int(result.returncode),
                    "stdout_tail": (result.stdout or "")[-4000:],
                    "stderr_tail": (result.stderr or "")[-4000:],
                },
            )
        try:
            import numpy as np

            pose_in_cam = np.loadtxt(pose_path).reshape(4, 4)
            cam_to_world = _resolve_camera_to_world(dict(observation.metadata or {}).get("camera_params") or {})
            pose_in_world = cam_to_world @ pose_in_cam
            pose = [float(v) for v in pose_in_world[:3, 3].tolist()] + _rotation_matrix_to_quat_wxyz(pose_in_world[:3, :3])
        except Exception as exc:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="pose_parse_failed",
                diagnostics={
                    **readiness,
                    "python_bin": self.python_bin,
                    "export_outputs": {
                        **export_outputs,
                        "debug_dir": str(export_outputs["debug_dir"]),
                        "test_scene_dir": str(export_outputs["test_scene_dir"]),
                    },
                    "command": command,
                    "returncode": int(result.returncode),
                    "stdout_tail": (result.stdout or "")[-4000:],
                    "stderr_tail": (result.stderr or "")[-4000:],
                    "exception": repr(exc),
                },
            )
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=pose,
            message="runtime_success",
            diagnostics={
                **readiness,
                "python_bin": self.python_bin,
                "repo_path": self.repo_path,
                "target_name": None if grounding is None else grounding.target_name,
                "command": command,
                "pose_path": str(pose_path),
                "test_scene_dir": str(export_outputs["test_scene_dir"]),
                "debug_dir": str(export_outputs["debug_dir"]),
                "stdout_tail": (result.stdout or "")[-4000:],
                "stderr_tail": (result.stderr or "")[-4000:],
            },
        )

    def _readiness(self) -> Dict[str, Any]:
        repo_path = _resolve_repo_path(self.repo_path)
        repo_exists = bool(repo_path and repo_path.exists())
        weight_dirs = _find_foundationpose_weight_dirs(repo_path)
        required_runs = {
            "2023-10-28-18-33-37": weight_dirs.get("2023-10-28-18-33-37", ""),
            "2024-01-11-20-02-45": weight_dirs.get("2024-01-11-20-02-45", ""),
        }
        demo_data = _find_matching_files(repo_path, ["demo_data/**/*"])
        import_status = _python_import_status(
            ["open3d", "trimesh", "pytorch3d", "nvdiffrast", "omegaconf"],
            python_bin=self.python_bin,
        )
        module_status = dict(import_status.get("module_status") or {})
        open3d_available = bool(module_status.get("open3d", False))
        trimesh_available = bool(module_status.get("trimesh", False))
        pytorch3d_available = bool(module_status.get("pytorch3d", False))
        nvdiffrast_available = bool(module_status.get("nvdiffrast", False))
        omegaconf_available = bool(module_status.get("omegaconf", False))
        blockers: List[str] = []
        if not repo_exists:
            blockers.append("repo_missing")
        if not import_status.get("python_bin_exists", False):
            blockers.append("python_bin_missing")
        if not weight_dirs:
            blockers.append("weights_missing")
        else:
            if not required_runs["2023-10-28-18-33-37"]:
                blockers.append("missing_refiner_weights")
            if not required_runs["2024-01-11-20-02-45"]:
                blockers.append("missing_scorer_weights")
        if not open3d_available:
            blockers.append("missing_dependency_open3d")
        if not trimesh_available:
            blockers.append("missing_dependency_trimesh")
        if not pytorch3d_available:
            blockers.append("missing_dependency_pytorch3d")
        if not nvdiffrast_available:
            blockers.append("missing_dependency_nvdiffrast")
        if not omegaconf_available:
            blockers.append("missing_dependency_omegaconf")
        available = not blockers
        if blockers:
            message = blockers[0]
        else:
            message = "ready_for_external_run"
        return {
            "available": available,
            "message": message,
            "blockers": blockers,
            "python_bin": self.python_bin,
            "python_import_status": import_status,
            "repo_exists": repo_exists,
            "resolved_repo_path": None if repo_path is None else str(repo_path),
            "weight_dirs": weight_dirs,
            "required_weight_dirs": required_runs,
            "demo_data_found": bool(demo_data),
            "open3d_available": open3d_available,
            "trimesh_available": trimesh_available,
            "pytorch3d_available": pytorch3d_available,
            "nvdiffrast_available": nvdiffrast_available,
            "omegaconf_available": omegaconf_available,
        }

    def _export_runtime_input(
        self,
        *,
        export_dir: Path,
        observation: PerceptionObservation,
        context: Any | None,
        grounding: GroundingResult | None,
    ) -> Dict[str, Any]:
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            return {"ok": False, "message": f"missing_runtime_dependency:{exc!r}"}

        rgb = None if observation.rgb is None else np.asarray(observation.rgb)
        depth = None if observation.depth is None else np.asarray(observation.depth, dtype=np.float64)
        if rgb is None or depth is None:
            return {"ok": False, "message": "missing_snapshot_rgb_or_depth"}
        sdk = None if context is None else getattr(context, "adapters", {}).get("sdk")
        asset_info = sdk.get_object_asset_info() if sdk is not None and hasattr(sdk, "get_object_asset_info") else {}
        source_mesh_file = str(asset_info.get("visual_mesh_path") or asset_info.get("collision_mesh_path") or "")
        if not source_mesh_file:
            return {"ok": False, "message": "missing_mesh_file", "asset_info": asset_info}
        mesh_outputs = self._prepare_mesh_file_for_runtime(source_mesh_file=source_mesh_file, export_dir=export_dir)
        if not mesh_outputs.get("ok", False):
            return {
                "ok": False,
                "message": str(mesh_outputs.get("message", "mesh_prepare_failed")),
                "asset_info": asset_info,
                "mesh_outputs": mesh_outputs,
            }
        mesh_file = str(mesh_outputs.get("mesh_file") or "")
        rgb_dir = export_dir / "rgb"
        depth_dir = export_dir / "depth"
        mask_dir = export_dir / "masks"
        debug_dir = export_dir / "foundationpose_debug"
        for path in (rgb_dir, depth_dir, mask_dir, debug_dir):
            path.mkdir(parents=True, exist_ok=True)
        rgb_path = rgb_dir / "000000.png"
        depth_path = depth_dir / "000000.png"
        mask_path = mask_dir / "000000.png"
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        if rgb.ndim == 3 and rgb.shape[2] > 3:
            rgb = rgb[:, :, :3]
        cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(depth_path), np.clip(depth, 0, np.iinfo(np.uint16).max).astype(np.uint16))
        mask, mask_source = self._resolve_grounding_mask(shape=depth.shape, grounding=grounding)
        cv2.imwrite(str(mask_path), (mask.astype(np.uint8) * 255))
        camera_params = dict(observation.metadata or {}).get("camera_params") or {}
        intrinsic = _to_numpy(camera_params["intrinsic_cv"]).reshape(3, 3)
        cam_k_path = export_dir / "cam_K.txt"
        np.savetxt(cam_k_path, intrinsic)
        metadata_path = export_dir / "metadata.json"
        metadata_path.write_text(
            json.dumps(
                {
                    "source_mesh_file": source_mesh_file,
                    "mesh_file": mesh_file,
                    "mask_source": mask_source,
                    "asset_info": asset_info,
                    "camera_params_available": bool(camera_params),
                    "mesh_outputs": mesh_outputs,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        return {
            "ok": True,
            "mesh_file": mesh_file,
            "source_mesh_file": source_mesh_file,
            "test_scene_dir": export_dir,
            "debug_dir": debug_dir,
            "rgb_path": str(rgb_path),
            "depth_path": str(depth_path),
            "mask_path": str(mask_path),
            "cam_k_path": str(cam_k_path),
            "metadata_path": str(metadata_path),
            "mask_source": mask_source,
            "mesh_outputs": mesh_outputs,
        }

    def _prepare_mesh_file_for_runtime(self, *, source_mesh_file: str, export_dir: Path) -> Dict[str, Any]:
        source_path = Path(str(source_mesh_file)).expanduser()
        if not source_path.is_absolute():
            source_path = (_repo_root() / source_path).resolve()
        if not source_path.is_file():
            return {"ok": False, "message": "mesh_file_missing", "source_mesh_file": str(source_path)}
        if source_path.suffix.lower() in {".obj", ".ply", ".stl"}:
            return {
                "ok": True,
                "mesh_file": str(source_path),
                "source_mesh_file": str(source_path),
                "prepared": False,
            }
        prepared_mesh_path = export_dir / "foundationpose_mesh.obj"
        script = """
import sys
from pathlib import Path
import trimesh

src = Path(sys.argv[1]).expanduser().resolve()
dst = Path(sys.argv[2]).expanduser().resolve()
dst.parent.mkdir(parents=True, exist_ok=True)
mesh = trimesh.load(str(src), force='scene')
if isinstance(mesh, trimesh.Scene):
    geometries = [geom for geom in mesh.geometry.values()]
    if not geometries:
        raise RuntimeError('scene_geometry_empty')
    mesh = geometries[0].copy() if len(geometries) == 1 else trimesh.util.concatenate(tuple(geometries))
if not isinstance(mesh, trimesh.Trimesh):
    raise RuntimeError(f'unsupported_mesh_type:{type(mesh).__name__}')
mesh.remove_unreferenced_vertices()
mesh.export(str(dst))
print(dst)
""".strip()
        try:
            result = subprocess.run(
                [self.python_bin, "-c", script, str(source_path), str(prepared_mesh_path)],
                capture_output=True,
                text=True,
                timeout=max(self.timeout_s, 30),
                check=False,
            )
        except Exception as exc:
            return {
                "ok": False,
                "message": "mesh_prepare_subprocess_error",
                "source_mesh_file": str(source_path),
                "prepared_mesh_path": str(prepared_mesh_path),
                "exception": repr(exc),
            }
        if result.returncode != 0 or not prepared_mesh_path.is_file():
            return {
                "ok": False,
                "message": "mesh_prepare_failed" if result.returncode != 0 else "prepared_mesh_missing",
                "source_mesh_file": str(source_path),
                "prepared_mesh_path": str(prepared_mesh_path),
                "returncode": int(result.returncode),
                "stdout_tail": (result.stdout or "")[-4000:],
                "stderr_tail": (result.stderr or "")[-4000:],
            }
        return {
            "ok": True,
            "mesh_file": str(prepared_mesh_path),
            "source_mesh_file": str(source_path),
            "prepared": True,
            "stdout_tail": (result.stdout or "")[-4000:],
            "stderr_tail": (result.stderr or "")[-4000:],
        }

    def _build_command(self, *, repo_path: str, mesh_path: str, test_scene_dir: str, debug_dir: str) -> List[str]:
        return [
            self.python_bin,
            str((Path(repo_path).expanduser().resolve() / "run_demo.py")),
            "--mesh_file",
            str(Path(mesh_path).expanduser().resolve()),
            "--test_scene_dir",
            str(Path(test_scene_dir).expanduser().resolve()),
            "--debug",
            "0",
            "--debug_dir",
            str(Path(debug_dir).expanduser().resolve()),
        ]

    @staticmethod
    def _build_bbox_mask(*, shape: tuple[int, ...], box_xyxy: Any):
        import numpy as np

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

    def _resolve_grounding_mask(self, *, shape: tuple[int, ...], grounding: GroundingResult | None):
        import numpy as np

        if grounding is not None:
            mask = getattr(grounding, "mask", None)
            if mask is not None:
                mask_array = np.asarray(mask, dtype=bool)
                if mask_array.shape == tuple(shape):
                    return mask_array, str(dict(getattr(grounding, "metadata", {}) or {}).get("mask_source", "grounding_mask"))
            box_xyxy = getattr(grounding, "box_xyxy", None)
        else:
            box_xyxy = None
        return self._build_bbox_mask(shape=shape, box_xyxy=box_xyxy), "grounding_bbox_rect"


class DelegatePoseEstimator(ObjectPoseEstimator):
    def __init__(self, name: str, delegate: Any, source_hint: str = ""):
        self.name = str(name)
        self.delegate = delegate
        self.source_hint = str(source_hint or self.name)

    def is_available(self) -> bool:
        return self.delegate is not None and hasattr(self.delegate, "get_object_pose")

    def estimate_pose(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
    ) -> BackendResult:
        if not self.is_available():
            return BackendResult(backend_name=self.name, available=False, ok=False, message="delegate_unavailable")
        try:
            pose = self.delegate.get_object_pose(observation, context=context)
        except TypeError:
            pose = self.delegate.get_object_pose()
        if pose is None:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="pose_unavailable",
            )
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=list(pose),
            diagnostics={"source_hint": self.source_hint},
        )


class ContactGraspNetBackend(GraspProposalBackend):
    name = "contact_graspnet"

    def __init__(
        self,
        repo_path: str | None = None,
        python_bin: str | None = None,
        timeout_s: int = 180,
        max_candidates: int = 12,
        template_delegate: Any | None = None,
    ):
        self.repo_path = str(repo_path or "")
        self.python_bin = _resolve_python_bin(python_bin)
        self.timeout_s = max(int(timeout_s), 1)
        self.max_candidates = max(int(max_candidates), 1)
        self.template_delegate = template_delegate
        self.last_template_source_debug: Dict[str, Any] = {}

    @staticmethod
    def _needs_relaxed_retry(summary: Dict[str, Any] | None) -> bool:
        payload = dict(summary or {})
        return int(payload.get("grasp_group_count") or 0) > 0 and int(payload.get("grasp_total") or 0) <= 0

    def _run_headless_attempt(
        self,
        *,
        readiness: Dict[str, Any],
        export_outputs: Dict[str, Any],
        output_dir: Path,
        local_regions: bool,
        filter_grasps: bool,
        label: str,
    ) -> Dict[str, Any]:
        command = self._build_command(
            repo_path=str(readiness.get("resolved_repo_path") or ""),
            npz_path=str(export_outputs.get("npz_path") or ""),
            ckpt_dir=str(readiness.get("preferred_checkpoint_dir") or ""),
            out_dir=str(output_dir),
            local_regions=local_regions,
            filter_grasps=filter_grasps,
        )
        try:
            completed = subprocess.run(
                command,
                cwd=str(readiness.get("resolved_repo_path") or "") or None,
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
                check=False,
            )
        except Exception as exc:
            return {
                "ok": False,
                "label": label,
                "local_regions": bool(local_regions),
                "filter_grasps": bool(filter_grasps),
                "command": command,
                "output_dir": str(output_dir),
                "exception": repr(exc),
            }

        summary_path = output_dir / "contact_graspnet_summary.json"
        summary: Dict[str, Any] = {}
        if summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except Exception:
                summary = {}
        return {
            "ok": completed.returncode == 0 and summary_path.is_file(),
            "label": label,
            "local_regions": bool(local_regions),
            "filter_grasps": bool(filter_grasps),
            "command": command,
            "output_dir": str(output_dir),
            "summary_path": str(summary_path),
            "summary": summary,
            "returncode": int(completed.returncode),
            "stdout_tail": (completed.stdout or "")[-4000:],
            "stderr_tail": (completed.stderr or "")[-4000:],
        }

    def is_available(self) -> bool:
        readiness = self._readiness()
        return bool(readiness.get("available", False))

    def propose_grasps(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> BackendResult:
        readiness = self._readiness()
        if not readiness.get("available", False):
            return BackendResult(
                backend_name=self.name,
                available=False,
                ok=False,
                message=str(readiness.get("message", "integration_not_implemented")),
                diagnostics={**readiness, "repo_path": self.repo_path, "python_bin": self.python_bin},
            )
        export_dir = _backend_run_dir(context, self.name)
        export_outputs = self._export_runtime_input(export_dir=export_dir, observation=observation, grounding=grounding)
        if not export_outputs.get("ok", False):
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message=str(export_outputs.get("message", "export_failed")),
                diagnostics={**readiness, "python_bin": self.python_bin, "export_outputs": export_outputs},
            )
        attempt_plan = [
            {
                "label": "strict",
                "output_dir": export_dir / "contact_graspnet_headless",
                "local_regions": True,
                "filter_grasps": True,
            },
            {
                "label": "retry_no_filter",
                "output_dir": export_dir / "contact_graspnet_headless_nofilter",
                "local_regions": True,
                "filter_grasps": False,
            },
            {
                "label": "retry_global",
                "output_dir": export_dir / "contact_graspnet_headless_global",
                "local_regions": False,
                "filter_grasps": False,
            },
        ]
        attempts: List[Dict[str, Any]] = []
        selected_attempt: Dict[str, Any] | None = None
        for index, attempt_cfg in enumerate(attempt_plan):
            attempt = self._run_headless_attempt(
                readiness=readiness,
                export_outputs=export_outputs,
                output_dir=Path(attempt_cfg["output_dir"]),
                local_regions=bool(attempt_cfg["local_regions"]),
                filter_grasps=bool(attempt_cfg["filter_grasps"]),
                label=str(attempt_cfg["label"]),
            )
            attempts.append(
                {
                    "label": attempt.get("label"),
                    "local_regions": attempt.get("local_regions"),
                    "filter_grasps": attempt.get("filter_grasps"),
                    "returncode": attempt.get("returncode"),
                    "summary_path": attempt.get("summary_path"),
                    "output_dir": attempt.get("output_dir"),
                    "grasp_group_count": int(dict(attempt.get("summary") or {}).get("grasp_group_count") or 0),
                    "grasp_total": int(dict(attempt.get("summary") or {}).get("grasp_total") or 0),
                }
            )
            selected_attempt = attempt
            if not attempt.get("ok", False):
                break
            if index == 0 and self._needs_relaxed_retry(dict(attempt.get("summary") or {})):
                continue
            if index == 1 and self._needs_relaxed_retry(dict(attempt.get("summary") or {})):
                continue
            break

        selected_attempt = dict(selected_attempt or {})
        if not selected_attempt.get("ok", False):
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="runtime_failed" if int(selected_attempt.get("returncode", -1)) != 0 else "summary_missing",
                diagnostics={
                    **readiness,
                    "python_bin": self.python_bin,
                    "export_outputs": export_outputs,
                    "command": selected_attempt.get("command"),
                    "returncode": int(selected_attempt.get("returncode", -1)),
                    "stdout_tail": str(selected_attempt.get("stdout_tail") or ""),
                    "stderr_tail": str(selected_attempt.get("stderr_tail") or ""),
                    "attempts": attempts,
                    "selected_attempt": selected_attempt.get("label"),
                    "exception": selected_attempt.get("exception"),
                },
            )

        output_dir = Path(str(selected_attempt.get("output_dir") or ""))
        summary_path = Path(str(selected_attempt.get("summary_path") or ""))
        summary = dict(selected_attempt.get("summary") or {})
        try:
            candidates = self._load_runtime_candidates(
                output_dir=output_dir,
                observation=observation,
                context=context,
                object_pose=object_pose,
            )
        except Exception as exc:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="candidate_parse_failed",
                diagnostics={
                    **readiness,
                    "python_bin": self.python_bin,
                    "export_outputs": export_outputs,
                    "command": selected_attempt.get("command"),
                    "returncode": int(selected_attempt.get("returncode", -1)),
                    "stdout_tail": str(selected_attempt.get("stdout_tail") or ""),
                    "stderr_tail": str(selected_attempt.get("stderr_tail") or ""),
                    "summary_path": str(summary_path),
                    "attempts": attempts,
                    "selected_attempt": selected_attempt.get("label"),
                    "exception": repr(exc),
                },
            )
        if not candidates:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="no_runtime_candidates",
                diagnostics={
                    **readiness,
                    "python_bin": self.python_bin,
                    "summary": summary,
                    "summary_path": str(summary_path),
                    "output_dir": str(output_dir),
                    "command": selected_attempt.get("command"),
                    "stdout_tail": str(selected_attempt.get("stdout_tail") or ""),
                    "stderr_tail": str(selected_attempt.get("stderr_tail") or ""),
                    "attempts": attempts,
                    "selected_attempt": selected_attempt.get("label"),
                },
            )
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=candidates,
            message="runtime_success",
            diagnostics={
                **readiness,
                "repo_path": self.repo_path,
                "python_bin": self.python_bin,
                "summary": summary,
                "summary_path": str(summary_path),
                "output_dir": str(output_dir),
                "command": selected_attempt.get("command"),
                "stdout_tail": str(selected_attempt.get("stdout_tail") or ""),
                "stderr_tail": str(selected_attempt.get("stderr_tail") or ""),
                "attempts": attempts,
                "selected_attempt": selected_attempt.get("label"),
                "candidate_count": len(candidates),
                "selected_backend": self.name,
            },
        )

    def _readiness(self) -> Dict[str, Any]:
        repo_path = _resolve_repo_path(self.repo_path)
        repo_exists = bool(repo_path and repo_path.exists())
        checkpoint_dirs = _find_contact_graspnet_checkpoint_dirs(repo_path)
        preferred_checkpoint_dir = (
            checkpoint_dirs.get("scene_test_2048_bs3_hor_sigma_001")
            or checkpoint_dirs.get("scene_test_2048_bs3_hor_sigma_0025")
            or checkpoint_dirs.get("scene_2048_bs3_rad2_32")
            or next(iter(checkpoint_dirs.values()), "")
        )
        checkpoint_files = _find_matching_files(repo_path, ["checkpoints/**/*.index", "checkpoints/**/*.meta", "checkpoints/**/*.data-*"])
        import_status = _python_import_status(["tensorflow"], python_bin=self.python_bin)
        tensorflow_available = bool(dict(import_status.get("module_status") or {}).get("tensorflow", False))
        blockers: List[str] = []
        if not repo_exists:
            blockers.append("repo_missing")
        if not checkpoint_dirs:
            blockers.append("checkpoints_missing")
        if not import_status.get("python_bin_exists", False):
            blockers.append("python_bin_missing")
        if not tensorflow_available:
            blockers.append("missing_dependency_tensorflow")
        available = not blockers
        if blockers:
            message = blockers[0]
        else:
            message = "ready_for_external_run"
        return {
            "available": available,
            "message": message,
            "blockers": blockers,
            "python_bin": self.python_bin,
            "python_import_status": import_status,
            "repo_exists": repo_exists,
            "resolved_repo_path": None if repo_path is None else str(repo_path),
            "checkpoint_dirs": checkpoint_dirs,
            "preferred_checkpoint_dir": preferred_checkpoint_dir,
            "checkpoint_files": checkpoint_files[:8],
            "tensorflow_available": tensorflow_available,
        }

    def _export_runtime_input(
        self,
        *,
        export_dir: Path,
        observation: PerceptionObservation,
        grounding: GroundingResult | None,
    ) -> Dict[str, Any]:
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            return {"ok": False, "message": f"missing_runtime_dependency:{exc!r}"}
        rgb = None if observation.rgb is None else np.asarray(observation.rgb)
        depth_mm = None if observation.depth is None else np.asarray(observation.depth, dtype=np.float64)
        if rgb is None or depth_mm is None:
            return {"ok": False, "message": "missing_snapshot_rgb_or_depth"}
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        if rgb.ndim == 3 and rgb.shape[2] > 3:
            rgb = rgb[:, :, :3]
        camera_params = dict(observation.metadata or {}).get("camera_params") or {}
        if "intrinsic_cv" not in camera_params:
            return {"ok": False, "message": "missing_camera_intrinsics"}
        intrinsic = np.asarray(camera_params["intrinsic_cv"], dtype=np.float64).reshape(3, 3)
        segmap, segmap_source = self._resolve_grounding_segmap(shape=depth_mm.shape, grounding=grounding)
        npz_path = export_dir / "robotwin_contact_graspnet_input.npz"
        export_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            npz_path,
            depth=np.asarray(depth_mm, dtype=np.float64) / 1000.0,
            K=intrinsic,
            rgb=rgb,
            segmap=segmap,
        )
        preview_path = export_dir / "preview.png"
        preview = rgb.copy()
        if np.count_nonzero(segmap):
            mask = segmap.astype(bool)
            preview[mask] = (0.6 * preview[mask] + 0.4 * np.array([66, 133, 244], dtype=np.float64)).astype(np.uint8)
        if grounding is not None and grounding.box_xyxy is not None:
            x1, y1, x2, y2 = [int(round(v)) for v in list(grounding.box_xyxy)[:4]]
            cv2.rectangle(preview, (x1, y1), (x2, y2), (66, 133, 244), 3)
        cv2.imwrite(str(preview_path), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
        metadata_path = export_dir / "metadata.json"
        metadata_path.write_text(
            json.dumps(
                {
                    "segmap_source": segmap_source,
                    "camera_params_available": bool(camera_params),
                    "segmap_foreground_pixels": int(np.count_nonzero(segmap)),
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        return {
            "ok": True,
            "npz_path": str(npz_path),
            "preview_path": str(preview_path),
            "metadata_path": str(metadata_path),
            "segmap_source": segmap_source,
            "segmap_foreground_pixels": int(np.count_nonzero(segmap)),
        }

    def _build_command(
        self,
        *,
        repo_path: str,
        npz_path: str,
        ckpt_dir: str,
        out_dir: str,
        local_regions: bool = True,
        filter_grasps: bool = True,
    ) -> List[str]:
        runner_path = _repo_root() / "script_runtime" / "runners" / "run_contact_graspnet_headless.py"
        command = [
            self.python_bin,
            str(runner_path),
            "--repo-path",
            str(Path(repo_path).expanduser().resolve()),
            "--npz-path",
            str(Path(npz_path).expanduser().resolve()),
            "--ckpt-dir",
            str(Path(ckpt_dir).expanduser().resolve()),
            "--out-dir",
            str(Path(out_dir).expanduser().resolve()),
        ]
        if local_regions:
            command.append("--local-regions")
        if filter_grasps:
            command.append("--filter-grasps")
        return command

    @staticmethod
    def _build_bbox_segmap(*, shape: tuple[int, ...], box_xyxy: Any):
        import numpy as np

        height, width = int(shape[0]), int(shape[1])
        segmap = np.zeros((height, width), dtype=np.uint8)
        if box_xyxy is None or len(list(box_xyxy)) < 4:
            return segmap
        x1, y1, x2, y2 = [int(round(float(v))) for v in list(box_xyxy)[:4]]
        x1 = max(min(x1, width - 1), 0)
        y1 = max(min(y1, height - 1), 0)
        x2 = max(min(x2, width), x1 + 1)
        y2 = max(min(y2, height), y1 + 1)
        segmap[y1:y2, x1:x2] = 1
        return segmap

    def _resolve_grounding_segmap(self, *, shape: tuple[int, ...], grounding: GroundingResult | None):
        import numpy as np

        if grounding is not None:
            mask = getattr(grounding, "mask", None)
            if mask is not None:
                mask_array = np.asarray(mask, dtype=bool)
                if mask_array.shape == tuple(shape):
                    return mask_array.astype(np.uint8), str(dict(getattr(grounding, "metadata", {}) or {}).get("mask_source", "grounding_mask"))
            box_xyxy = getattr(grounding, "box_xyxy", None)
        else:
            box_xyxy = None
        return self._build_bbox_segmap(shape=shape, box_xyxy=box_xyxy), "grounding_bbox_rect"

    @staticmethod
    def _evaluate_pose_pair(sdk: Any | None, *, pregrasp_pose: List[float], pose: List[float]) -> Dict[str, Any]:
        planner_status = "Unknown"
        planner_waypoint_count = None
        planner_debug: Dict[str, Any] = {}
        if sdk is None or not hasattr(sdk, "evaluate_pose_candidates"):
            return {
                "planner_status": planner_status,
                "planner_waypoint_count": planner_waypoint_count,
                "planner_debug": planner_debug,
            }
        try:
            pre_eval = list(sdk.evaluate_pose_candidates([pregrasp_pose], kind="pregrasp"))
            grasp_eval = list(sdk.evaluate_pose_candidates([pose], kind="grasp"))
        except Exception as exc:
            return {
                "planner_status": "Failure",
                "planner_waypoint_count": None,
                "planner_debug": {"exception": repr(exc)},
            }
        pre_row = {} if not pre_eval else dict(pre_eval[0] or {})
        grasp_row = {} if not grasp_eval else dict(grasp_eval[0] or {})
        planner_status = "Success" if pre_row.get("status") == "Success" and grasp_row.get("status") == "Success" else "Failure"
        counts = [row.get("waypoint_count") for row in (pre_row, grasp_row) if row.get("waypoint_count") is not None]
        if counts:
            planner_waypoint_count = int(sum(int(v) for v in counts))
        planner_debug = {"pregrasp": pre_row, "grasp": grasp_row}
        return {
            "planner_status": planner_status,
            "planner_waypoint_count": planner_waypoint_count,
            "planner_debug": planner_debug,
        }

    @staticmethod
    def _sdk_object_model_name(sdk: Any | None) -> str:
        if sdk is None:
            return ""
        for attr_name in ("_object_model_name", "object_model_name"):
            value = getattr(sdk, attr_name, None)
            if value is None:
                continue
            try:
                resolved = value() if callable(value) else value
            except Exception:
                continue
            if resolved:
                return str(resolved)
        return ""

    @staticmethod
    def _sdk_object_model_id(sdk: Any | None) -> Any:
        if sdk is None:
            return None
        value = getattr(sdk, "_object_model_id", None)
        if value is None:
            return None
        try:
            return value() if callable(value) else value
        except Exception:
            return None

    @staticmethod
    def _default_affordance_type(sdk: Any | None, model_name: str) -> str:
        if sdk is not None:
            value = getattr(sdk, "_default_affordance_type_for_object", None)
            if value is not None:
                try:
                    resolved = value() if callable(value) else value
                    if resolved:
                        return str(resolved)
                except Exception:
                    pass
        if model_name in {"021_cup", "002_bowl"}:
            return "rim_grasp"
        return "body_support"

    @staticmethod
    def _infer_contact_affordance_type(
        *,
        contact_point: Sequence[float] | None,
        object_pose: List[float] | None,
        model_name: str,
        default_affordance_type: str,
    ) -> str:
        import numpy as np

        if contact_point is None or len(list(contact_point)) < 3:
            return default_affordance_type
        if object_pose is None or len(list(object_pose)) < 7:
            return default_affordance_type
        if model_name not in {"021_cup", "002_bowl"}:
            return default_affordance_type
        object_xyz = np.asarray(list(object_pose[:3]), dtype=np.float64)
        object_rot = _quat_wxyz_to_matrix(list(object_pose[3:7]))
        local_point = object_rot.T @ (np.asarray(list(contact_point[:3]), dtype=np.float64) - object_xyz)
        return "rim_grasp" if float(local_point[2]) >= 0.02 else "body_support"

    def _apply_contact_geometry_semantics(
        self,
        *,
        candidate: Dict[str, Any],
        sdk: Any | None,
        object_pose: List[float] | None,
        semantic_source: str,
        semantic_priority: float,
    ) -> Dict[str, Any]:
        item = dict(candidate)
        model_name = self._sdk_object_model_name(sdk)
        model_id = self._sdk_object_model_id(sdk)
        default_affordance_type = self._default_affordance_type(sdk, model_name)
        affordance_type = self._infer_contact_affordance_type(
            contact_point=item.get("contact_point"),
            object_pose=object_pose,
            model_name=model_name,
            default_affordance_type=default_affordance_type,
        )
        if model_name:
            item["object_model_name"] = item.get("object_model_name") or model_name
        if model_id is not None:
            item["object_model_id"] = item.get("object_model_id") or model_id
        item["affordance_type"] = item.get("affordance_type") or affordance_type
        item["functional_role"] = item.get("functional_role") or "carry"
        item["semantic_source"] = item.get("semantic_source") or semantic_source
        existing_priority = float(item.get("semantic_priority") or 0.0)
        item["semantic_priority"] = max(existing_priority, float(semantic_priority))
        return item

    @staticmethod
    def _inherit_template_task_semantics(
        *,
        candidate: Dict[str, Any],
        template: Dict[str, Any] | None,
    ) -> Dict[str, Any]:
        item = dict(candidate)
        source = dict(template or {})
        if not source:
            return item

        for key in (
            "task_compatibility",
            "functional_role",
            "semantic_reference_contact_id",
            "contact_group_index",
            "object_model_name",
            "object_model_id",
        ):
            if source.get(key) is not None and item.get(key) in (None, "", []):
                item[key] = source.get(key)

        template_contact_id = source.get("contact_point_id")
        if template_contact_id is not None and item.get("template_contact_point_id") is None:
            item["template_contact_point_id"] = template_contact_id

        template_affordance = dict(source.get("affordance") or {})
        if source.get("affordance_type") and not item.get("affordance_type"):
            item["affordance_type"] = source.get("affordance_type")

        if template_affordance:
            affordance = dict(item.get("affordance") or {})
            for key in (
                "task_compatibility",
                "affordance_type",
                "functional_role",
                "notes",
                "preferred_affordances",
                "strict",
                "visual_review_required",
            ):
                if template_affordance.get(key) is not None and affordance.get(key) in (None, "", []):
                    affordance[key] = template_affordance.get(key)
            if source.get("semantic_source") and not affordance.get("template_semantic_source"):
                affordance["template_semantic_source"] = source.get("semantic_source")
            if template_contact_id is not None and affordance.get("template_contact_point_id") is None:
                affordance["template_contact_point_id"] = template_contact_id
            if affordance:
                item["affordance"] = affordance

        template_semantic_source = str(source.get("semantic_source") or "").strip()
        if template_semantic_source:
            item["template_semantic_source"] = template_semantic_source

        item["task_semantics_origin"] = "template_contact"
        return item

    def _load_template_candidates(
        self,
        *,
        sdk: Any | None,
        observation: PerceptionObservation,
        context: Any | None,
        active_arm: str,
        require_planner_success: bool = True,
        record_debug: bool = True,
    ) -> List[Dict[str, Any]]:
        delegate = self.template_delegate if self.template_delegate is not None else sdk
        if record_debug:
            self.last_template_source_debug = {
                "available": bool(delegate is not None and hasattr(delegate, "get_grasp_candidates")),
                "source_kind": "template_delegate" if self.template_delegate is not None else "sdk",
                "active_arm": str(active_arm or ""),
                "delegate_candidate_count": 0,
                "matching_arm_candidate_count": 0,
                "pose_ready_template_count": 0,
                "feasible_template_count": 0,
                "selected_template_count": 0,
                "selected_template_labels": [],
                "bridge_donor_candidate_count": 0,
                "bridge_donor_labels": [],
                "failure_reason": "",
            }
        if delegate is None or not hasattr(delegate, "get_grasp_candidates"):
            if record_debug:
                self.last_template_source_debug["failure_reason"] = "delegate_unavailable"
            return []
        try:
            candidates = delegate.get_grasp_candidates(observation, context=context)
        except TypeError:
            candidates = delegate.get_grasp_candidates()
        except Exception:
            if record_debug:
                self.last_template_source_debug["failure_reason"] = "delegate_exception"
            return []
        if record_debug:
            self.last_template_source_debug["delegate_candidate_count"] = len(list(candidates or []))
        templates: List[Dict[str, Any]] = []
        for candidate in list(candidates or []):
            item = dict(candidate or {})
            if str(item.get("arm") or active_arm) != active_arm:
                continue
            if record_debug:
                self.last_template_source_debug["matching_arm_candidate_count"] += 1
            if item.get("pose") is None or item.get("pregrasp_pose") is None:
                continue
            if record_debug:
                self.last_template_source_debug["pose_ready_template_count"] += 1
            if require_planner_success and str(item.get("planner_status", "Unknown") or "Unknown") != "Success":
                continue
            if record_debug and require_planner_success:
                self.last_template_source_debug["feasible_template_count"] += 1
            templates.append(item)
        templates.sort(key=_sort_key_for_candidate)
        selected = templates[: min(len(templates), 3)]
        if record_debug:
            self.last_template_source_debug["selected_template_count"] = len(selected)
            self.last_template_source_debug["selected_template_labels"] = [
                str(row.get("variant_label") or f"contact_{row.get('contact_point_id')}")
                for row in selected
            ]
            if require_planner_success and not selected and not self.last_template_source_debug["failure_reason"]:
                self.last_template_source_debug["failure_reason"] = "no_feasible_template_candidates"
        return selected

    @staticmethod
    def _template_contact_world_point(sdk: Any | None, contact_id: Any) -> List[float]:
        if sdk is None or contact_id is None:
            return []
        actor_getter = getattr(sdk, "_object_actor", None)
        if actor_getter is None:
            return []
        try:
            actor = actor_getter() if callable(actor_getter) else actor_getter
        except Exception:
            return []
        if actor is None or not hasattr(actor, "get_contact_point"):
            return []
        try:
            contact_pose = actor.get_contact_point(int(contact_id), "list")
        except Exception:
            return []
        if contact_pose is None or len(list(contact_pose)) < 3:
            return []
        return [float(v) for v in list(contact_pose)[:3]]

    def _build_guided_template_candidates(
        self,
        *,
        sdk: Any | None,
        templates: Sequence[Dict[str, Any]],
        raw_contact_evidence: Sequence[Dict[str, Any]],
        active_arm: str,
        object_pose: List[float] | None,
    ) -> List[Dict[str, Any]]:
        import numpy as np

        variants: List[Dict[str, Any]] = []
        for template in list(templates or []):
            template_id = template.get("contact_point_id")
            template_contact_point = self._template_contact_world_point(sdk, template_id)
            if len(template_contact_point) < 3:
                continue
            best_row = None
            best_support = None
            for row in list(raw_contact_evidence or []):
                contact_point = list(row.get("contact_point") or [])
                if len(contact_point) < 3:
                    continue
                distance = float(
                    np.linalg.norm(
                        np.asarray(contact_point[:3], dtype=np.float64)
                        - np.asarray(template_contact_point[:3], dtype=np.float64)
                    )
                )
                support = float(row.get("score", 0.0) or 0.0) - 2.5 * distance
                if best_support is None or support > best_support:
                    best_support = support
                    best_row = dict(row)
                    best_row["guided_contact_distance"] = distance
            if best_row is None:
                continue
            item = {
                "pose": list(template.get("pose") or []),
                "pregrasp_pose": list(template.get("pregrasp_pose") or []),
                "arm": str(template.get("arm") or active_arm),
                "score": max(float(best_support or 0.0), 0.0),
                "variant_label": f"contact_graspnet_guided_c{template_id}",
                "planner_status": str(template.get("planner_status", "Unknown") or "Unknown"),
                "planner_waypoint_count": template.get("planner_waypoint_count"),
                "planner_debug": dict(template.get("planner_debug") or {}),
                "proposal_backend": self.name,
                "proposal_sources": [self.name, f"template_contact_{template_id}", "guided_contact_family"],
                "contact_point": list(best_row.get("contact_point") or []),
                "template_contact_point": template_contact_point,
                "template_contact_point_id": template_id,
                "template_variant_label": template.get("variant_label"),
                "guided_contact_distance": best_row.get("guided_contact_distance"),
                "guided_segment_id": best_row.get("segment_id"),
                "guided_source_index": best_row.get("source_index"),
            }
            item = self._apply_contact_geometry_semantics(
                candidate=item,
                sdk=sdk,
                object_pose=object_pose,
                semantic_source="contact_graspnet_guided_contact_family",
                semantic_priority=1.0 + max(float(best_support or 0.0), 0.0),
            )
            item = self._inherit_template_task_semantics(candidate=item, template=template)
            variants.append(item)
        return variants

    def _build_template_transfer_candidates(
        self,
        *,
        raw_contact_point: Sequence[float] | None,
        segment_id: str,
        source_index: int,
        raw_score: float,
        active_arm: str,
        pregrasp_distance: float,
        sdk: Any | None,
        templates: Sequence[Dict[str, Any]],
        object_pose: List[float] | None,
    ) -> List[Dict[str, Any]]:
        import numpy as np

        if raw_contact_point is None or len(list(raw_contact_point)) < 3:
            return []
        point = np.asarray(list(raw_contact_point[:3]), dtype=np.float64)
        variants: List[Dict[str, Any]] = []
        for template in list(templates or []):
            template_pose = list(template.get("pose") or [])
            if len(template_pose) < 7:
                continue
            quat = [float(v) for v in template_pose[3:7]]
            rotation = _quat_wxyz_to_matrix(quat)
            grasp_xyz = point + rotation @ np.asarray([-0.12, 0.0, 0.0], dtype=np.float64)
            pregrasp_xyz = point + rotation @ np.asarray([-(0.12 + pregrasp_distance), 0.0, 0.0], dtype=np.float64)
            pose = [float(v) for v in grasp_xyz.tolist()] + quat
            pregrasp_pose = [float(v) for v in pregrasp_xyz.tolist()] + quat
            planner_eval = self._evaluate_pose_pair(sdk, pregrasp_pose=pregrasp_pose, pose=pose)
            template_id = template.get("contact_point_id")
            item = {
                "pose": pose,
                "pregrasp_pose": pregrasp_pose,
                "arm": active_arm,
                "score": float(raw_score),
                "variant_label": f"contact_graspnet_template_c{template_id}_{segment_id}_{source_index}",
                "planner_status": planner_eval["planner_status"],
                "planner_waypoint_count": planner_eval["planner_waypoint_count"],
                "planner_debug": planner_eval["planner_debug"],
                "proposal_backend": self.name,
                "proposal_sources": [self.name, f"template_contact_{template_id}"],
                "contact_point": [float(v) for v in point.tolist()],
                "segment_id": str(segment_id),
                "template_contact_point_id": template_id,
                "template_variant_label": template.get("variant_label"),
            }
            item = self._apply_contact_geometry_semantics(
                candidate=item,
                sdk=sdk,
                object_pose=object_pose,
                semantic_source="contact_graspnet_template_transfer",
                semantic_priority=0.5 + float(raw_score),
            )
            item = self._inherit_template_task_semantics(candidate=item, template=template)
            variants.append(item)
        return variants

    def _load_runtime_candidates(
        self,
        *,
        output_dir: Path,
        observation: PerceptionObservation,
        context: Any | None,
        object_pose: List[float] | None,
    ) -> List[Dict[str, Any]]:
        import numpy as np

        camera_params = dict(observation.metadata or {}).get("camera_params") or {}
        cam_to_world = _resolve_camera_to_world(camera_params)
        sdk = None if context is None else getattr(context, "adapters", {}).get("sdk")
        blackboard = None if context is None else getattr(context, "blackboard", None)
        active_arm = "right"
        if blackboard is not None and hasattr(blackboard, "get"):
            active_arm = str(blackboard.get("active_arm") or active_arm)
        if sdk is not None and hasattr(sdk, "active_arm") and getattr(sdk, "active_arm", None):
            active_arm = str(getattr(sdk, "active_arm"))
        pregrasp_distance = 0.10
        if blackboard is not None and hasattr(blackboard, "get"):
            pregrasp_distance = float(blackboard.get("pregrasp_distance", pregrasp_distance) or pregrasp_distance)
        if sdk is not None and hasattr(sdk, "pregrasp_distance"):
            pregrasp_distance = float(getattr(sdk, "pregrasp_distance", pregrasp_distance))
        template_candidates = self._load_template_candidates(
            sdk=sdk,
            observation=observation,
            context=context,
            active_arm=active_arm,
        )
        bridge_template_candidates = list(template_candidates)
        if not bridge_template_candidates:
            bridge_template_candidates = self._load_template_candidates(
                sdk=sdk,
                observation=observation,
                context=context,
                active_arm=active_arm,
                require_planner_success=False,
                record_debug=False,
            )
            self.last_template_source_debug["bridge_donor_candidate_count"] = len(bridge_template_candidates)
            self.last_template_source_debug["bridge_donor_labels"] = [
                str(row.get("variant_label") or f"contact_{row.get('contact_point_id')}")
                for row in list(bridge_template_candidates or [])
            ]
        candidates: List[Dict[str, Any]] = []
        raw_contact_evidence: List[Dict[str, Any]] = []
        for segment_path in sorted(output_dir.glob("segment_*_grasps.npz")):
            segment_id = segment_path.stem.replace("segment_", "").replace("_grasps", "")
            payload = np.load(segment_path)
            grasp_mats = np.asarray(payload["pred_grasps_cam"], dtype=np.float64)
            scores = np.asarray(payload["scores"], dtype=np.float64).reshape(-1)
            contact_pts = np.asarray(payload["contact_pts"], dtype=np.float64).reshape(-1, 3)
            for index in range(min(grasp_mats.shape[0], self.max_candidates)):
                world_matrix = cam_to_world @ grasp_mats[index]
                pose = [float(v) for v in world_matrix[:3, 3].tolist()] + _rotation_matrix_to_quat_wxyz(world_matrix[:3, :3])
                approach_axis = world_matrix[:3, 2]
                norm = float(np.linalg.norm(approach_axis))
                if norm < 1e-6:
                    approach_axis = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
                    norm = 1.0
                approach_axis = approach_axis / norm
                pregrasp_pose = list(pose)
                pregrasp_pose[0] -= float(approach_axis[0]) * pregrasp_distance
                pregrasp_pose[1] -= float(approach_axis[1]) * pregrasp_distance
                pregrasp_pose[2] -= float(approach_axis[2]) * pregrasp_distance
                planner_eval = self._evaluate_pose_pair(sdk, pregrasp_pose=pregrasp_pose, pose=pose)
                raw_score = float(scores[index]) if index < scores.shape[0] else 0.0
                contact_point: List[float] = []
                if index < contact_pts.shape[0]:
                    point_cam = np.append(contact_pts[index], 1.0)
                    contact_world = (cam_to_world @ point_cam.reshape(4, 1)).reshape(-1)[:3]
                    contact_point = [float(v) for v in contact_world.tolist()]
                item = {
                    "pose": pose,
                    "pregrasp_pose": pregrasp_pose,
                    "arm": active_arm,
                    "score": raw_score,
                    "variant_label": f"contact_graspnet_seg{segment_id}_{index}",
                    "planner_status": planner_eval["planner_status"],
                    "planner_waypoint_count": planner_eval["planner_waypoint_count"],
                    "planner_debug": planner_eval["planner_debug"],
                    "proposal_backend": self.name,
                    "proposal_sources": [self.name],
                    "contact_point": contact_point,
                    "segment_id": str(segment_id),
                }
                item = self._apply_contact_geometry_semantics(
                    candidate=item,
                    sdk=sdk,
                    object_pose=object_pose,
                    semantic_source="contact_graspnet_geometry",
                    semantic_priority=raw_score,
                )
                candidates.append(item)
                raw_contact_evidence.append(
                    {
                        "contact_point": contact_point,
                        "score": raw_score,
                        "segment_id": str(segment_id),
                        "source_index": index,
                    }
                )
                candidates.extend(
                    self._build_template_transfer_candidates(
                        raw_contact_point=contact_point,
                        segment_id=str(segment_id),
                        source_index=index,
                        raw_score=raw_score,
                        active_arm=active_arm,
                        pregrasp_distance=pregrasp_distance,
                        sdk=sdk,
                        templates=template_candidates,
                        object_pose=object_pose,
                    )
                )
        candidates.extend(
            self._build_guided_template_candidates(
                sdk=sdk,
                templates=template_candidates,
                raw_contact_evidence=raw_contact_evidence,
                active_arm=active_arm,
                object_pose=object_pose,
            )
        )
        candidates.extend(
            self._build_guided_availability_bridge_candidates(
                sdk=sdk,
                templates=bridge_template_candidates,
                raw_contact_evidence=raw_contact_evidence,
                active_arm=active_arm,
                object_pose=object_pose,
                existing_candidates=candidates,
                pregrasp_distance=pregrasp_distance,
            )
        )
        return self._select_runtime_candidates(candidates)

    def _select_runtime_candidates(self, candidates: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
        ranked = sorted([dict(row or {}) for row in list(candidates or [])], key=_sort_key_for_candidate)
        if len(ranked) <= self.max_candidates:
            return ranked

        guided = [row for row in ranked if _contact_graspnet_variant_kind(row) == "guided"]
        template = [row for row in ranked if _contact_graspnet_variant_kind(row) == "template"]
        raw = [row for row in ranked if _contact_graspnet_variant_kind(row) == "raw"]
        other = [row for row in ranked if _contact_graspnet_variant_kind(row) == "other"]

        selected: List[Dict[str, Any]] = []
        selected_ids = set()

        def take(rows: Sequence[Dict[str, Any]], limit: int) -> None:
            if limit <= 0:
                return
            for row in list(rows or []):
                key = id(row)
                if key in selected_ids:
                    continue
                selected.append(dict(row))
                selected_ids.add(key)
                if len([_row for _row in selected if _contact_graspnet_variant_kind(_row) == _contact_graspnet_variant_kind(row)]) >= limit:
                    break

        guided_budget = min(len(guided), max(3, self.max_candidates // 4))
        template_budget = min(len(template), max(3, self.max_candidates // 4))
        raw_budget = min(len(raw), max(self.max_candidates - guided_budget - template_budget, 0))

        take(guided, guided_budget)
        take(template, template_budget)
        take(raw, raw_budget)

        for row in guided + template + raw + other:
            if len(selected) >= self.max_candidates:
                break
            key = id(row)
            if key in selected_ids:
                continue
            selected.append(dict(row))
            selected_ids.add(key)

        selected.sort(key=_sort_key_for_candidate)
        return selected[: self.max_candidates]

    def _build_guided_availability_bridge_candidates(
        self,
        *,
        sdk: Any | None,
        templates: Sequence[Dict[str, Any]],
        raw_contact_evidence: Sequence[Dict[str, Any]],
        active_arm: str,
        object_pose: List[float] | None,
        existing_candidates: Sequence[Dict[str, Any]],
        pregrasp_distance: float,
    ) -> List[Dict[str, Any]]:
        import numpy as np

        if not templates or not raw_contact_evidence:
            return []
        has_guided_success = any(
            str(row.get("variant_label") or "").startswith("contact_graspnet_guided_")
            and str(row.get("planner_status", "Unknown") or "Unknown") == "Success"
            for row in list(existing_candidates or [])
        )
        if has_guided_success:
            return []

        object_model_name = self._sdk_object_model_name(sdk)
        if object_model_name not in {"021_cup", "002_bowl"}:
            return []
        if self._default_affordance_type(sdk, object_model_name) != "rim_grasp":
            return []

        variants: List[Dict[str, Any]] = []
        for template in list(templates or []):
            template_pose = list(template.get("pose") or [])
            if len(template_pose) < 7:
                continue
            template_id = template.get("contact_point_id")
            template_contact_point = self._template_contact_world_point(sdk, template_id)
            quat = [float(v) for v in template_pose[3:7]]
            rotation = _quat_wxyz_to_matrix(quat)
            best_variant = None
            best_support = None
            for evidence in list(raw_contact_evidence or []):
                contact_point = list(evidence.get("contact_point") or [])
                if len(contact_point) < 3:
                    continue
                point = np.asarray(contact_point[:3], dtype=np.float64)
                support = float(evidence.get("score", 0.0) or 0.0)
                if len(template_contact_point) >= 3:
                    distance = float(
                        np.linalg.norm(
                            point - np.asarray(template_contact_point[:3], dtype=np.float64)
                        )
                    )
                    support -= 1.75 * distance
                pose_xyz = point + rotation @ np.asarray([-0.10, 0.0, 0.0], dtype=np.float64)
                pregrasp_xyz = point + rotation @ np.asarray([-(0.10 + pregrasp_distance), 0.0, 0.0], dtype=np.float64)
                pose = [float(v) for v in pose_xyz.tolist()] + quat
                pregrasp_pose = [float(v) for v in pregrasp_xyz.tolist()] + quat
                planner_eval = self._evaluate_pose_pair(sdk, pregrasp_pose=pregrasp_pose, pose=pose)
                candidate = {
                    "pose": pose,
                    "pregrasp_pose": pregrasp_pose,
                    "arm": str(template.get("arm") or active_arm),
                    "score": max(support, 0.0),
                    "variant_label": f"contact_graspnet_guided_c{template_id}_bridge",
                    "planner_status": planner_eval["planner_status"],
                    "planner_waypoint_count": planner_eval["planner_waypoint_count"],
                    "planner_debug": planner_eval["planner_debug"],
                    "proposal_backend": self.name,
                    "proposal_sources": [
                        self.name,
                        f"template_contact_{template_id}",
                        "guided_contact_family",
                        "guided_availability_bridge",
                    ],
                    "contact_point": [float(v) for v in point.tolist()],
                    "template_contact_point": template_contact_point,
                    "template_contact_point_id": template_id,
                    "template_variant_label": template.get("variant_label"),
                    "guided_segment_id": evidence.get("segment_id"),
                    "guided_source_index": evidence.get("source_index"),
                    "guided_bridge_reason": "guided_family_unavailable",
                }
                candidate = self._apply_contact_geometry_semantics(
                    candidate=candidate,
                    sdk=sdk,
                    object_pose=object_pose,
                    semantic_source="contact_graspnet_guided_availability_bridge",
                    semantic_priority=1.25 + max(support, 0.0),
                )
                candidate = self._inherit_template_task_semantics(candidate=candidate, template=template)
                if best_support is None or _sort_key_for_candidate(candidate) < _sort_key_for_candidate(best_variant or {}):
                    best_support = support
                    best_variant = candidate
            if best_variant is not None:
                variants.append(best_variant)
        return variants


class GraspNetBaselineBackend(GraspProposalBackend):
    name = "graspnet_baseline"

    def __init__(self, repo_path: str | None = None):
        self.repo_path = str(repo_path or "")

    def is_available(self) -> bool:
        readiness = self._readiness()
        return bool(readiness.get("available", False))

    def propose_grasps(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> BackendResult:
        readiness = self._readiness()
        return BackendResult(
            backend_name=self.name,
            available=bool(readiness.get("available", False)),
            ok=False,
            message=str(readiness.get("message", "integration_not_implemented")),
            diagnostics={**readiness, "repo_path": self.repo_path},
        )

    def _readiness(self) -> Dict[str, Any]:
        repo_path = _resolve_repo_path(self.repo_path)
        repo_exists = bool(repo_path and repo_path.exists())
        torch_available = _import_available("torch")
        return {
            "available": False,
            "message": "repo_missing" if not repo_exists else "integration_not_implemented",
            "repo_exists": repo_exists,
            "resolved_repo_path": None if repo_path is None else str(repo_path),
            "torch_available": torch_available,
        }


class GraspGenBackend(GraspProposalBackend):
    name = "graspgen"

    def __init__(self, repo_path: str | None = None):
        self.repo_path = str(repo_path or "")

    def is_available(self) -> bool:
        readiness = self._readiness()
        return bool(readiness.get("available", False))

    def propose_grasps(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> BackendResult:
        readiness = self._readiness()
        return BackendResult(
            backend_name=self.name,
            available=bool(readiness.get("available", False)),
            ok=False,
            message=str(readiness.get("message", "integration_not_implemented")),
            diagnostics={**readiness, "repo_path": self.repo_path},
        )

    def _readiness(self) -> Dict[str, Any]:
        repo_path = _resolve_repo_path(self.repo_path)
        repo_exists = bool(repo_path and repo_path.exists())
        torch_available = _import_available("torch")
        return {
            "available": False,
            "message": "repo_missing" if not repo_exists else "integration_not_implemented",
            "repo_exists": repo_exists,
            "resolved_repo_path": None if repo_path is None else str(repo_path),
            "torch_available": torch_available,
        }


class DelegateGraspProposalBackend(GraspProposalBackend):
    def __init__(self, name: str, delegate: Any):
        self.name = str(name)
        self.delegate = delegate

    def is_available(self) -> bool:
        return self.delegate is not None and hasattr(self.delegate, "get_grasp_candidates")

    def propose_grasps(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> BackendResult:
        if not self.is_available():
            return BackendResult(backend_name=self.name, available=False, ok=False, message="delegate_unavailable")
        get_grasp_candidates = self.delegate.get_grasp_candidates
        supports_context = False
        supports_observation = False
        try:
            signature = inspect.signature(get_grasp_candidates)
            supports_context = "context" in signature.parameters or any(
                parameter.kind == inspect.Parameter.VAR_KEYWORD
                for parameter in signature.parameters.values()
            )
            supports_observation = any(
                parameter.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
                for parameter in signature.parameters.values()
            ) or any(
                parameter.kind == inspect.Parameter.VAR_POSITIONAL
                for parameter in signature.parameters.values()
            )
        except Exception:
            supports_context = False
            supports_observation = False
        if supports_context:
            candidates = get_grasp_candidates(observation, context=context)
        elif supports_observation:
            candidates = get_grasp_candidates(observation)
        else:
            candidates = get_grasp_candidates()
        if not candidates:
            return BackendResult(
                backend_name=self.name,
                available=True,
                ok=False,
                message="no_candidates",
            )
        payload = []
        for candidate in list(candidates):
            item = dict(candidate)
            item["proposal_backend"] = self.name
            item.setdefault("proposal_sources", [self.name])
            payload.append(item)
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=payload,
            diagnostics={"candidate_count": len(payload)},
        )


class HeuristicSemanticReranker(TaskAwareGraspReranker):
    name = "heuristic_semantic"

    def rerank(
        self,
        candidates: Sequence[Dict[str, Any]],
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
        object_pose: List[float] | None = None,
    ) -> Sequence[Dict[str, Any]]:
        ranked = [dict(candidate) for candidate in candidates]
        task_name = resolve_runtime_task_name(context=context)
        blackboard = None if context is None else getattr(context, "blackboard", None)
        if task_name:
            ranked = annotate_grasp_candidates(task_name, ranked, blackboard=blackboard)
            return sort_grasp_candidates_by_semantics(ranked)
        ranked.sort(key=_sort_key_for_candidate)
        return ranked


class FMFirstGraspStackAdapter(PerceptionAdapter):
    """Multi-stage adapter that tries several upstream grasp pipelines.

    The adapter compares multiple backends instead of assuming a single monolithic
    provider is correct. It is designed so we can keep adding real backends
    without changing the downstream runtime contract.
    """

    def __init__(
        self,
        *,
        target_grounders: Optional[Sequence[TargetGrounder]] = None,
        pose_estimators: Optional[Sequence[ObjectPoseEstimator]] = None,
        grasp_backends: Optional[Sequence[GraspProposalBackend]] = None,
        reranker: Optional[TaskAwareGraspReranker] = None,
        geometry_delegate: Any | None = None,
        oracle_backend: Any | None = None,
        use_oracle_fallback: bool = True,
    ):
        self.target_grounders = list(target_grounders or [])
        self.pose_estimators = list(pose_estimators or [])
        self.grasp_backends = list(grasp_backends or [])
        self.reranker = reranker or HeuristicSemanticReranker()
        self.geometry_delegate = geometry_delegate
        self.oracle_backend = oracle_backend
        self.use_oracle_fallback = bool(use_oracle_fallback)
        self.last_target_diagnostics: List[Dict[str, Any]] = []
        self.last_pose_diagnostics: List[Dict[str, Any]] = []
        self.last_grasp_diagnostics: List[Dict[str, Any]] = []
        self.last_target_source = "none"
        self.last_pose_source = "none"
        self.last_grasp_source = "none"
        self.last_grounding: Optional[GroundingResult] = None
        self.last_target_stage_summary: Dict[str, Any] = {}
        self.last_pose_stage_summary: Dict[str, Any] = {}
        self.last_grasp_stage_summary: Dict[str, Any] = {}

    def get_object_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        grounding = self._run_grounding(observation, context=context)
        self.last_grounding = grounding
        diagnostics: List[Dict[str, Any]] = []
        for estimator in self.pose_estimators:
            result = estimator.estimate_pose(observation, context=context, grounding=grounding)
            diagnostics.append(result.as_dict())
            if result.ok and result.payload is not None:
                self.last_pose_diagnostics = diagnostics
                self.last_pose_source = estimator.name
                self.last_pose_stage_summary = {
                    "selected_backend": estimator.name,
                    "selected_backend_kind": "fm_backend" if not isinstance(estimator, DelegatePoseEstimator) else "fallback_delegate",
                    "fallback_reason": "",
                    "diagnostics_count": len(diagnostics),
                }
                return list(result.payload)
        if self.use_oracle_fallback and self.oracle_backend is not None and hasattr(self.oracle_backend, "get_object_pose"):
            pose = self.oracle_backend.get_object_pose()
            diagnostics.append(
                {
                    "backend_name": "oracle_backend_fallback",
                    "available": True,
                    "ok": pose is not None,
                    "message": "" if pose is not None else "pose_unavailable",
                    "diagnostics": {},
                }
            )
            self.last_pose_diagnostics = diagnostics
            if pose is not None:
                self.last_pose_source = "oracle_backend_fallback"
                self.last_pose_stage_summary = {
                    "selected_backend": "oracle_backend_fallback",
                    "selected_backend_kind": "oracle_fallback",
                    "fallback_reason": self._first_backend_failure(diagnostics, preferred_backend="foundationpose"),
                    "diagnostics_count": len(diagnostics),
                }
                return list(pose)
        self.last_pose_diagnostics = diagnostics
        self.last_pose_source = "none"
        self.last_pose_stage_summary = {
            "selected_backend": "none",
            "selected_backend_kind": "none",
            "fallback_reason": self._first_backend_failure(diagnostics, preferred_backend="foundationpose"),
            "diagnostics_count": len(diagnostics),
        }
        return None

    def get_place_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if self.geometry_delegate is not None and hasattr(self.geometry_delegate, "get_place_pose"):
            try:
                return self.geometry_delegate.get_place_pose(observation, context=context)
            except TypeError:
                return self.geometry_delegate.get_place_pose()
        if self.oracle_backend is not None and hasattr(self.oracle_backend, "get_place_pose"):
            return self.oracle_backend.get_place_pose()
        return None

    def get_place_release_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if self.geometry_delegate is not None and hasattr(self.geometry_delegate, "get_place_release_pose"):
            try:
                return self.geometry_delegate.get_place_release_pose(observation, context=context)
            except TypeError:
                return self.geometry_delegate.get_place_release_pose()
        if self.oracle_backend is not None and hasattr(self.oracle_backend, "get_place_release_pose"):
            return self.oracle_backend.get_place_release_pose()
        return None

    def get_retreat_pose(self, observation: PerceptionObservation, context: Any | None = None) -> Optional[List[float]]:
        if self.geometry_delegate is not None and hasattr(self.geometry_delegate, "get_retreat_pose"):
            try:
                return self.geometry_delegate.get_retreat_pose(observation, context=context)
            except TypeError:
                return self.geometry_delegate.get_retreat_pose()
        if self.oracle_backend is not None and hasattr(self.oracle_backend, "get_retreat_pose"):
            return self.oracle_backend.get_retreat_pose()
        return None

    def get_grasp_candidates(
        self,
        observation: PerceptionObservation,
        context: Any | None = None,
    ) -> Optional[List[Dict[str, Any]]]:
        grounding = self.last_grounding or self._run_grounding(observation, context=context)
        object_pose = self._resolve_object_pose(observation, context=context, grounding=grounding)
        diagnostics: List[Dict[str, Any]] = []
        candidate_groups: List[List[Dict[str, Any]]] = []
        for backend in self.grasp_backends:
            result = backend.propose_grasps(
                observation,
                context=context,
                grounding=grounding,
                object_pose=object_pose,
            )
            diagnostics.append(result.as_dict())
            if result.ok and result.payload:
                candidate_groups.append([dict(candidate) for candidate in list(result.payload)])
        merged = _merge_candidate_lists(candidate_groups)
        if merged:
            reranked = list(
                self.reranker.rerank(
                    merged,
                    observation,
                    context=context,
                    grounding=grounding,
                    object_pose=object_pose,
                )
            )
            self.last_grasp_diagnostics = diagnostics
            selected_backend = str(reranked[0].get("proposal_backend") or reranked[0].get("proposal_sources", ["multi_backend_merge"])[0])
            selected_backend_kind = "fm_backend" if selected_backend not in {"oracle_feasibility", "depth_synthesized"} else "fallback_delegate"
            contact_runtime_ok = any(
                str(row.get("backend_name") or "") == "contact_graspnet" and bool(row.get("ok", False))
                for row in diagnostics
            )
            if selected_backend_kind == "fm_backend":
                fallback_reason = ""
            elif contact_runtime_ok:
                fallback_reason = "fallback_selected_over_fm_backend"
            else:
                fallback_reason = self._first_backend_failure(diagnostics, preferred_backend="contact_graspnet")
            self.last_grasp_source = selected_backend or "multi_backend_merge"
            template_debug = {}
            contact_runtime = next(
                (backend for backend in self.grasp_backends if getattr(backend, "name", "") == "contact_graspnet"),
                None,
            )
            if contact_runtime is not None:
                template_debug = dict(getattr(contact_runtime, "last_template_source_debug", {}) or {})
            guided_summary = _guided_stage_summary(reranked=reranked, diagnostics=diagnostics, template_debug=template_debug)
            self.last_grasp_stage_summary = {
                "selected_backend": selected_backend or "multi_backend_merge",
                "selected_backend_kind": selected_backend_kind,
                "fallback_reason": fallback_reason,
                "merged_backend_count": len(candidate_groups),
                "candidate_count": len(reranked),
                **guided_summary,
            }
            return reranked
        self.last_grasp_diagnostics = diagnostics
        self.last_grasp_source = "none"
        template_debug = {}
        contact_runtime = next(
            (backend for backend in self.grasp_backends if getattr(backend, "name", "") == "contact_graspnet"),
            None,
        )
        if contact_runtime is not None:
            template_debug = dict(getattr(contact_runtime, "last_template_source_debug", {}) or {})
        guided_summary = _guided_stage_summary(reranked=[], diagnostics=diagnostics, template_debug=template_debug)
        self.last_grasp_stage_summary = {
            "selected_backend": "none",
            "selected_backend_kind": "none",
            "fallback_reason": self._first_backend_failure(diagnostics, preferred_backend="contact_graspnet"),
            "merged_backend_count": 0,
            "candidate_count": 0,
            **guided_summary,
        }
        return None

    def _run_grounding(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
    ) -> GroundingResult | None:
        diagnostics: List[Dict[str, Any]] = []
        for grounder in self.target_grounders:
            result = grounder.ground(observation, context=context)
            diagnostics.append(result.as_dict())
            if result.ok and result.payload is not None:
                self.last_target_diagnostics = diagnostics
                self.last_target_source = grounder.name
                self.last_target_stage_summary = {
                    "selected_backend": grounder.name,
                    "selected_backend_kind": "fm_backend" if grounder.name != "task_goal_prompt" else "fallback_delegate",
                    "fallback_reason": self._first_backend_failure(diagnostics, preferred_backend="grounded_sam2"),
                    "diagnostics_count": len(diagnostics),
                }
                return result.payload
        self.last_target_diagnostics = diagnostics
        self.last_target_source = "none"
        self.last_target_stage_summary = {
            "selected_backend": "none",
            "selected_backend_kind": "none",
            "fallback_reason": self._first_backend_failure(diagnostics, preferred_backend="grounded_sam2"),
            "diagnostics_count": len(diagnostics),
        }
        return None

    @staticmethod
    def _first_backend_failure(diagnostics: Sequence[Dict[str, Any]], preferred_backend: str) -> str:
        for row in list(diagnostics or []):
            if str(row.get("backend_name", "")) == str(preferred_backend) and not bool(row.get("ok", False)):
                message = str(row.get("message", "") or "").strip()
                return message or "backend_not_ready"
        for row in list(diagnostics or []):
            if not bool(row.get("ok", False)):
                message = str(row.get("message", "") or "").strip()
                if message:
                    return message
        return ""

    def _resolve_object_pose(
        self,
        observation: PerceptionObservation,
        *,
        context: Any | None = None,
        grounding: GroundingResult | None = None,
    ) -> List[float] | None:
        if context is not None:
            blackboard = getattr(context, "blackboard", None)
            if blackboard is not None and hasattr(blackboard, "get"):
                pose = blackboard.get("object_pose")
                if pose is not None:
                    return list(pose)
            world_state = getattr(context, "world_state", None)
            scene = None if world_state is None else getattr(world_state, "scene", None)
            pose = None if scene is None else getattr(scene, "object_pose", None)
            if pose is not None:
                return list(pose)
        return self.get_object_pose(observation, context=context)


def build_default_fm_first_grasp_stack(
    *,
    oracle_backend: Any | None = None,
    robotwin_depth_provider: RoboTwinDepthPoseProvider | None = None,
    grounded_sam2_repo: str | None = None,
    grounded_sam2_model_id: str = "IDEA-Research/grounding-dino-tiny",
    grounded_sam2_box_threshold: float = 0.35,
    grounded_sam2_text_threshold: float = 0.25,
    grounded_sam2_max_detections: int = 8,
    grounded_sam2_device: str | None = None,
    foundationpose_repo: str | None = None,
    foundationpose_python_bin: str | None = None,
    foundationpose_timeout_s: int = 180,
    contact_graspnet_repo: str | None = None,
    contact_graspnet_python_bin: str | None = None,
    contact_graspnet_timeout_s: int = 180,
    contact_graspnet_max_candidates: int = 12,
    graspnet_repo: str | None = None,
    graspgen_repo: str | None = None,
    include_grounded_sam2: bool = True,
    include_task_goal_grounder: bool = True,
    include_foundationpose: bool = True,
    include_contact_graspnet: bool = True,
    include_graspnet_baseline: bool = True,
    include_graspgen: bool = True,
    include_oracle_pose: bool = True,
    include_oracle_grasp: bool = True,
    include_depth_pose: bool = True,
    include_depth_grasp: bool = True,
) -> FMFirstGraspStackAdapter:
    target_grounders: List[TargetGrounder] = []
    if include_grounded_sam2:
        target_grounders.append(
            GroundedSAM2Grounder(
                repo_path=grounded_sam2_repo,
                model_id=grounded_sam2_model_id,
                box_threshold=grounded_sam2_box_threshold,
                text_threshold=grounded_sam2_text_threshold,
                max_detections=grounded_sam2_max_detections,
                device=grounded_sam2_device,
            )
        )
    if include_task_goal_grounder:
        target_grounders.append(TaskGoalTargetGrounder())

    pose_estimators: List[ObjectPoseEstimator] = []
    if include_foundationpose:
        pose_estimators.append(
            FoundationPoseEstimator(
                repo_path=foundationpose_repo,
                python_bin=foundationpose_python_bin,
                timeout_s=foundationpose_timeout_s,
            )
        )
    if include_depth_pose and robotwin_depth_provider is not None:
        pose_estimators.append(
            DelegatePoseEstimator(
                "robotwin_depth",
                robotwin_depth_provider,
                source_hint="robotwin_depth_provider",
            )
        )
    if include_oracle_pose and oracle_backend is not None:
        pose_estimators.append(
            DelegatePoseEstimator(
                "oracle_pose",
                oracle_backend,
                source_hint="oracle_backend",
            )
        )

    grasp_backends: List[GraspProposalBackend] = []
    if include_contact_graspnet:
        grasp_backends.append(
            ContactGraspNetBackend(
                repo_path=contact_graspnet_repo,
                python_bin=contact_graspnet_python_bin,
                timeout_s=contact_graspnet_timeout_s,
                max_candidates=contact_graspnet_max_candidates,
                template_delegate=robotwin_depth_provider,
            )
        )
    if include_graspnet_baseline:
        grasp_backends.append(GraspNetBaselineBackend(repo_path=graspnet_repo))
    if include_graspgen:
        grasp_backends.append(GraspGenBackend(repo_path=graspgen_repo))
    if include_oracle_grasp and oracle_backend is not None:
        grasp_backends.append(DelegateGraspProposalBackend("oracle_feasibility", oracle_backend))
    if include_depth_grasp and robotwin_depth_provider is not None:
        grasp_backends.append(DelegateGraspProposalBackend("depth_synthesized", robotwin_depth_provider))

    geometry_delegate = robotwin_depth_provider or oracle_backend
    return FMFirstGraspStackAdapter(
        target_grounders=target_grounders,
        pose_estimators=pose_estimators,
        grasp_backends=grasp_backends,
        reranker=HeuristicSemanticReranker(),
        geometry_delegate=geometry_delegate,
        oracle_backend=oracle_backend,
        use_oracle_fallback=True,
    )
