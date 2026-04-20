"""Task-level grasp affordance annotation and semantic validation helpers.

This module separates two questions that were previously conflated:

1. Did the gripper physically secure something?
2. Is the selected grasp candidate semantically appropriate for the task?

The first question is handled by adapter-specific contact / lift checks.
The second question is handled here via a lightweight affordance registry and
task-compatibility policy. The design is intentionally simple so current
heuristics, future perception modules, and learned affordance scorers can all
share the same interface.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GraspAffordanceRule:
    label: str
    affordance_type: str
    functional_role: str
    variant_substrings: Tuple[str, ...] = ()
    contact_point_ids: Tuple[int, ...] = ()
    task_names: Tuple[str, ...] = ()
    notes: str = ""
    semantic_priority: float = 0.0
    semantic_source: str = "registry"

    def matches(self, task_name: str, candidate: Dict[str, Any]) -> bool:
        if self.task_names and task_name and task_name not in self.task_names:
            return False
        label = str(candidate.get("variant_label", "") or "")
        contact_id = candidate.get("contact_point_id")
        if self.contact_point_ids and contact_id is not None:
            if int(contact_id) in {int(value) for value in self.contact_point_ids}:
                return True
        if self.variant_substrings and label:
            lowered = label.lower()
            return any(token.lower() in lowered for token in self.variant_substrings)
        return False


@dataclass(frozen=True)
class TaskGraspSemanticPolicy:
    preferred_affordances: Tuple[str, ...] = ()
    incompatible_affordances: Tuple[str, ...] = ()
    strict: bool = False
    visual_review_required: bool = False


DEFAULT_AFFORDANCE_RULES: Tuple[GraspAffordanceRule, ...] = (
    GraspAffordanceRule(
        label="handle_grasp",
        affordance_type="handle",
        functional_role="carry",
        variant_substrings=("handle", "hook", "grip"),
        semantic_priority=1.0,
    ),
    GraspAffordanceRule(
        label="rim_grasp",
        affordance_type="rim_grasp",
        functional_role="support",
        variant_substrings=("rim", "lip", "edge"),
        semantic_priority=0.7,
    ),
    GraspAffordanceRule(
        label="body_grasp",
        affordance_type="body_support",
        functional_role="clamp",
        variant_substrings=("body", "side", "shell"),
        semantic_priority=0.55,
    ),
    GraspAffordanceRule(
        label="surface_grasp",
        affordance_type="surface_support",
        functional_role="pinch",
        variant_substrings=("surface", "face", "pad"),
        semantic_priority=0.4,
    ),
)


TASK_GRASP_POLICIES: Dict[str, TaskGraspSemanticPolicy] = {
    "place_empty_cup": TaskGraspSemanticPolicy(
        preferred_affordances=("handle", "body_support", "rim_grasp"),
        visual_review_required=True,
    ),
    "place_mouse_pad": TaskGraspSemanticPolicy(
        preferred_affordances=("surface_support", "rim_grasp"),
        visual_review_required=True,
    ),
    "place_container_plate": TaskGraspSemanticPolicy(
        preferred_affordances=("handle", "body_support", "rim_grasp"),
        visual_review_required=True,
    ),
}


TASK_COMPATIBILITY_ORDER: Dict[str, int] = {
    "preferred": 0,
    "compatible": 1,
    "unknown": 2,
    "incompatible": 3,
}


def resolve_runtime_task_name(*providers: Any, context: Any | None = None) -> str:
    for provider in providers:
        if provider is None:
            continue
        task_name = getattr(provider, "task_name", "")
        if task_name:
            return str(task_name)
    if context is not None:
        task_goal = dict(getattr(context.world_state.execution, "task_goal", {}) or {})
        for key in ("task_name", "task", "target_task"):
            value = task_goal.get(key)
            if value:
                return str(value)
        if hasattr(context.blackboard, "get"):
            value = context.blackboard.get("task_name") or context.blackboard.get("runtime_task_name")
            if value:
                return str(value)
    return ""


def resolve_grasp_semantic_policy(task_name: str, *, blackboard: Any | None = None) -> Dict[str, Any]:
    base = TASK_GRASP_POLICIES.get(task_name, TaskGraspSemanticPolicy())
    preferred = tuple(str(value) for value in base.preferred_affordances)
    incompatible = tuple(str(value) for value in base.incompatible_affordances)
    strict = bool(base.strict)
    visual_review_required = bool(base.visual_review_required)

    if blackboard is not None:
        required = blackboard.get("required_grasp_affordances")
        incompatible_override = blackboard.get("incompatible_grasp_affordances")
        strict = bool(blackboard.get("semantic_grasp_required", strict))
        visual_review_required = bool(blackboard.get("visual_review_required", visual_review_required))
        if required:
            preferred = tuple(str(value) for value in list(required))
        if incompatible_override:
            incompatible = tuple(str(value) for value in list(incompatible_override))

    return {
        "task_name": task_name,
        "preferred_affordances": preferred,
        "incompatible_affordances": incompatible,
        "strict": strict,
        "visual_review_required": visual_review_required,
    }


def annotate_grasp_candidates(
    task_name: str,
    candidates: Sequence[Dict[str, Any]],
    *,
    blackboard: Any | None = None,
) -> List[Dict[str, Any]]:
    return [annotate_grasp_candidate(task_name, candidate, blackboard=blackboard) for candidate in candidates]


def semantic_sort_key(candidate: Dict[str, Any]) -> tuple[int, int, float, float, int]:
    planner_status = str(candidate.get("planner_status", "Unknown") or "Unknown")
    compatibility = str(candidate.get("task_compatibility", "unknown") or "unknown")
    affordance = dict(candidate.get("affordance") or {})
    semantic_priority = float(
        candidate.get("semantic_priority")
        or affordance.get("semantic_priority")
        or 0.0
    )
    planner_waypoint_count = int(candidate.get("planner_waypoint_count") or 10**9)
    score = float(candidate.get("score", 0.0) or 0.0)
    return (
        0 if planner_status == "Success" else 1,
        TASK_COMPATIBILITY_ORDER.get(compatibility, TASK_COMPATIBILITY_ORDER["unknown"]),
        -semantic_priority,
        -score,
        planner_waypoint_count,
    )


def sort_grasp_candidates_by_semantics(candidates: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return sorted([dict(candidate) for candidate in candidates], key=semantic_sort_key)


def annotate_grasp_candidate(
    task_name: str,
    candidate: Dict[str, Any],
    *,
    blackboard: Any | None = None,
) -> Dict[str, Any]:
    annotated = dict(candidate)
    policy = resolve_grasp_semantic_policy(task_name, blackboard=blackboard)
    rule = _match_affordance_rule(task_name, candidate, blackboard=blackboard)
    existing = dict(annotated.get("affordance") or {})
    affordance_type = str(
        annotated.get("affordance_type")
        or existing.get("affordance_type")
        or (rule.affordance_type if rule is not None else "unknown")
    )
    functional_role = str(
        annotated.get("functional_role")
        or existing.get("functional_role")
        or (rule.functional_role if rule is not None else "generic")
    )
    semantic_source = str(
        annotated.get("semantic_source")
        or existing.get("semantic_source")
        or (rule.semantic_source if rule is not None else "unannotated")
    )
    semantic_priority = float(
        annotated.get("semantic_priority")
        or existing.get("semantic_priority")
        or (rule.semantic_priority if rule is not None else 0.0)
    )
    task_compatibility = str(
        annotated.get("task_compatibility")
        or existing.get("task_compatibility")
        or _task_compatibility_for_affordance(affordance_type, policy)
    )
    notes = str(existing.get("notes") or (rule.notes if rule is not None else ""))

    affordance = {
        "task_name": task_name,
        "affordance_type": affordance_type,
        "functional_role": functional_role,
        "task_compatibility": task_compatibility,
        "semantic_source": semantic_source,
        "semantic_priority": semantic_priority,
        "notes": notes,
        "preferred_affordances": list(policy["preferred_affordances"]),
        "strict": bool(policy["strict"]),
        "visual_review_required": bool(policy["visual_review_required"]),
    }
    annotated["affordance"] = affordance
    annotated["affordance_type"] = affordance_type
    annotated["functional_role"] = functional_role
    annotated["task_compatibility"] = task_compatibility
    annotated["semantic_source"] = semantic_source
    annotated["semantic_priority"] = semantic_priority
    return annotated


def build_grasp_semantic_report(
    task_name: str,
    candidate: Dict[str, Any] | None,
    *,
    grasped: bool,
    object_pose: Optional[List[float]] = None,
    eef_pose: Optional[List[float]] = None,
    blackboard: Any | None = None,
) -> Dict[str, Any]:
    annotated = annotate_grasp_candidate(task_name, candidate or {}, blackboard=blackboard)
    policy = resolve_grasp_semantic_policy(task_name, blackboard=blackboard)
    task_compatibility = str(annotated.get("task_compatibility", "unknown"))

    reasons: List[str] = []
    ok = bool(grasped)
    if not grasped:
        reasons.append("physical_grasp_not_confirmed")
    if task_compatibility == "incompatible":
        ok = False
        reasons.append("candidate_affordance_incompatible")
    elif task_compatibility == "unknown" and policy["strict"]:
        ok = False
        reasons.append("semantic_affordance_unresolved")
    elif task_compatibility == "unknown":
        reasons.append("semantic_affordance_unresolved_soft")

    delta = _object_to_eef_delta(object_pose, eef_pose)
    message = "Grasp semantics accepted"
    if not grasped:
        message = "Grasp not confirmed"
    elif reasons and not ok:
        message = f"Grasp semantics failed: {', '.join(reasons)}"
    elif reasons:
        message = f"Grasp accepted with warnings: {', '.join(reasons)}"

    return {
        "ok": ok,
        "grasped": bool(grasped),
        "message": message,
        "task_name": task_name,
        "candidate_label": annotated.get("variant_label"),
        "contact_point_id": annotated.get("contact_point_id"),
        "object_model_name": annotated.get("object_model_name"),
        "object_model_id": annotated.get("object_model_id"),
        "contact_group_index": annotated.get("contact_group_index"),
        "semantic_reference_contact_id": annotated.get("semantic_reference_contact_id"),
        "selected_contact_in_preferred_family": task_compatibility == "preferred",
        "affordance": annotated.get("affordance", {}),
        "task_compatibility": task_compatibility,
        "semantic_gate": "strict" if policy["strict"] else "soft",
        "visual_review_required": bool(policy["visual_review_required"]),
        "reasons": reasons,
        "object_to_eef_delta": delta,
    }


def _task_compatibility_for_affordance(affordance_type: str, policy: Dict[str, Any]) -> str:
    if not affordance_type or affordance_type == "unknown":
        return "unknown"
    if affordance_type in set(policy["incompatible_affordances"]):
        return "incompatible"
    if affordance_type in set(policy["preferred_affordances"]):
        return "preferred"
    return "compatible"


def _match_affordance_rule(
    task_name: str,
    candidate: Dict[str, Any],
    *,
    blackboard: Any | None = None,
) -> Optional[GraspAffordanceRule]:
    for rule in _iter_override_rules(blackboard):
        if rule.matches(task_name, candidate):
            return rule
    for rule in DEFAULT_AFFORDANCE_RULES:
        if rule.matches(task_name, candidate):
            return rule
    return None


def _iter_override_rules(blackboard: Any | None) -> Iterable[GraspAffordanceRule]:
    if blackboard is None:
        return ()
    raw_rules = blackboard.get("grasp_affordance_overrides") or []
    rules: List[GraspAffordanceRule] = []
    for index, row in enumerate(raw_rules):
        if not isinstance(row, dict):
            continue
        label = str(row.get("label", f"override_{index}"))
        contact_ids = row.get("contact_point_ids")
        if contact_ids is None and row.get("contact_point_id") is not None:
            contact_ids = [row.get("contact_point_id")]
        variant_substrings = row.get("variant_substrings")
        if variant_substrings is None and row.get("variant_contains") is not None:
            variant_substrings = [row.get("variant_contains")]
        task_names = row.get("task_names")
        if task_names is None and row.get("task_name") is not None:
            task_names = [row.get("task_name")]
        rules.append(
            GraspAffordanceRule(
                label=label,
                affordance_type=str(row.get("affordance_type", "unknown")),
                functional_role=str(row.get("functional_role", "generic")),
                variant_substrings=tuple(str(value) for value in list(variant_substrings or [])),
                contact_point_ids=tuple(int(value) for value in list(contact_ids or [])),
                task_names=tuple(str(value) for value in list(task_names or [])),
                notes=str(row.get("notes", "")),
                semantic_priority=float(row.get("semantic_priority", 1.0)),
                semantic_source=str(row.get("semantic_source", "override")),
            )
        )
    return tuple(rules)


def _object_to_eef_delta(
    object_pose: Optional[List[float]],
    eef_pose: Optional[List[float]],
) -> Dict[str, float] | None:
    if object_pose is None or eef_pose is None or len(object_pose) < 3 or len(eef_pose) < 3:
        return None
    dx = float(object_pose[0] - eef_pose[0])
    dy = float(object_pose[1] - eef_pose[1])
    dz = float(object_pose[2] - eef_pose[2])
    return {
        "dx": dx,
        "dy": dy,
        "dz": dz,
        "xyz_norm": float(np.linalg.norm([dx, dy, dz])),
    }
