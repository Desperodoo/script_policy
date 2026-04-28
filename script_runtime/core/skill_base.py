"""Base skill abstractions."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from .failure_codes import FailureCode
from .result_types import RecoveryAction, SkillResult, TraceEvent

REFRESH_REASON_SCRATCH_KEY = "_refresh_reason"


@dataclass
class SkillContext:
    """Runtime context injected into skills."""

    blackboard: Any
    adapters: Dict[str, Any] = field(default_factory=dict)
    task_id: str = ""
    trace_sink: Optional[Any] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def world_state(self):  # pragma: no cover - trivial
        return self.blackboard.world_state

    def emit_trace(self, event: TraceEvent) -> None:
        if self.trace_sink is not None:
            self.trace_sink.record(event)


class Skill:
    """Base skill contract for script runtime."""

    def __init__(
        self,
        name: str,
        timeout_s: float = 5.0,
        failure_code: FailureCode = FailureCode.UNKNOWN,
        safety_limits: Optional[Dict[str, Any]] = None,
    ):
        self.name = name
        self.timeout_s = float(timeout_s)
        self.default_failure_code = failure_code
        self.safety_limits = dict(safety_limits or {})

    def preconditions(self, world_state: Any, context: SkillContext) -> Tuple[bool, str]:
        return True, ""

    def run(self, context: SkillContext) -> SkillResult:  # pragma: no cover - abstract
        raise NotImplementedError

    def recover(self, context: SkillContext) -> Optional[RecoveryAction]:
        return None

    def trace_payload(self, context: SkillContext, result: SkillResult) -> Dict[str, Any]:
        return {
            "message": result.message,
            "payload": result.payload,
        }

    def execute(self, context: SkillContext) -> SkillResult:
        ok, reason = self.preconditions(context.world_state, context)
        started = time.monotonic()
        if not ok:
            result = SkillResult.failure(
                FailureCode.PRECONDITION_FAILED,
                message=reason or f"Preconditions failed for {self.name}",
                recovery_action=self.recover(context),
            )
            self._emit_trace(context, result, time.monotonic() - started)
            return result

        result = self.run(context)
        if result.status is result.status.FAILURE and result.recovery_action is None:
            result.recovery_action = self.recover(context)
        self._emit_trace(context, result, time.monotonic() - started)
        return result

    def _emit_trace(self, context: SkillContext, result: SkillResult, duration_s: float) -> None:
        execution = context.world_state.execution
        retry_count = execution.retry_counts.get(self.name, 0)
        event = TraceEvent(
            task_id=context.task_id or execution.task_id,
            skill_name=self.name,
            node_name=str(context.metadata.get("current_node_name") or ""),
            result=result.status.value,
            failure_code=result.failure_code.value,
            duration_s=duration_s,
            retry_count=retry_count,
            active_source=execution.active_source,
            control_owner=execution.control_owner,
            inputs_summary=self.trace_payload(context, result),
            payload=result.payload,
        )
        context.emit_trace(event)


def set_pending_refresh_reason(blackboard: Any, reason: str) -> None:
    if blackboard is None or not hasattr(blackboard, "set"):
        return
    blackboard.set(REFRESH_REASON_SCRATCH_KEY, str(reason or "unspecified"))


def clear_pending_refresh_reason(blackboard: Any) -> None:
    if blackboard is None or not hasattr(blackboard, "delete"):
        return
    blackboard.delete(REFRESH_REASON_SCRATCH_KEY)


def consume_refresh_reason(blackboard: Any, default: str = "unspecified") -> str:
    if blackboard is None or not hasattr(blackboard, "get"):
        return default
    reason = str(blackboard.get(REFRESH_REASON_SCRATCH_KEY, default) or default)
    clear_pending_refresh_reason(blackboard)
    return reason


def request_world_refresh(context: SkillContext, sdk: Any, reason: str) -> Any:
    if sdk is None or not hasattr(sdk, "refresh_world"):
        return None
    set_pending_refresh_reason(context.blackboard, reason)
    try:
        return sdk.refresh_world(context.blackboard)
    finally:
        clear_pending_refresh_reason(context.blackboard)


def in_support_regrasp_context(context: SkillContext) -> bool:
    blackboard = getattr(context, "blackboard", None)
    if blackboard is None or not hasattr(blackboard, "get"):
        return False
    if bool(blackboard.get("probe_support_regrasp_active", False)):
        return True
    if str(blackboard.get("probe_phase", "") or "").strip().lower() == "support_regrasp":
        return True
    node_name = str(getattr(context, "metadata", {}).get("current_node_name") or "").strip().lower()
    return node_name.startswith("support_") or "support_regrasp" in node_name


def set_support_regrasp_substage(context: SkillContext, substage: str) -> str:
    text = str(substage or "").strip()
    blackboard = getattr(context, "blackboard", None)
    if not text or blackboard is None or not hasattr(blackboard, "set") or not in_support_regrasp_context(context):
        return ""
    blackboard.set("support_regrasp_substage", text)
    return text


def support_regrasp_trace_context(context: SkillContext) -> Dict[str, Any]:
    if not in_support_regrasp_context(context):
        return {}
    blackboard = getattr(context, "blackboard", None)
    if blackboard is None or not hasattr(blackboard, "get"):
        return {}
    values = {
        "support_arm": str(blackboard.get("support_arm", "") or ""),
        "support_target_frame": str(blackboard.get("support_target_frame", "") or ""),
        "support_pregrasp_pose_source": str(blackboard.get("support_pregrasp_pose_source", "") or ""),
        "support_regrasp_substage": str(blackboard.get("support_regrasp_substage", "") or ""),
    }
    return {key: value for key, value in values.items() if value}


def sync_active_arm_to_candidate(context: SkillContext, candidate: Dict[str, Any] | None) -> str:
    item = dict(candidate or {})
    arm = str(item.get("arm") or "").strip().lower()
    if arm not in {"left", "right"} or not in_support_regrasp_context(context):
        return ""
    blackboard = getattr(context, "blackboard", None)
    if blackboard is not None and hasattr(blackboard, "set"):
        blackboard.set("active_arm", arm)
        blackboard.set("support_arm", arm)
    sdk = getattr(context, "adapters", {}).get("sdk")
    if sdk is not None and hasattr(sdk, "active_arm"):
        setattr(sdk, "active_arm", arm)
    return arm


def _candidate_identity(candidate: Dict[str, Any] | None) -> tuple:
    row = dict(candidate or {})
    contact_id = row.get("contact_point_id")
    if contact_id is not None:
        return ("contact", int(contact_id), str(row.get("arm", "") or ""))
    pose = tuple(round(float(v), 5) for v in list(row.get("pose") or [])[:7])
    return ("pose", str(row.get("arm", "") or ""), pose)


def _candidate_identity_text(candidate: Dict[str, Any] | None) -> str:
    row = dict(candidate or {})
    contact_id = row.get("contact_point_id")
    arm = str(row.get("arm", "") or "")
    label = str(row.get("variant_label") or row.get("label") or "")
    if contact_id is not None:
        if label:
            return f"contact:{arm}:{int(contact_id)}:{label}"
        return f"contact:{arm}:{int(contact_id)}"
    pose = [round(float(v), 5) for v in list(row.get("pose") or [])[:3]]
    return f"pose:{arm}:{label}:{pose}"


def _candidate_trace_brief(candidate: Dict[str, Any] | None) -> Dict[str, Any]:
    row = dict(candidate or {})
    label = str(row.get("variant_label") or row.get("label") or "")
    return {
        "variant_label": label,
        "contact_point_id": row.get("contact_point_id"),
        "arm": str(row.get("arm", "") or ""),
        "planner_status": str(row.get("planner_status", "Unknown") or "Unknown"),
        "planner_waypoint_count": row.get("planner_waypoint_count"),
        "task_compatibility": str(row.get("task_compatibility", "unknown") or "unknown"),
        "score": float(row.get("score", 0.0) or 0.0),
        "pose_xyz": [float(v) for v in list(row.get("pose") or [])[:3]],
        "pregrasp_xyz": [float(v) for v in list(row.get("pregrasp_pose") or [])[:3]],
    }


def _current_selection_node(context: SkillContext) -> str:
    return str(getattr(context, "metadata", {}).get("current_node_name") or "").strip()


def begin_grasp_attempt(
    context: SkillContext,
    candidate: Dict[str, Any] | None,
    *,
    source: str = "",
    forced_perception_rebuild: bool = False,
    rebuild_reason: str = "",
) -> Dict[str, Any]:
    blackboard = getattr(context, "blackboard", None)
    item = dict(candidate or {})
    if blackboard is None or not hasattr(blackboard, "set") or not item:
        return item
    attempt_index = int(blackboard.get("grasp_attempt_index", 0) or 0) + 1
    blackboard.set("grasp_attempt_index", attempt_index)
    blackboard.set("grasp_attempt_initial_candidate", dict(item))
    blackboard.set("grasp_attempt_candidate", dict(item))
    blackboard.set("grasp_attempt_candidate_identity", _candidate_identity_text(item))
    blackboard.set("grasp_attempt_candidate_source", str(source or ""))
    blackboard.set("grasp_attempt_selection_node", _current_selection_node(context))
    blackboard.set("grasp_attempt_reselected", False)
    blackboard.set("grasp_attempt_reselection_node", "")
    blackboard.set("grasp_attempt_reselection_skill", "")
    blackboard.set("grasp_attempt_reselection_reason", "")
    blackboard.set("grasp_attempt_forced_perception_rebuild", bool(forced_perception_rebuild))
    blackboard.set("grasp_attempt_forced_perception_rebuild_reason", str(rebuild_reason or ""))
    return item


def ensure_grasp_attempt_candidate(
    context: SkillContext,
    candidate: Dict[str, Any] | None,
    *,
    source: str = "",
) -> Dict[str, Any]:
    blackboard = getattr(context, "blackboard", None)
    item = dict(candidate or {})
    if blackboard is None or not hasattr(blackboard, "get") or not hasattr(blackboard, "set") or not item:
        return item
    current = dict(blackboard.get("grasp_attempt_candidate") or {})
    if not current:
        return begin_grasp_attempt(context, item, source=source)
    blackboard.set("grasp_attempt_candidate", dict(item))
    blackboard.set("grasp_attempt_candidate_identity", _candidate_identity_text(item))
    if source:
        blackboard.set("grasp_attempt_candidate_source", str(source))
    return item


def reselect_grasp_attempt_candidate(
    context: SkillContext,
    candidate: Dict[str, Any] | None,
    *,
    source: str = "",
    reason: str = "",
) -> Dict[str, Any]:
    blackboard = getattr(context, "blackboard", None)
    item = dict(candidate or {})
    if blackboard is None or not hasattr(blackboard, "get") or not hasattr(blackboard, "set") or not item:
        return item
    initial = dict(blackboard.get("grasp_attempt_initial_candidate") or {})
    if not initial:
        return begin_grasp_attempt(context, item, source=source)
    blackboard.set("grasp_attempt_candidate", dict(item))
    blackboard.set("grasp_attempt_candidate_identity", _candidate_identity_text(item))
    if source:
        blackboard.set("grasp_attempt_candidate_source", str(source))
    if _candidate_identity(item) != _candidate_identity(initial):
        blackboard.set("grasp_attempt_reselected", True)
        blackboard.set("grasp_attempt_reselection_node", _current_selection_node(context))
        blackboard.set("grasp_attempt_reselection_skill", str(source or context.world_state.execution.current_skill or ""))
        blackboard.set("grasp_attempt_reselection_reason", str(reason or ""))
    return item


def grasp_attempt_trace_context(context: SkillContext) -> Dict[str, Any]:
    blackboard = getattr(context, "blackboard", None)
    if blackboard is None or not hasattr(blackboard, "get"):
        return {}
    current = dict(blackboard.get("grasp_attempt_candidate") or {})
    initial = dict(blackboard.get("grasp_attempt_initial_candidate") or {})
    if not current and not initial:
        return {}
    values: Dict[str, Any] = {
        "grasp_attempt_index": int(blackboard.get("grasp_attempt_index", 0) or 0),
        "grasp_attempt_candidate": _candidate_trace_brief(current),
        "grasp_attempt_initial_candidate": _candidate_trace_brief(initial),
        "grasp_attempt_candidate_identity": str(blackboard.get("grasp_attempt_candidate_identity", "") or ""),
        "grasp_attempt_candidate_source": str(blackboard.get("grasp_attempt_candidate_source", "") or ""),
        "grasp_attempt_selection_node": str(blackboard.get("grasp_attempt_selection_node", "") or ""),
        "grasp_attempt_reselected": bool(blackboard.get("grasp_attempt_reselected", False)),
        "grasp_attempt_reselection_node": str(blackboard.get("grasp_attempt_reselection_node", "") or ""),
        "grasp_attempt_reselection_skill": str(blackboard.get("grasp_attempt_reselection_skill", "") or ""),
        "grasp_attempt_reselection_reason": str(blackboard.get("grasp_attempt_reselection_reason", "") or ""),
        "grasp_attempt_forced_perception_rebuild": bool(blackboard.get("grasp_attempt_forced_perception_rebuild", False)),
        "grasp_attempt_forced_perception_rebuild_reason": str(
            blackboard.get("grasp_attempt_forced_perception_rebuild_reason", "") or ""
        ),
    }
    return values
