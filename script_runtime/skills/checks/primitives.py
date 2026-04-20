"""Check and guard skills."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from script_runtime.core.failure_codes import FailureCode
from script_runtime.core.result_types import SkillResult
from script_runtime.core.skill_base import Skill, SkillContext, request_world_refresh
from script_runtime.planning import build_grasp_semantic_report, resolve_runtime_task_name


def _trace_snapshot(sdk, label: str):
    if hasattr(sdk, "get_trace_snapshot"):
        try:
            return dict(sdk.get_trace_snapshot(label=label) or {})
        except Exception as exc:
            return {"label": label, "snapshot_error": repr(exc)}
    return {"label": label}


def _candidate_identity(candidate: Dict[str, Any]) -> tuple:
    contact_id = candidate.get("contact_point_id")
    if contact_id is not None:
        return ("contact", int(contact_id), str(candidate.get("arm", "") or ""))
    pose = tuple(round(float(v), 5) for v in list(candidate.get("pose") or [])[:7])
    return ("pose", str(candidate.get("arm", "") or ""), pose)


def _candidate_summary(candidate: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "variant_label": str(candidate.get("variant_label", "") or ""),
        "contact_point_id": candidate.get("contact_point_id"),
        "planner_status": str(candidate.get("planner_status", "Unknown") or "Unknown"),
        "planner_waypoint_count": candidate.get("planner_waypoint_count"),
        "task_compatibility": str(candidate.get("task_compatibility", "unknown") or "unknown"),
        "score": float(candidate.get("score", 0.0)),
        "pose_xyz": [float(v) for v in list(candidate.get("pose") or [])[:3]],
        "pregrasp_xyz": [float(v) for v in list(candidate.get("pregrasp_pose") or [])[:3]],
    }


class ReselectGraspAfterPregrasp(Skill):
    def __init__(self):
        super().__init__(name="ReselectGraspAfterPregrasp", timeout_s=0.2, failure_code=FailureCode.NO_GRASP_CANDIDATE)

    def run(self, context: SkillContext) -> SkillResult:
        if not bool(context.blackboard.get("post_pregrasp_reselection_enabled", True)):
            return SkillResult.success(reselected=False, skipped=True, reason="post_pregrasp_reselection_disabled")

        refresh = dict(context.blackboard.get("last_grasp_candidate_refresh") or {})
        if str(refresh.get("refresh_reason", "")) != "post_GoPregrasp":
            return SkillResult.success(
                reselected=False,
                skipped=True,
                reason="no_post_go_pregrasp_refresh",
                grasp_candidate_refresh=refresh,
            )

        candidates = list(context.blackboard.get("grasp_candidates", context.world_state.learned.grasp_candidates) or [])
        active = dict(context.blackboard.get("active_grasp_candidate") or {})
        if not candidates or not active:
            return SkillResult.success(
                reselected=False,
                skipped=True,
                reason="missing_candidates_or_active",
                grasp_candidate_refresh=refresh,
            )

        selected, reselection_reason = self._select_candidate(candidates=candidates, active=active, refresh=refresh, context=context)
        if selected is None:
            return SkillResult.success(
                reselected=False,
                skipped=True,
                reason="no_improved_candidate_to_promote",
                grasp_candidate_refresh=refresh,
                active_candidate=_candidate_summary(active),
            )

        reordered = [selected] + [candidate for candidate in candidates if _candidate_identity(candidate) != _candidate_identity(selected)]
        context.blackboard.update_world(learned={"grasp_candidates": reordered})
        context.blackboard.set("grasp_candidates", reordered)
        context.blackboard.set("active_grasp_candidate", selected)
        if selected.get("pose") is not None:
            context.blackboard.set("active_grasp_pose", selected["pose"])
        if selected.get("pregrasp_pose") is not None:
            context.blackboard.set("pregrasp_pose", selected["pregrasp_pose"])

        return SkillResult.success(
            reselected=True,
            reselection_reason=reselection_reason,
            previous_active_candidate=_candidate_summary(active),
            new_active_candidate=_candidate_summary(selected),
            grasp_candidate_refresh=refresh,
            reordered_candidates=[_candidate_summary(candidate) for candidate in reordered[:6]],
        )

    def _select_candidate(
        self,
        *,
        candidates: List[Dict[str, Any]],
        active: Dict[str, Any],
        refresh: Dict[str, Any],
        context: SkillContext,
    ) -> tuple[Optional[Dict[str, Any]], str]:
        selected = self._select_improved_candidate(candidates=candidates, active=active, refresh=refresh)
        if selected is not None:
            return selected, "improved_after_pregrasp"
        selected = self._select_alternative_after_recent_failure(candidates=candidates, active=active, context=context)
        if selected is not None:
            return selected, "avoid_recent_failed_active_candidate"
        return None, ""

    def _select_improved_candidate(
        self,
        *,
        candidates: List[Dict[str, Any]],
        active: Dict[str, Any],
        refresh: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        active_key = _candidate_identity(active)
        candidate_by_key = {_candidate_identity(candidate): candidate for candidate in candidates}
        improved_rows = []
        for row in list(refresh.get("improved_candidates") or []):
            current = dict(row.get("current") or {})
            key = _candidate_identity(current)
            candidate = candidate_by_key.get(key)
            if candidate is None or key == active_key:
                continue
            if str(candidate.get("planner_status", "Unknown") or "Unknown") != "Success":
                continue
            improved_rows.append((row, candidate))
        if not improved_rows:
            return None

        def _priority(item):
            row, candidate = item
            current = dict(row.get("current") or {})
            compatibility = str(candidate.get("task_compatibility", "unknown") or "unknown")
            compatibility_rank = {"preferred": 0, "compatible": 1, "unknown": 2, "incompatible": 3}.get(
                compatibility,
                2,
            )
            rank = int(current.get("rank", 10**6))
            waypoint_count = int(candidate.get("planner_waypoint_count") or 10**6)
            score = -float(candidate.get("score", 0.0))
            return compatibility_rank, rank, waypoint_count, score

        improved_rows.sort(key=_priority)
        return improved_rows[0][1]

    def _select_alternative_after_recent_failure(
        self,
        *,
        candidates: List[Dict[str, Any]],
        active: Dict[str, Any],
        context: SkillContext,
    ) -> Optional[Dict[str, Any]]:
        failed_candidate = dict(context.blackboard.get("last_failed_grasp_candidate") or {})
        if not failed_candidate:
            return None
        active_key = _candidate_identity(active)
        failed_key = _candidate_identity(failed_candidate)
        if active_key != failed_key:
            return None

        alternatives = []
        for index, candidate in enumerate(candidates):
            key = _candidate_identity(candidate)
            if key == active_key:
                continue
            compatibility = str(candidate.get("task_compatibility", "unknown") or "unknown")
            if compatibility == "incompatible":
                continue
            status = str(candidate.get("planner_status", "Unknown") or "Unknown")
            if status != "Success":
                continue
            alternatives.append((index, candidate))
        if not alternatives:
            return None

        def _priority(item):
            index, candidate = item
            compatibility = str(candidate.get("task_compatibility", "unknown") or "unknown")
            compatibility_rank = {"preferred": 0, "compatible": 1, "unknown": 2}.get(compatibility, 3)
            waypoint_count = int(candidate.get("planner_waypoint_count") or 10**6)
            score = -float(candidate.get("score", 0.0))
            return compatibility_rank, waypoint_count, index, score

        alternatives.sort(key=_priority)
        return alternatives[0][1]


class CheckSceneReady(Skill):
    def __init__(self):
        super().__init__(name="CheckSceneReady", timeout_s=0.2, failure_code=FailureCode.PERCEPTION_LOST)

    def run(self, context: SkillContext) -> SkillResult:
        scene = context.world_state.scene
        perception = context.world_state.perception
        if not scene.workspace_ready:
            return SkillResult.failure(FailureCode.WORKSPACE_VIOLATION, message="Workspace not ready")
        if perception.tracking_lost or perception.depth_anomaly:
            return SkillResult.failure(FailureCode.PERCEPTION_LOST, message="Perception degraded")
        return SkillResult.success(scene_ready=True)


class WaitForObjectStable(Skill):
    def __init__(self):
        super().__init__(name="WaitForObjectStable", timeout_s=0.5, failure_code=FailureCode.PERCEPTION_LOST)

    def run(self, context: SkillContext) -> SkillResult:
        conf = context.world_state.perception.detection_confidence
        if conf < 0.5:
            return SkillResult.failure(FailureCode.PERCEPTION_LOST, message="Object not stable enough")
        return SkillResult.success(stable=True)


class CheckGrasp(Skill):
    def __init__(self):
        super().__init__(name="CheckGrasp", timeout_s=0.5, failure_code=FailureCode.GRASP_FAIL)

    def run(self, context: SkillContext) -> SkillResult:
        grasp_adapter = context.adapters.get("maniskill") or context.adapters.get("sdk")
        grasped = bool(context.world_state.scene.grasped)
        if grasp_adapter is not None and hasattr(grasp_adapter, "is_grasped"):
            grasped = bool(grasp_adapter.is_grasped())
            context.blackboard.update_world(scene={"grasped": grasped})
        candidate = context.blackboard.get("active_grasp_candidate")
        task_name = resolve_runtime_task_name(grasp_adapter, context.adapters.get("perception"), context=context)
        if grasp_adapter is not None and hasattr(grasp_adapter, "evaluate_grasp_semantics"):
            semantic_report = dict(grasp_adapter.evaluate_grasp_semantics(context=context) or {})
        else:
            semantic_report = build_grasp_semantic_report(
                task_name,
                candidate,
                grasped=grasped,
                object_pose=context.world_state.scene.object_pose,
                eef_pose=context.world_state.robot.eef_pose,
                blackboard=context.blackboard,
            )
        context.blackboard.set("last_grasp_semantic_report", semantic_report)
        if not grasped:
            return SkillResult.failure(FailureCode.GRASP_FAIL, message="Grasp not confirmed")
        if not semantic_report.get("ok", True):
            return SkillResult.failure(
                FailureCode.GRASP_FAIL,
                message=str(semantic_report.get("message", "Grasp semantics not accepted")),
                grasp_confirmed=grasped,
                grasp_semantic_report=semantic_report,
            )
        return SkillResult.success(
            grasp_confirmed=True,
            grasp_semantics_ok=True,
            grasp_semantic_report=semantic_report,
        )


class CheckContact(Skill):
    def __init__(self):
        super().__init__(name="CheckContact", timeout_s=0.5, failure_code=FailureCode.CONTACT_ANOMALY)

    def run(self, context: SkillContext) -> SkillResult:
        state = context.world_state.scene.contact_state
        if state in ("anomaly", "unexpected"):
            return SkillResult.failure(FailureCode.CONTACT_ANOMALY, message=f"Unexpected contact: {state}")
        return SkillResult.success(contact_state=state)


class CheckTaskSuccess(Skill):
    def __init__(self):
        super().__init__(name="CheckTaskSuccess", timeout_s=2.0, failure_code=FailureCode.UNKNOWN)

    def run(self, context: SkillContext) -> SkillResult:
        sdk = context.adapters.get("sdk")
        before_settle = _trace_snapshot(sdk, "check_task_success_before_settle") if sdk is not None else {}
        if sdk is not None and hasattr(sdk, "settle"):
            sdk.settle()
            request_world_refresh(context, sdk, reason="post_CheckTaskSuccess_settle")
        after_settle = _trace_snapshot(sdk, "check_task_success_after_settle") if sdk is not None else {}

        if sdk is None or not hasattr(sdk, "evaluate_task_success"):
            return SkillResult.success(
                env_success=True,
                mode="no_eval_hook",
                before_settle_snapshot=before_settle,
                after_settle_snapshot=after_settle,
            )

        result = sdk.evaluate_task_success()
        if not result.get("ok", False):
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="Task success evaluation failed",
                sdk_result=result,
                before_settle_snapshot=before_settle,
                after_settle_snapshot=after_settle,
            )
        if not result.get("success", False):
            return SkillResult.failure(
                FailureCode.UNKNOWN,
                message="Environment success check failed",
                env_result=result,
                before_settle_snapshot=before_settle,
                after_settle_snapshot=after_settle,
            )
        return SkillResult.success(
            env_success=True,
            env_result=result,
            before_settle_snapshot=before_settle,
            after_settle_snapshot=after_settle,
        )
