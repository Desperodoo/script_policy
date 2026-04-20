"""Perception-oriented skills."""

from __future__ import annotations

from typing import Any, List

from script_runtime.adapters import PerceptionObservation
from script_runtime.core.failure_codes import FailureCode
from script_runtime.core.result_types import RecoveryAction, SkillResult
from script_runtime.core.skill_base import Skill, SkillContext
from script_runtime.planning import (
    annotate_grasp_candidates,
    resolve_grasp_semantic_policy,
    resolve_runtime_task_name,
    sort_grasp_candidates_by_semantics,
)


def _build_observation(context: SkillContext) -> PerceptionObservation:
    camera = context.adapters.get("camera")
    snapshot = camera.get_snapshot() if camera is not None and hasattr(camera, "get_snapshot") else None
    metadata = {} if snapshot is None else dict(getattr(snapshot, "metadata", {}) or {})
    metadata.update(
        {
            "task_id": context.task_id or context.world_state.execution.task_id,
            "calibration_version": None if snapshot is None else snapshot.calibration_version,
        }
    )
    return PerceptionObservation(
        rgb=None if snapshot is None else snapshot.rgb,
        depth=None if snapshot is None else snapshot.depth,
        task_goal=dict(context.world_state.execution.task_goal),
        metadata=metadata,
    )


class GetObjectPose(Skill):
    def __init__(self):
        super().__init__(name="GetObjectPose", timeout_s=1.0, failure_code=FailureCode.NO_OBJECT_DETECTED)

    def run(self, context: SkillContext) -> SkillResult:
        observation = _build_observation(context)
        pose_provider = context.adapters.get("perception")
        fallback_pose_provider = context.adapters.get("maniskill") or context.adapters.get("sdk")
        pose = None
        pose_source = "none"
        if pose_provider is not None and hasattr(pose_provider, "get_object_pose"):
            try:
                pose = pose_provider.get_object_pose(observation, context=context)
            except TypeError:
                pose = pose_provider.get_object_pose()
            if pose is not None:
                pose_source = str(getattr(pose_provider, "last_pose_source", "perception_adapter"))
            if hasattr(pose_provider, "get_place_pose"):
                try:
                    place_pose = pose_provider.get_place_pose(observation, context=context)
                except TypeError:
                    place_pose = pose_provider.get_place_pose()
                if place_pose is not None:
                    context.blackboard.update_world(scene={"place_pose": place_pose})
                    context.blackboard.set("place_pose", place_pose)
            if hasattr(pose_provider, "get_place_release_pose"):
                try:
                    release_pose = pose_provider.get_place_release_pose(observation, context=context)
                except TypeError:
                    release_pose = pose_provider.get_place_release_pose()
                if release_pose is not None:
                    context.blackboard.set("place_release_pose", release_pose)
            if hasattr(pose_provider, "get_retreat_pose"):
                try:
                    retreat_pose = pose_provider.get_retreat_pose(observation, context=context)
                except TypeError:
                    retreat_pose = pose_provider.get_retreat_pose()
                if retreat_pose is not None:
                    context.blackboard.set("retreat_pose", retreat_pose)
        if pose is None and fallback_pose_provider is not None and hasattr(fallback_pose_provider, "get_object_pose"):
            pose = fallback_pose_provider.get_object_pose()
            if pose is not None:
                pose_source = "oracle_fallback"
            if hasattr(fallback_pose_provider, "get_place_pose"):
                place_pose = fallback_pose_provider.get_place_pose()
                if place_pose is not None:
                    context.blackboard.update_world(scene={"place_pose": place_pose})
                    context.blackboard.set("place_pose", place_pose)
            if hasattr(fallback_pose_provider, "get_place_release_pose"):
                release_pose = fallback_pose_provider.get_place_release_pose()
                if release_pose is not None:
                    context.blackboard.set("place_release_pose", release_pose)
            if hasattr(fallback_pose_provider, "get_retreat_pose"):
                retreat_pose = fallback_pose_provider.get_retreat_pose()
                if retreat_pose is not None:
                    context.blackboard.set("retreat_pose", retreat_pose)
        if pose is None:
            pose = context.blackboard.get("object_pose", context.world_state.scene.object_pose)
        if pose is None:
            return SkillResult.failure(FailureCode.NO_OBJECT_DETECTED, message="Object pose unavailable")
        context.blackboard.update_world(scene={"object_pose": pose})
        pose_diagnostics = list(getattr(pose_provider, "last_pose_diagnostics", []) or [])
        grounding_diagnostics = list(getattr(pose_provider, "last_target_diagnostics", []) or [])
        component_diagnostics = list(getattr(pose_provider, "last_component_diagnostics", []) or [])
        target_stage_summary = dict(getattr(pose_provider, "last_target_stage_summary", {}) or {})
        pose_stage_summary = dict(getattr(pose_provider, "last_pose_stage_summary", {}) or {})
        if pose_diagnostics:
            context.blackboard.set("object_pose_diagnostics", pose_diagnostics)
        if grounding_diagnostics:
            context.blackboard.set("target_grounding_diagnostics", grounding_diagnostics)
        if component_diagnostics:
            context.blackboard.set("object_component_diagnostics", component_diagnostics)
        if target_stage_summary:
            context.blackboard.set("target_grounding_stage_summary", target_stage_summary)
        if pose_stage_summary:
            context.blackboard.set("object_pose_stage_summary", pose_stage_summary)
        return SkillResult.success(
            object_pose=pose,
            perception_source=pose_source,
            selected_backend=pose_stage_summary.get("selected_backend", pose_source),
            fallback_reason=pose_stage_summary.get("fallback_reason", ""),
            target_grounding_stage_summary=target_stage_summary,
            object_pose_stage_summary=pose_stage_summary,
            object_pose_diagnostics=pose_diagnostics,
            target_grounding_diagnostics=grounding_diagnostics,
            object_component_diagnostics=component_diagnostics,
            used_camera=observation.rgb is not None or observation.depth is not None,
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="ReacquirePerception")


class GetGraspCandidates(Skill):
    def __init__(self):
        super().__init__(name="GetGraspCandidates", timeout_s=1.0, failure_code=FailureCode.NO_GRASP_CANDIDATE)

    def run(self, context: SkillContext) -> SkillResult:
        observation = _build_observation(context)
        candidate_provider = context.adapters.get("perception")
        fallback_provider = context.adapters.get("maniskill") or context.adapters.get("sdk")
        candidates = None
        candidate_source = "none"
        if candidate_provider is not None and hasattr(candidate_provider, "get_grasp_candidates"):
            try:
                candidates = candidate_provider.get_grasp_candidates(observation, context=context)
            except TypeError:
                candidates = candidate_provider.get_grasp_candidates()
            if candidates is not None:
                candidate_source = str(getattr(candidate_provider, "last_grasp_source", "perception_adapter"))
        if candidates is None and fallback_provider is not None and hasattr(fallback_provider, "get_grasp_candidates"):
            candidates = fallback_provider.get_grasp_candidates()
            if candidates is not None:
                candidate_source = "oracle_fallback"
        if candidates is None:
            candidates = context.blackboard.get("grasp_candidates", context.world_state.learned.grasp_candidates)
            if candidates:
                candidate_source = "blackboard_cache"
        if not candidates:
            return SkillResult.failure(FailureCode.NO_GRASP_CANDIDATE, message="No grasp candidates")

        learned = context.adapters.get("learned")
        if learned is not None:
            scores = learned.score(candidates, context.world_state, context)
            enriched = []
            for candidate, score in zip(candidates, scores):
                item = dict(candidate)
                item["score"] = score
                enriched.append(item)
            candidates = sorted(enriched, key=lambda item: item.get("score", 0.0), reverse=True)
        task_name = resolve_runtime_task_name(candidate_provider, fallback_provider, context=context)
        candidates = annotate_grasp_candidates(task_name, candidates, blackboard=context.blackboard)
        candidates = sort_grasp_candidates_by_semantics(candidates)
        semantic_policy = resolve_grasp_semantic_policy(task_name, blackboard=context.blackboard)
        context.blackboard.update_world(learned={"grasp_candidates": candidates})
        context.blackboard.set("active_grasp_candidate", candidates[0])
        if candidates[0].get("pose") is not None:
            context.blackboard.set("active_grasp_pose", candidates[0]["pose"])
        if candidates[0].get("pregrasp_pose") is not None:
            context.blackboard.set("pregrasp_pose", candidates[0]["pregrasp_pose"])
        context.blackboard.set("grasp_semantic_policy", semantic_policy)
        context.blackboard.set("visual_review_required", bool(semantic_policy.get("visual_review_required", False)))
        planner_diagnostics = []
        if candidate_provider is not None:
            planner_diagnostics = list(getattr(candidate_provider, "last_grasp_diagnostics", []) or [])
        grounding_diagnostics = []
        if candidate_provider is not None:
            grounding_diagnostics = list(getattr(candidate_provider, "last_target_diagnostics", []) or [])
        target_stage_summary = dict(getattr(candidate_provider, "last_target_stage_summary", {}) or {})
        grasp_stage_summary = dict(getattr(candidate_provider, "last_grasp_stage_summary", {}) or {})
        context.blackboard.set("grasp_candidate_diagnostics", planner_diagnostics)
        if grounding_diagnostics:
            context.blackboard.set("target_grounding_diagnostics", grounding_diagnostics)
        if target_stage_summary:
            context.blackboard.set("target_grounding_stage_summary", target_stage_summary)
        if grasp_stage_summary:
            context.blackboard.set("grasp_candidate_stage_summary", grasp_stage_summary)
        return SkillResult.success(
            grasp_candidates=candidates,
            candidate_source=candidate_source,
            selected_backend=grasp_stage_summary.get("selected_backend", candidate_source),
            selected_backend_kind=grasp_stage_summary.get("selected_backend_kind", ""),
            fallback_reason=grasp_stage_summary.get("fallback_reason", ""),
            guided_feasible_families=list(grasp_stage_summary.get("guided_feasible_families", []) or []),
            has_guided_feasible_family=bool(grasp_stage_summary.get("has_guided_feasible_family", False)),
            target_grounding_stage_summary=target_stage_summary,
            grasp_candidate_stage_summary=grasp_stage_summary,
            grasp_candidate_diagnostics=planner_diagnostics,
            target_grounding_diagnostics=grounding_diagnostics,
            grasp_semantic_policy=semantic_policy,
            active_grasp_affordance=candidates[0].get("affordance", {}),
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="ReacquirePerception")


class ReacquirePerception(Skill):
    def __init__(self):
        super().__init__(name="ReacquirePerception", timeout_s=2.0, failure_code=FailureCode.PERCEPTION_LOST)

    def run(self, context: SkillContext) -> SkillResult:
        camera = context.adapters.get("camera")
        snapshot = camera.get_snapshot() if camera is not None else None
        if snapshot is None:
            return SkillResult.failure(FailureCode.PERCEPTION_LOST, message="Camera snapshot unavailable")
        context.blackboard.update_world(
            perception={"tracking_lost": False, "calibration_version": snapshot.calibration_version}
        )
        return SkillResult.success(calibration_version=snapshot.calibration_version)
