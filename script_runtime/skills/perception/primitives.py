"""Perception-oriented skills."""

from __future__ import annotations

from typing import Any, List

from script_runtime.adapters import PerceptionObservation
from script_runtime.core.failure_codes import FailureCode
from script_runtime.core.result_types import RecoveryAction, SkillResult
from script_runtime.core.skill_base import Skill, SkillContext


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
        return SkillResult.success(
            object_pose=pose,
            perception_source=pose_source,
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
        context.blackboard.update_world(learned={"grasp_candidates": candidates})
        context.blackboard.set("active_grasp_candidate", candidates[0])
        if candidates[0].get("pose") is not None:
            context.blackboard.set("active_grasp_pose", candidates[0]["pose"])
        if candidates[0].get("pregrasp_pose") is not None:
            context.blackboard.set("pregrasp_pose", candidates[0]["pregrasp_pose"])
        return SkillResult.success(grasp_candidates=candidates, candidate_source=candidate_source)

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
