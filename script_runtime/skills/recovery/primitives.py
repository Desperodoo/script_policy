"""Recovery skills."""

from __future__ import annotations

from script_runtime.core.failure_codes import FailureCode
from script_runtime.core.result_types import SkillResult
from script_runtime.core.skill_base import Skill, SkillContext


class SafeRetreat(Skill):
    def __init__(self):
        super().__init__(name="SafeRetreat", timeout_s=2.0, failure_code=FailureCode.COLLISION_RISK)

    def run(self, context: SkillContext) -> SkillResult:
        sdk = context.adapters["sdk"]
        retreat_pose = context.blackboard.get("retreat_pose", context.world_state.scene.place_pose)
        current_pose = context.world_state.robot.eef_pose or context.blackboard.get("active_place_release_pose")
        home_joints = context.blackboard.get("home_joints")
        if self._already_safe(context, current_pose):
            return SkillResult.success(
                skipped=True,
                fallback_used="already_safe_clearance",
                reason="eef already clear without grasped object",
            )

        retreat_candidates = []
        if retreat_pose is not None:
            retreat_candidates.append(("retreat_pose", "move_l", retreat_pose))
        if current_pose and len(current_pose) >= 7:
            lifted_pose = list(current_pose)
            lifted_pose[2] += 0.08
            retreat_candidates.append(("lifted_current_pose", "move_l", lifted_pose))
        if home_joints:
            retreat_candidates.append(("home_joints", "move_j", home_joints))

        attempted = []
        for label, action, target in retreat_candidates:
            if action == "move_l":
                result = sdk.move_l(target, speed=0.3)
            else:
                result = sdk.move_j(target, speed=0.3)
            attempted.append({"label": label, "action": action, "ok": bool(result.get("ok", False))})
            if result.get("ok", False):
                if hasattr(sdk, "refresh_world"):
                    sdk.refresh_world(context.blackboard)
                return SkillResult.success(command=result["command"], fallback_used=label if label != "retreat_pose" else "")

        return SkillResult.failure(
            FailureCode.SDK_ERROR,
            message="Safe retreat failed",
            sdk_result={"attempted_targets": attempted},
        )

    def _already_safe(self, context: SkillContext, current_pose):
        if context.world_state.scene.grasped:
            return False
        if not current_pose or len(current_pose) < 3:
            return False
        object_pose = context.world_state.scene.object_pose or context.blackboard.get("object_pose")
        place_pose = context.world_state.scene.place_pose or context.blackboard.get("place_pose")
        current_z = float(current_pose[2])
        object_safe = bool(object_pose and len(object_pose) >= 3 and current_z >= float(object_pose[2]) + 0.12)
        place_safe = bool(place_pose and len(place_pose) >= 3 and current_z >= float(place_pose[2]) + 0.05)
        return object_safe or place_safe


class RetryWithNextCandidate(Skill):
    def __init__(self):
        super().__init__(name="RetryWithNextCandidate", timeout_s=0.2, failure_code=FailureCode.NO_GRASP_CANDIDATE)

    def run(self, context: SkillContext) -> SkillResult:
        candidates = list(context.world_state.learned.grasp_candidates)
        if len(candidates) <= 1:
            return SkillResult.failure(FailureCode.NO_GRASP_CANDIDATE, message="No next candidate available")
        candidates.pop(0)
        context.blackboard.update_world(learned={"grasp_candidates": candidates})
        context.blackboard.set("active_grasp_candidate", candidates[0])
        if candidates[0].get("pose") is not None:
            context.blackboard.set("active_grasp_pose", candidates[0]["pose"])
        if candidates[0].get("pregrasp_pose") is not None:
            context.blackboard.set("pregrasp_pose", candidates[0]["pregrasp_pose"])
        return SkillResult.success(next_candidate=candidates[0])


class HumanTakeover(Skill):
    def __init__(self):
        super().__init__(name="HumanTakeover", timeout_s=0.1, failure_code=FailureCode.HUMAN_ABORT)

    def run(self, context: SkillContext) -> SkillResult:
        context.blackboard.update_world(
            execution={
                "active_source": "human",
                "control_owner": "upper_machine",
                "previous_failure_code": FailureCode.HUMAN_ABORT,
            }
        )
        return SkillResult.failure(FailureCode.HUMAN_ABORT, message="Escalated to human takeover")
