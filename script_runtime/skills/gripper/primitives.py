"""Primitive gripper skills."""

from __future__ import annotations

from typing import Optional

from script_runtime.core.failure_codes import FailureCode
from script_runtime.core.result_types import RecoveryAction, SkillResult
from script_runtime.core.skill_base import Skill, SkillContext, request_world_refresh


def _trace_snapshot(sdk, label: str):
    if hasattr(sdk, "get_trace_snapshot"):
        try:
            return dict(sdk.get_trace_snapshot(label=label) or {})
        except Exception as exc:
            return {"label": label, "snapshot_error": repr(exc)}
    return {"label": label}


class OpenGripper(Skill):
    def __init__(self, width: Optional[float] = None):
        super().__init__(name="OpenGripper", timeout_s=2.0, failure_code=FailureCode.SDK_ERROR)
        self.width = width

    def run(self, context: SkillContext) -> SkillResult:
        sdk = context.adapters["sdk"]
        if hasattr(sdk, "should_release_object") and not sdk.should_release_object():
            context.blackboard.set("skip_release_sequence", True)
            return SkillResult.success(skipped_release=True, reason="goal_requires_hold")

        context.blackboard.set("skip_release_sequence", False)
        state_before = _trace_snapshot(sdk, "open_gripper_before")
        width = self.width
        if width is None:
            width = context.blackboard.get("open_gripper_width")
        result = sdk.open_gripper(width)
        if not result.get("ok", False):
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="open_gripper failed",
                sdk_result=result,
                state_before=state_before,
            )
        request_world_refresh(context, sdk, reason="post_OpenGripper")
        context.blackboard.update_world(scene={"grasped": False})
        return SkillResult.success(
            command=result["command"],
            state_before=state_before,
            state_after=_trace_snapshot(sdk, "open_gripper_after"),
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )


class PrepareGripperForGrasp(Skill):
    def __init__(self, width: Optional[float] = None):
        super().__init__(name="PrepareGripperForGrasp", timeout_s=10.0, failure_code=FailureCode.SDK_ERROR)
        self.width = width

    def run(self, context: SkillContext) -> SkillResult:
        sdk = context.adapters["sdk"]
        state_before = _trace_snapshot(sdk, "prepare_gripper_before")
        width = self.width
        if width is None:
            width = context.blackboard.get("open_gripper_width")
        result = sdk.open_gripper(width)
        if not result.get("ok", False):
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="prepare gripper failed",
                sdk_result=result,
                state_before=state_before,
            )
        request_world_refresh(context, sdk, reason="post_PrepareGripperForGrasp")
        context.blackboard.update_world(scene={"grasped": False})
        return SkillResult.success(
            command=result["command"],
            state_before=state_before,
            state_after=_trace_snapshot(sdk, "prepare_gripper_after"),
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )


class CloseGripper(Skill):
    def __init__(self, width: Optional[float] = None):
        super().__init__(name="CloseGripper", timeout_s=2.0, failure_code=FailureCode.GRASP_FAIL)
        self.width = width

    def run(self, context: SkillContext) -> SkillResult:
        width = self.width
        if width is None:
            width = context.blackboard.get("close_gripper_width")
        result = context.adapters["sdk"].close_gripper(width, guarded=False)
        if not result.get("ok", False):
            return SkillResult.failure(FailureCode.SDK_ERROR, message="close_gripper failed", sdk_result=result)
        sdk = context.adapters["sdk"]
        request_world_refresh(context, sdk, reason="post_CloseGripper")
        context.blackboard.update_world(scene={"grasped": True})
        return SkillResult.success(
            command=result["command"],
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="ReacquirePerception")


class GuardedClose(Skill):
    def __init__(self, width: Optional[float] = None):
        super().__init__(name="GuardedClose", timeout_s=2.0, failure_code=FailureCode.GRASP_FAIL)
        self.width = width

    def run(self, context: SkillContext) -> SkillResult:
        width = self.width
        if width is None:
            width = context.blackboard.get("close_gripper_width")
        result = context.adapters["sdk"].close_gripper(width, guarded=True)
        if not result.get("ok", False):
            return SkillResult.failure(FailureCode.SDK_ERROR, message="guarded close failed", sdk_result=result)
        sdk = context.adapters["sdk"]
        request_world_refresh(context, sdk, reason="post_GuardedClose")
        context.blackboard.update_world(scene={"grasped": True})
        return SkillResult.success(
            command=result["command"],
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="RetryWithNextCandidate")


class ExecuteGraspPhase(Skill):
    def __init__(self):
        super().__init__(name="ExecuteGraspPhase", timeout_s=6.0, failure_code=FailureCode.GRASP_FAIL)

    def run(self, context: SkillContext) -> SkillResult:
        target_pose = context.blackboard.get("active_grasp_pose")
        if not target_pose:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing active grasp pose")

        sdk = context.adapters["sdk"]
        result = sdk.execute_grasp_phase(target_pose, context=context)
        request_world_refresh(context, sdk, reason="post_ExecuteGraspPhase")
        grasp_refresh = context.blackboard.get("last_grasp_candidate_refresh")

        maniskill = context.adapters.get("maniskill")
        if maniskill is not None and hasattr(maniskill, "is_grasped"):
            grasped = bool(maniskill.is_grasped())
        else:
            grasped = bool(result.get("ok", False) or context.world_state.scene.grasped)
        context.blackboard.update_world(scene={"grasped": grasped})

        if not result.get("ok", False) or not grasped:
            active_candidate = context.blackboard.get("active_grasp_candidate")
            if active_candidate is not None:
                context.blackboard.set("last_failed_grasp_candidate", dict(active_candidate))
            return SkillResult.failure(
                FailureCode.GRASP_FAIL,
                message=result.get("message", "Grasp phase did not secure object"),
                sdk_result=result,
                grasp_candidate_refresh=grasp_refresh,
            )
        context.blackboard.delete("last_failed_grasp_candidate")
        eef_pose = list(context.world_state.robot.eef_pose)
        if len(eef_pose) >= 7:
            lift_pose = list(eef_pose)
            lift_pose[2] += 0.10
            context.blackboard.set("lift_pose", lift_pose)
            context.blackboard.set("grasp_anchor_eef_pose", eef_pose)
        object_pose = list(context.world_state.scene.object_pose or [])
        if len(object_pose) >= 7:
            context.blackboard.set("grasp_anchor_object_pose", object_pose)
        return SkillResult.success(
            command=result.get("command", {}),
            grasped=grasped,
            sdk_result=result,
            grasp_diagnostics=result.get("grasp_diagnostics"),
            grasp_candidate_refresh=grasp_refresh,
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="RetryWithNextCandidate")

    def trace_payload(self, context: SkillContext, result: SkillResult):
        return {
            "message": result.message,
            "payload": result.payload,
            "eef_pose": list(context.world_state.robot.eef_pose),
            "grasped": bool(context.world_state.scene.grasped),
        }


class ResetGripper(OpenGripper):
    def __init__(self, width: Optional[float] = None):
        super().__init__(width=width)
        self.name = "ResetGripper"
