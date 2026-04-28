"""Primitive motion skills."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from script_runtime.core.failure_codes import FailureCode
from script_runtime.place import resolve_place_module
from script_runtime.core.result_types import RecoveryAction, SkillResult
from script_runtime.core.skill_base import (
    Skill,
    SkillContext,
    request_world_refresh,
    set_support_regrasp_substage,
    support_regrasp_trace_context,
)


def _trace_snapshot(sdk: Any, label: str) -> Dict[str, Any]:
    if hasattr(sdk, "get_trace_snapshot"):
        try:
            return dict(sdk.get_trace_snapshot(label=label) or {})
        except Exception as exc:
            return {"label": label, "snapshot_error": repr(exc)}
    return {"label": label}


def _motion_pose_diagnostics(sdk: Any, target_pose: List[float] | None) -> Dict[str, Any]:
    target = [float(v) for v in list(target_pose or [])]
    diagnostics: Dict[str, Any] = {
        "target_pose": list(target),
        "target_xyz": list(target[:3]),
    }
    if len(target) >= 7:
        diagnostics["target_quat"] = list(target[3:7])
    status: Dict[str, Any] = {}
    if hasattr(sdk, "get_status"):
        try:
            status = dict(sdk.get_status() or {})
        except Exception as exc:
            diagnostics["status_error"] = repr(exc)
            return diagnostics
    current_pose = [float(v) for v in list(status.get("eef_pose") or [])]
    diagnostics["current_eef_pose"] = list(current_pose)
    diagnostics["current_xyz"] = list(current_pose[:3])
    if len(current_pose) >= 7:
        diagnostics["current_quat"] = list(current_pose[3:7])
    if len(target) >= 3 and len(current_pose) >= 3:
        dx = float(target[0] - current_pose[0])
        dy = float(target[1] - current_pose[1])
        dz = float(target[2] - current_pose[2])
        diagnostics["xyz_delta"] = {"dx": dx, "dy": dy, "dz": dz}
        diagnostics["xy_norm"] = math.sqrt(dx * dx + dy * dy)
        diagnostics["xyz_norm"] = math.sqrt(dx * dx + dy * dy + dz * dz)
    if len(target) >= 7 and len(current_pose) >= 7:
        dot = sum(float(a) * float(b) for a, b in zip(target[3:7], current_pose[3:7]))
        dot = max(min(abs(dot), 1.0), 0.0)
        diagnostics["orientation_error_rad"] = 2.0 * math.acos(dot)
    return diagnostics


@dataclass
class MotionTargetMixin:
    target_pose: Optional[List[float]] = None
    target_joints: Optional[List[float]] = None
    speed: float = 1.0


class MoveJ(Skill):
    def __init__(
        self,
        name: str = "MoveJ",
        target_joints: Optional[List[float]] = None,
        speed: float = 1.0,
        blackboard_key: str = "target_joints",
    ):
        super().__init__(name=name, timeout_s=8.0, failure_code=FailureCode.NO_IK)
        self.target_joints = target_joints
        self.speed = speed
        self.blackboard_key = blackboard_key

    def run(self, context: SkillContext) -> SkillResult:
        if self.name in {"Retreat", "GoHome"} and context.blackboard.get("skip_release_sequence", False):
            return SkillResult.success(skipped=True, reason="holding_object_for_goal")
        target = self.target_joints or context.blackboard.get(self.blackboard_key)
        if not target:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing joint target")
        sdk = context.adapters["sdk"]
        result = sdk.move_j(target, speed=self.speed)
        if not result.get("ok", False):
            if self.name == "GoHome" and not context.world_state.scene.grasped:
                return SkillResult.success(
                    cleanup_best_effort=True,
                    reason="go_home_failed_after_release",
                    sdk_result=result,
                )
            return SkillResult.failure(FailureCode.SDK_ERROR, message="move_j failed", sdk_result=result)
        request_world_refresh(context, sdk, reason=f"post_{self.name}")
        return SkillResult.success(
            command=result["command"],
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="SafeRetreat")

    def trace_payload(self, context: SkillContext, result: SkillResult):
        payload = {
            "message": result.message,
            "payload": result.payload,
        }
        payload.update(support_regrasp_trace_context(context))
        return payload


class MoveL(Skill):
    def __init__(
        self,
        name: str = "MoveL",
        target_pose: Optional[List[float]] = None,
        speed: float = 1.0,
        blackboard_key: str = "target_pose",
    ):
        super().__init__(name=name, timeout_s=8.0, failure_code=FailureCode.NO_IK)
        self.target_pose = target_pose
        self.speed = speed
        self.blackboard_key = blackboard_key

    def run(self, context: SkillContext) -> SkillResult:
        if self.name in {"Retreat", "GoHome"} and context.blackboard.get("skip_release_sequence", False):
            return SkillResult.success(skipped=True, reason="holding_object_for_goal")
        if self.name == "GoPregrasp":
            set_support_regrasp_substage(context, "support_pregrasp_motion")
        elif self.name == "Lift":
            set_support_regrasp_substage(context, "support_lift_completion")
        target = self.target_pose or context.blackboard.get(self.blackboard_key)
        if not target:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing pose target")
        sdk = context.adapters["sdk"]
        motion_before = _motion_pose_diagnostics(sdk, list(target))
        result = sdk.move_l(target, speed=self.speed)
        if not result.get("ok", False):
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="move_l failed",
                sdk_result=result,
                motion_target_pose=list(target),
                motion_diagnostics_before=motion_before,
            )
        request_world_refresh(context, sdk, reason=f"post_{self.name}")
        motion_after = _motion_pose_diagnostics(sdk, list(target))
        return SkillResult.success(
            command=result["command"],
            motion_target_pose=list(target),
            motion_diagnostics_before=motion_before,
            motion_diagnostics_after=motion_after,
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="SafeRetreat")

    def trace_payload(self, context: SkillContext, result: SkillResult):
        payload = {
            "message": result.message,
            "payload": result.payload,
        }
        payload.update(support_regrasp_trace_context(context))
        return payload


class ServoDelta(Skill):
    def __init__(self, name: str = "ServoDelta", delta_pose: Optional[List[float]] = None):
        super().__init__(name=name, timeout_s=2.0, failure_code=FailureCode.COLLISION_RISK)
        self.delta_pose = delta_pose

    def run(self, context: SkillContext) -> SkillResult:
        delta = self.delta_pose or context.blackboard.get("servo_delta")
        if not delta:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing servo delta")
        sdk = context.adapters["sdk"]
        result = sdk.servo_delta(delta)
        if not result.get("ok", False):
            return SkillResult.failure(FailureCode.SDK_ERROR, message="servo_delta failed", sdk_result=result)
        request_world_refresh(context, sdk, reason=f"post_{self.name}")
        return SkillResult.success(
            command=result["command"],
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )

    def recover(self, context: SkillContext):
        return RecoveryAction(name="Stop")


class Stop(Skill):
    def __init__(self, name: str = "Stop"):
        super().__init__(name=name, timeout_s=1.0, failure_code=FailureCode.SDK_ERROR)

    def run(self, context: SkillContext) -> SkillResult:
        result = context.adapters["sdk"].stop()
        if not result.get("ok", False):
            return SkillResult.failure(FailureCode.SDK_ERROR, message="stop failed", sdk_result=result)
        return SkillResult.success(command=result["command"])


class GoHome(MoveJ):
    def __init__(self, home_joints: Optional[List[float]] = None):
        super().__init__(
            name="GoHome",
            target_joints=home_joints,
            speed=0.8,
            blackboard_key="home_joints",
        )


class GoPregrasp(MoveL):
    def __init__(self, target_pose: Optional[List[float]] = None):
        super().__init__(
            name="GoPregrasp",
            target_pose=target_pose,
            speed=0.5,
            blackboard_key="pregrasp_pose",
        )


class Retreat(MoveL):
    def __init__(self, target_pose: Optional[List[float]] = None):
        super().__init__(
            name="Retreat",
            target_pose=target_pose,
            speed=0.5,
            blackboard_key="retreat_pose",
        )

    def run(self, context: SkillContext) -> SkillResult:
        target = self.target_pose or context.blackboard.get(self.blackboard_key)
        sdk = context.adapters["sdk"]
        state_before = _trace_snapshot(sdk, "retreat_before")

        if target:
            result = sdk.move_l(target, speed=self.speed)
            if result.get("ok", False):
                request_world_refresh(context, sdk, reason="post_Retreat")
                return SkillResult.success(
                    command=result["command"],
                    state_before=state_before,
                    state_after=_trace_snapshot(sdk, "retreat_after"),
                    grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
                )

        home_joints = context.blackboard.get("home_joints")
        if home_joints:
            fallback = sdk.move_j(home_joints, speed=min(self.speed, 0.5))
            if fallback.get("ok", False):
                request_world_refresh(context, sdk, reason="post_Retreat")
                return SkillResult.success(
                    command=fallback["command"],
                    fallback_used="move_j_home",
                    original_target=target,
                    state_before=state_before,
                    state_after=_trace_snapshot(sdk, "retreat_after"),
                    grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
                )
            if not context.world_state.scene.grasped:
                return SkillResult.success(
                    cleanup_best_effort=True,
                    reason="retreat_failed_after_release",
                    sdk_result=fallback,
                    original_target=target,
                    state_before=state_before,
                    state_after=_trace_snapshot(sdk, "retreat_after_failed"),
                )
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="retreat fallback move_j failed",
                sdk_result=fallback,
                original_target=target,
                state_before=state_before,
            )

        if not context.world_state.scene.grasped:
            return SkillResult.success(
                cleanup_best_effort=True,
                reason="retreat_failed_after_release",
                original_target=target,
                state_before=state_before,
            )
        return SkillResult.failure(
            FailureCode.SDK_ERROR,
            message="move_l failed",
            sdk_result={"ok": False, "action": "move_l", "command": {"type": "move_l", "pose": target, "speed": self.speed}},
            state_before=state_before,
        )


class Lift(MoveL):
    def __init__(self, target_pose: Optional[List[float]] = None):
        super().__init__(
            name="Lift",
            target_pose=target_pose,
            speed=0.4,
            blackboard_key="lift_pose",
        )

    @staticmethod
    def _support_lift_plans(delta_z: float) -> List[Dict[str, Any]]:
        dz = float(delta_z)
        if abs(dz) <= 1e-6:
            return []
        plans = [
            {"plan_name": "full_lift", "fractions": [1.0]},
            {"plan_name": "lift_60_40", "fractions": [0.6, 1.0]},
            {"plan_name": "lift_50_50", "fractions": [0.5, 1.0]},
            {"plan_name": "lift_35_35_30", "fractions": [0.35, 0.7, 1.0]},
        ]
        deduped: List[Dict[str, Any]] = []
        seen = set()
        for plan in plans:
            fractions = [float(value) for value in list(plan.get("fractions") or []) if abs(float(value)) > 1e-6]
            if not fractions:
                continue
            key = tuple(round(value, 6) for value in fractions)
            if key in seen:
                continue
            seen.add(key)
            deduped.append({"plan_name": str(plan.get("plan_name") or "support_lift"), "fractions": fractions})
        return deduped

    def _run_support_lift_plan(
        self,
        sdk: Any,
        *,
        plan_name: str,
        start_pose: List[float],
        delta_z: float,
        fractions: List[float],
        speed: float,
    ) -> Dict[str, Any]:
        steps: List[Dict[str, Any]] = []
        base_pose = [float(value) for value in list(start_pose or [])[:7]]
        for index, fraction in enumerate(list(fractions or [])):
            status = dict(getattr(sdk, "get_status", lambda: {})() or {})
            current_pose = list(status.get("eef_pose") or [])
            if len(current_pose) < 7:
                return {
                    "ok": False,
                    "plan_name": plan_name,
                    "failed_step_index": index,
                    "message": "Current ee pose unavailable for support lift",
                    "steps": steps,
                }
            target_pose = list(base_pose[:7])
            target_pose[2] = float(base_pose[2]) + float(delta_z) * float(fraction)
            result = dict(sdk.move_l(target_pose, speed=speed) or {})
            steps.append(
                {
                    "step_index": index,
                    "delta_xyz": [
                        float(target_pose[0] - current_pose[0]),
                        float(target_pose[1] - current_pose[1]),
                        float(target_pose[2] - current_pose[2]),
                    ],
                    "start_pose": [float(value) for value in current_pose[:7]],
                    "target_pose": [float(value) for value in target_pose[:7]],
                    "sdk_result": result,
                }
            )
            if not result.get("ok", False):
                return {
                    "ok": False,
                    "plan_name": plan_name,
                    "failed_step_index": index,
                    "sdk_result": result,
                    "steps": steps,
                }
        return {"ok": True, "plan_name": plan_name, "steps": steps}

    def run(self, context: SkillContext) -> SkillResult:
        support_regrasp_active = bool(context.blackboard.get("probe_support_regrasp_active", False))
        target = self.target_pose or context.blackboard.get(self.blackboard_key)
        if not target:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing pose target")
        sdk = context.adapters["sdk"]
        motion_before = _motion_pose_diagnostics(sdk, list(target))
        result = sdk.move_l(target, speed=self.speed)
        if result.get("ok", False):
            request_world_refresh(context, sdk, reason=f"post_{self.name}")
            motion_after = _motion_pose_diagnostics(sdk, list(target))
            return SkillResult.success(
                command=result["command"],
                motion_target_pose=list(target),
                motion_diagnostics_before=motion_before,
                motion_diagnostics_after=motion_after,
                grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
            )

        if not support_regrasp_active:
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="move_l failed",
                sdk_result=result,
                motion_target_pose=list(target),
                motion_diagnostics_before=motion_before,
            )

        status = dict(getattr(sdk, "get_status", lambda: {})() or {})
        current_pose = list(status.get("eef_pose") or [])
        if len(current_pose) < 7:
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="move_l failed",
                sdk_result=result,
                motion_target_pose=list(target),
                motion_diagnostics_before=motion_before,
            )

        delta_z = float(list(target)[2]) - float(current_pose[2])
        attempts: List[Dict[str, Any]] = [
            {
                "ok": False,
                "plan_name": "full_lift",
                "failed_step_index": 0,
                "sdk_result": dict(result),
                "steps": [
                    {
                        "step_index": 0,
                        "delta_xyz": [0.0, 0.0, delta_z],
                        "start_pose": [float(value) for value in current_pose[:7]],
                        "target_pose": [float(value) for value in list(target)[:7]],
                        "sdk_result": dict(result),
                    }
                ],
            }
        ]
        chosen_attempt: Dict[str, Any] | None = None
        for plan in self._support_lift_plans(delta_z):
            if str(plan.get("plan_name") or "") == "full_lift":
                continue
            attempt = self._run_support_lift_plan(
                sdk,
                plan_name=str(plan.get("plan_name") or "support_lift"),
                start_pose=[float(value) for value in current_pose[:7]],
                delta_z=float(delta_z),
                fractions=[float(value) for value in list(plan.get("fractions") or [])],
                speed=self.speed,
            )
            attempts.append(attempt)
            if attempt.get("ok", False):
                chosen_attempt = attempt
                break

        if chosen_attempt is None:
            return SkillResult.failure(
                FailureCode.SDK_ERROR,
                message="move_l failed",
                sdk_result=attempts[-1].get("sdk_result") if attempts else result,
                motion_target_pose=list(target),
                motion_diagnostics_before=motion_before,
                attempted_motion_plans=attempts,
            )

        request_world_refresh(context, sdk, reason=f"post_{self.name}")
        motion_after = _motion_pose_diagnostics(sdk, list(target))
        return SkillResult.success(
            command=dict((chosen_attempt.get("steps") or [{}])[-1].get("sdk_result") or {}).get("command"),
            motion_target_pose=list(target),
            motion_diagnostics_before=motion_before,
            motion_diagnostics_after=motion_after,
            motion_plan=str(chosen_attempt.get("plan_name") or ""),
            motion_plan_attempts=attempts,
            grasp_candidate_refresh=context.blackboard.get("last_grasp_candidate_refresh"),
        )


class PlaceApproach(MoveL):
    def __init__(self, target_pose: Optional[List[float]] = None):
        super().__init__(
            name="PlaceApproach",
            target_pose=target_pose,
            speed=0.4,
            blackboard_key="place_pose",
        )

    def run(self, context: SkillContext) -> SkillResult:
        sdk = context.adapters["sdk"]
        base_target = self.target_pose or context.blackboard.get(self.blackboard_key)
        if not base_target:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing pose target")
        module = resolve_place_module(context)
        return module.execute_place_approach(context, skill=self, sdk=sdk, target_pose=base_target)


class PlaceRelease(MoveL):
    def __init__(self, target_pose: Optional[List[float]] = None):
        super().__init__(
            name="PlaceRelease",
            target_pose=target_pose,
            speed=0.25,
            blackboard_key="place_release_pose",
        )

    def run(self, context: SkillContext) -> SkillResult:
        sdk = context.adapters["sdk"]
        base_target = self.target_pose or context.blackboard.get(self.blackboard_key)
        if not base_target:
            return SkillResult.failure(FailureCode.NO_IK, message="Missing pose target")
        module = resolve_place_module(context)
        return module.execute_place_release(context, skill=self, sdk=sdk, target_pose=base_target)


class ApproachActiveGrasp(MoveL):
    def __init__(self, target_pose: Optional[List[float]] = None):
        super().__init__(
            name="ApproachActiveGrasp",
            target_pose=target_pose,
            speed=0.2,
            blackboard_key="active_grasp_pose",
        )


class ResetArm(MoveJ):
    def __init__(self, target_joints: Optional[List[float]] = None):
        super().__init__(
            name="ResetArm",
            target_joints=target_joints,
            speed=0.5,
            blackboard_key="reset_joints",
        )
