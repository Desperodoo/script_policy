"""Minimal dual-arm handover probe task tree."""

from __future__ import annotations

from typing import Any

from script_runtime.core.task_base import ScriptTask
from script_runtime.executors.pytrees_executor import RecoveryNode, RetryNode, SequenceNode, SkillNode, TimeoutNode
from script_runtime.tasks.probes.common import (
    PROBE_EXECUTE_GRASP_TIMEOUT_S,
    PROBE_GO_PREGRASP_TIMEOUT_S,
    PROBE_PREPARE_GRIPPER_TIMEOUT_S,
)


def _grasp_sequence(name_prefix: str) -> SequenceNode:
    return SequenceNode(
        f"{name_prefix}_grasp_attempt",
        [
            TimeoutNode(
                f"{name_prefix}_prepare_gripper_timeout",
                SkillNode(f"{name_prefix}_prepare_gripper_for_grasp", "PrepareGripperForGrasp"),
                timeout_s=PROBE_PREPARE_GRIPPER_TIMEOUT_S,
            ),
            TimeoutNode(
                f"{name_prefix}_go_pregrasp_timeout",
                SkillNode(f"{name_prefix}_go_pregrasp", "GoPregrasp"),
                timeout_s=PROBE_GO_PREGRASP_TIMEOUT_S,
            ),
            SkillNode(f"{name_prefix}_reselect_grasp_after_pregrasp", "ReselectGraspAfterPregrasp"),
            TimeoutNode(
                f"{name_prefix}_grasp_phase_timeout",
                SkillNode(f"{name_prefix}_execute_grasp_phase", "ExecuteGraspPhase"),
                timeout_s=PROBE_EXECUTE_GRASP_TIMEOUT_S,
            ),
            SkillNode(f"{name_prefix}_check_grasp", "CheckGrasp"),
            SkillNode(f"{name_prefix}_lift", "Lift"),
            SkillNode(f"{name_prefix}_check_grasp_after_lift", "CheckGrasp"),
        ],
    )


class HandoverProbeTask(ScriptTask):
    """Probe dual-arm handover ownership transfer without bespoke skills."""

    def __init__(self, goal: dict | None = None):
        super().__init__(
            name="handover_probe",
            goal=goal or {},
            metadata={"task_contract": "handover_probe"},
        )

    def build(self, registry: Any):
        source_grasp = _grasp_sequence("source")
        receiver_grasp = _grasp_sequence("receiver")

        def build_grasp_recovery(_context, _result):
            return SequenceNode(
                "handover_probe_grasp_recovery_sequence",
                [
                    SkillNode("safe_retreat", "SafeRetreat"),
                    SkillNode("retry_next_candidate", "RetryWithNextCandidate"),
                ],
            )

        self.root = SequenceNode(
            "handover_probe_root",
            [
                SkillNode("check_scene_ready", "CheckSceneReady"),
                RetryNode(
                    "source_acquire_object",
                    SequenceNode(
                        "source_perception_sequence",
                        [
                            SkillNode("source_get_object_pose", "GetObjectPose"),
                            SkillNode("source_get_grasp_candidates", "GetGraspCandidates"),
                        ],
                    ),
                    max_attempts=2,
                ),
                RetryNode(
                    "source_grasp_retry_budget",
                    RecoveryNode("source_grasp_recovery", source_grasp, build_grasp_recovery),
                    max_attempts=4,
                ),
                SkillNode("switch_active_arm_to_receiver", "SwitchActiveArm"),
                RetryNode(
                    "receiver_acquire_object",
                    SequenceNode(
                        "receiver_perception_sequence",
                        [
                            SkillNode("receiver_get_object_pose", "GetObjectPose"),
                            SkillNode("receiver_get_grasp_candidates", "GetGraspCandidates"),
                        ],
                    ),
                    max_attempts=2,
                ),
                RetryNode(
                    "receiver_grasp_retry_budget",
                    RecoveryNode("receiver_grasp_recovery", receiver_grasp, build_grasp_recovery),
                    max_attempts=3,
                ),
                SkillNode("switch_active_arm_to_source", "SwitchActiveArm"),
                SkillNode("open_source_gripper", "OpenGripper"),
                SkillNode("switch_active_arm_to_receiver_verify", "SwitchActiveArm"),
                SkillNode("check_receiver_grasp", "CheckGrasp"),
                SkillNode("place_approach", "PlaceApproach"),
                SkillNode("place_release", "PlaceRelease"),
                SkillNode("open_receiver_gripper", "OpenGripper"),
                SkillNode("check_task_success", "CheckTaskSuccess"),
            ],
        )
        return self.root
