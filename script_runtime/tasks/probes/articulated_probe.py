"""Minimal articulated-object probe task tree."""

from __future__ import annotations

from typing import Any

from script_runtime.core.task_base import ScriptTask
from script_runtime.executors.pytrees_executor import RecoveryNode, RetryNode, SequenceNode, SkillNode, TimeoutNode
from script_runtime.tasks.probes.common import (
    PROBE_EXECUTE_GRASP_TIMEOUT_S,
    PROBE_GO_PREGRASP_TIMEOUT_S,
    PROBE_PREPARE_GRIPPER_TIMEOUT_S,
)


class ArticulatedProbeTask(ScriptTask):
    """Probe articulated-object manipulation with minimal runtime-specific logic."""

    def __init__(self, goal: dict | None = None):
        super().__init__(
            name="articulated_probe",
            goal=goal or {},
            metadata={"task_contract": "articulated_probe"},
        )

    def build(self, registry: Any):
        grasp_attempt = SequenceNode(
            "articulated_grasp_attempt",
            [
                TimeoutNode(
                    "articulated_prepare_gripper_timeout",
                    SkillNode("articulated_prepare_gripper_for_grasp", "PrepareGripperForGrasp"),
                    timeout_s=PROBE_PREPARE_GRIPPER_TIMEOUT_S,
                ),
                TimeoutNode(
                    "articulated_go_pregrasp_timeout",
                    SkillNode("articulated_go_pregrasp", "GoPregrasp"),
                    timeout_s=PROBE_GO_PREGRASP_TIMEOUT_S,
                ),
                SkillNode("articulated_reselect_grasp_after_pregrasp", "ReselectGraspAfterPregrasp"),
                TimeoutNode(
                    "articulated_grasp_phase_timeout",
                    SkillNode("articulated_execute_grasp_phase", "ExecuteGraspPhase"),
                    timeout_s=PROBE_EXECUTE_GRASP_TIMEOUT_S,
                ),
                SkillNode("articulated_check_grasp", "CheckGrasp"),
            ],
        )

        def build_grasp_recovery(_context, _result):
            return SequenceNode(
                "articulated_grasp_recovery_sequence",
                [
                    SkillNode("articulated_safe_retreat", "SafeRetreat"),
                    SkillNode("articulated_retry_next_candidate", "RetryWithNextCandidate"),
                ],
            )

        self.root = SequenceNode(
            "articulated_probe_root",
            [
                SkillNode("articulated_check_scene_ready", "CheckSceneReady"),
                RetryNode(
                    "articulated_acquire_target",
                    SequenceNode(
                        "articulated_perception_sequence",
                        [
                            SkillNode("articulated_get_object_pose", "GetObjectPose"),
                            SkillNode("articulated_get_grasp_candidates", "GetGraspCandidates"),
                        ],
                    ),
                    max_attempts=2,
                ),
                RetryNode(
                    "articulated_grasp_retry_budget",
                    RecoveryNode("articulated_grasp_recovery", grasp_attempt, build_grasp_recovery),
                    max_attempts=4,
                ),
                SkillNode("articulated_handle_alignment", "PlaceApproach"),
                SkillNode("articulated_joint_drive", "PlaceRelease"),
                SkillNode("articulated_contact_check", "CheckContact"),
                SkillNode("articulated_open_gripper", "OpenGripper"),
                SkillNode("articulated_check_task_success", "CheckTaskSuccess"),
            ],
        )
        return self.root
