"""Minimal staged-place probe task tree."""

from __future__ import annotations

from typing import Any

from script_runtime.core.task_base import ScriptTask
from script_runtime.executors.pytrees_executor import RecoveryNode, RetryNode, SequenceNode, SkillNode, TimeoutNode


class StagedPlaceProbeTask(ScriptTask):
    """Probe staged-place contracts without assuming final open-gripper release."""

    def __init__(self, goal: dict | None = None):
        super().__init__(
            name="staged_place_probe",
            goal=goal or {},
            metadata={"task_contract": "staged_place_probe"},
        )

    def build(self, registry: Any):
        grasp_attempt = SequenceNode(
            "probe_grasp_attempt",
            [
                TimeoutNode(
                    "prepare_gripper_timeout",
                    SkillNode("prepare_gripper_for_grasp", "PrepareGripperForGrasp"),
                    timeout_s=10.0,
                ),
                TimeoutNode("go_pregrasp_timeout", SkillNode("go_pregrasp", "GoPregrasp"), timeout_s=8.0),
                SkillNode("reselect_grasp_after_pregrasp", "ReselectGraspAfterPregrasp"),
                TimeoutNode("grasp_phase_timeout", SkillNode("execute_grasp_phase", "ExecuteGraspPhase"), timeout_s=8.0),
                SkillNode("check_grasp", "CheckGrasp"),
                SkillNode("lift", "Lift"),
                SkillNode("check_grasp_after_lift", "CheckGrasp"),
            ],
        )

        def build_grasp_recovery(_context, _result):
            return SequenceNode(
                "probe_grasp_recovery_sequence",
                [
                    SkillNode("safe_retreat", "SafeRetreat"),
                    SkillNode("retry_next_candidate", "RetryWithNextCandidate"),
                ],
            )

        self.root = SequenceNode(
            "staged_place_probe_root",
            [
                SkillNode("check_scene_ready", "CheckSceneReady"),
                RetryNode(
                    "acquire_object",
                    SequenceNode(
                        "probe_perception_sequence",
                        [
                            SkillNode("get_object_pose", "GetObjectPose"),
                            SkillNode("get_grasp_candidates", "GetGraspCandidates"),
                        ],
                    ),
                    max_attempts=2,
                ),
                RetryNode(
                    "grasp_retry_budget",
                    RecoveryNode("grasp_recovery", grasp_attempt, build_grasp_recovery),
                    max_attempts=5,
                ),
                SkillNode("place_approach", "PlaceApproach"),
                SkillNode("place_release", "PlaceRelease"),
                SkillNode("check_contact", "CheckContact"),
                SkillNode("check_task_success", "CheckTaskSuccess"),
            ],
        )
        return self.root
