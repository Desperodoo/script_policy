"""Minimal staged-place probe task tree."""

from __future__ import annotations

from typing import Any

from script_runtime.core.task_base import ScriptTask
from script_runtime.executors.pytrees_executor import RecoveryNode, RetryNode, SequenceNode, SkillNode, TimeoutNode
from script_runtime.tasks.probes.common import (
    PROBE_EXECUTE_GRASP_TIMEOUT_S,
    PROBE_GO_PREGRASP_TIMEOUT_S,
    PROBE_PREPARE_GRIPPER_TIMEOUT_S,
)


class StagedPlaceProbeTask(ScriptTask):
    """Probe staged-place contracts with an explicit post-place release follow-up."""

    def __init__(self, goal: dict | None = None):
        super().__init__(
            name="staged_place_probe",
            goal=goal or {},
            metadata={"task_contract": "staged_place_probe"},
        )

    def build(self, registry: Any):
        def build_grasp_attempt(name_prefix: str) -> SequenceNode:
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

        grasp_attempt = build_grasp_attempt("probe")
        support_grasp_attempt = build_grasp_attempt("support")

        def build_grasp_recovery(name_prefix: str):
            return SequenceNode(
                f"{name_prefix}_grasp_recovery_sequence",
                [
                    SkillNode(f"{name_prefix}_safe_retreat", "SafeRetreat"),
                    SkillNode(f"{name_prefix}_retry_next_candidate", "RetryWithNextCandidate"),
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
                    RecoveryNode("grasp_recovery", grasp_attempt, lambda context, result: build_grasp_recovery("probe")),
                    max_attempts=5,
                ),
                SkillNode("place_approach", "PlaceApproach"),
                SkillNode("place_release", "PlaceRelease"),
                # Staged-place probes should make the release step explicit so we
                # can separate "object was positioned" from "follow-up release /
                # disengage was missing".
                SkillNode("post_place_open_gripper", "OpenGripper"),
                SkillNode("post_place_retreat", "Retreat"),
                SkillNode("check_contact", "CheckContact"),
                SkillNode("prepare_support_regrasp", "PrepareSupportRegrasp"),
                RetryNode(
                    "support_acquire_object",
                    SequenceNode(
                        "support_perception_sequence",
                        [
                            SkillNode("support_get_object_pose", "GetObjectPose"),
                            SkillNode("support_get_grasp_candidates", "GetGraspCandidates"),
                        ],
                    ),
                    max_attempts=2,
                ),
                RetryNode(
                    "support_grasp_retry_budget",
                    RecoveryNode(
                        "support_grasp_recovery",
                        support_grasp_attempt,
                        lambda context, result: build_grasp_recovery("support"),
                    ),
                    max_attempts=4,
                ),
                SkillNode("support_lift_pull", "SupportLiftPull"),
                SkillNode("support_check_task_success", "CheckTaskSuccess"),
            ],
        )
        return self.root
