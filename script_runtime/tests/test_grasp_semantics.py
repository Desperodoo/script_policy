from script_runtime.core import FailureCode, SkillContext, SkillStatus, TaskBlackboard, WorldState
from script_runtime.planning import annotate_grasp_candidates, sort_grasp_candidates_by_semantics
from script_runtime.skills.checks import CheckGrasp, PrepareSupportRegrasp, ReselectGraspAfterPregrasp, SupportLiftPull
from script_runtime.skills.perception import GetGraspCandidates


class _DummyPerception:
    task_name = "place_container_plate"

    def get_grasp_candidates(self, *_args, **_kwargs):
        return [
            {
                "variant_label": "contact_0",
                "contact_point_id": 0,
                "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            }
        ]


def test_annotate_grasp_candidates_applies_override_rule():
    blackboard = TaskBlackboard(WorldState())
    blackboard.set(
        "grasp_affordance_overrides",
        [
            {
                "label": "container_handle",
                "task_name": "place_container_plate",
                "contact_point_id": 0,
                "affordance_type": "handle",
                "functional_role": "carry",
                "semantic_priority": 2.0,
            }
        ],
    )

    candidates = annotate_grasp_candidates(
        "place_container_plate",
        [{"variant_label": "contact_0", "contact_point_id": 0}],
        blackboard=blackboard,
    )

    assert candidates[0]["affordance_type"] == "handle"
    assert candidates[0]["functional_role"] == "carry"
    assert candidates[0]["task_compatibility"] == "preferred"
    assert candidates[0]["semantic_source"] == "override"


def test_get_grasp_candidates_records_affordance_metadata(blackboard, adapters):
    blackboard.set(
        "grasp_affordance_overrides",
        [
            {
                "label": "container_handle",
                "task_name": "place_container_plate",
                "contact_point_id": 0,
                "affordance_type": "handle",
                "functional_role": "carry",
            }
        ],
    )
    adapters["perception"] = _DummyPerception()

    result = GetGraspCandidates().run(SkillContext(blackboard=blackboard, adapters=adapters, task_id="affordance"))

    assert result.status == SkillStatus.SUCCESS
    active = blackboard.get("active_grasp_candidate")
    assert active["affordance_type"] == "handle"
    assert active["task_compatibility"] == "preferred"
    assert blackboard.get("grasp_semantic_policy")["visual_review_required"] is True
    assert blackboard.get("grasp_attempt_candidate")["contact_point_id"] == 0
    assert blackboard.get("grasp_attempt_initial_candidate")["contact_point_id"] == 0
    assert blackboard.get("grasp_attempt_reselected") is False


def test_get_grasp_candidates_syncs_active_arm_to_selected_candidate(blackboard, adapters):
    class _ArmAwarePerception:
        task_name = "place_can_basket"

        def get_grasp_candidates(self, *_args, **_kwargs):
            return [
                {
                    "variant_label": "contact_1",
                    "contact_point_id": 1,
                    "arm": "left",
                    "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "planner_status": "Success",
                }
            ]

    adapters["perception"] = _ArmAwarePerception()
    adapters["sdk"].active_arm = "right"
    blackboard.set("active_arm", "right")
    blackboard.set("probe_support_regrasp_active", True)

    result = GetGraspCandidates().run(SkillContext(blackboard=blackboard, adapters=adapters, task_id="sync-arm"))

    assert result.status == SkillStatus.SUCCESS
    assert blackboard.get("active_arm") == "left"
    assert blackboard.get("support_arm") == "left"
    assert adapters["sdk"].active_arm == "left"


def test_get_grasp_candidates_aligns_sdk_active_arm_to_designated_support_arm(blackboard, adapters):
    class _SupportPerception:
        task_name = "place_can_basket"

        def __init__(self):
            self.seen_active_arm = ""

        def get_grasp_candidates(self, _observation=None, context=None):
            self.seen_active_arm = str(getattr(context.adapters.get("sdk"), "active_arm", "") or "")
            return [
                {
                    "variant_label": "contact_7",
                    "contact_point_id": 7,
                    "arm": "right",
                    "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "planner_status": "Success",
                }
            ]

    perception = _SupportPerception()
    adapters["perception"] = perception
    adapters["sdk"].active_arm = "left"
    blackboard.set("probe_support_regrasp_active", True)
    blackboard.set("support_arm", "right")
    blackboard.set("active_arm", "left")

    result = GetGraspCandidates().run(SkillContext(blackboard=blackboard, adapters=adapters, task_id="support-arm-align"))

    assert result.status == SkillStatus.SUCCESS
    assert perception.seen_active_arm == "right"
    assert adapters["sdk"].active_arm == "right"
    assert blackboard.get("active_arm") == "right"
    assert blackboard.get("support_arm") == "right"
    assert blackboard.get("active_grasp_candidate")["arm"] == "right"


def test_get_grasp_candidates_fails_when_designated_support_arm_has_no_candidates(blackboard, adapters):
    class _WrongArmPerception:
        task_name = "place_can_basket"

        def get_grasp_candidates(self, *_args, **_kwargs):
            return [
                {
                    "variant_label": "contact_1",
                    "contact_point_id": 1,
                    "arm": "left",
                    "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "planner_status": "Success",
                }
            ]

    adapters["perception"] = _WrongArmPerception()
    blackboard.set("probe_support_regrasp_active", True)
    blackboard.set("support_arm", "right")

    result = GetGraspCandidates().run(
        SkillContext(
            blackboard=blackboard,
            adapters=adapters,
            task_id="support-arm-filter",
            metadata={"current_node_name": "support_get_grasp_candidates"},
        )
    )

    assert result.status == SkillStatus.FAILURE
    assert result.failure_code == FailureCode.NO_GRASP_CANDIDATE
    assert result.payload["support_arm"] == "right"
    assert result.payload["available_candidate_arms"] == ["left"]
    assert "designated support arm" in result.message.lower()


def test_check_grasp_requires_semantic_match_when_enabled(blackboard, adapters):
    blackboard.update_world(scene={"grasped": True})
    blackboard.set("active_grasp_candidate", {"variant_label": "contact_3", "contact_point_id": 3})
    blackboard.set("semantic_grasp_required", True)

    result = CheckGrasp().run(SkillContext(blackboard=blackboard, adapters=adapters, task_id="semantic-strict"))

    assert result.status == SkillStatus.FAILURE
    assert result.failure_code == FailureCode.GRASP_FAIL
    assert "semantic" in result.message.lower()


def test_check_grasp_accepts_task_compatible_affordance(blackboard, adapters):
    blackboard.update_world(scene={"grasped": True})
    blackboard.set(
        "active_grasp_candidate",
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "affordance_type": "handle",
            "functional_role": "carry",
        },
    )
    blackboard.set("semantic_grasp_required", True)
    blackboard.update_world(execution={"task_goal": {"task_name": "place_container_plate"}})

    result = CheckGrasp().run(SkillContext(blackboard=blackboard, adapters=adapters, task_id="semantic-ok"))

    assert result.status == SkillStatus.SUCCESS
    assert result.payload["grasp_semantics_ok"] is True
    assert result.payload["grasp_semantic_report"]["task_compatibility"] == "preferred"


def test_sort_grasp_candidates_by_semantics_prefers_preferred_then_compatible():
    candidates = [
        {"variant_label": "bad", "task_compatibility": "incompatible", "score": 10.0},
        {"variant_label": "ok", "task_compatibility": "compatible", "score": 0.5},
        {"variant_label": "best", "task_compatibility": "preferred", "score": 0.1},
    ]

    ranked = sort_grasp_candidates_by_semantics(candidates)

    assert [row["variant_label"] for row in ranked] == ["best", "ok", "bad"]


def test_sort_grasp_candidates_by_semantics_uses_semantic_priority_before_score():
    candidates = [
        {"variant_label": "fallback", "task_compatibility": "preferred", "score": 0.9, "semantic_priority": 0.0},
        {"variant_label": "fm", "task_compatibility": "preferred", "score": 0.1, "semantic_priority": 1.0},
    ]

    ranked = sort_grasp_candidates_by_semantics(candidates)

    assert [row["variant_label"] for row in ranked] == ["fm", "fallback"]


def test_reselect_grasp_after_pregrasp_promotes_newly_feasible_candidate(blackboard, adapters):
    candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 520,
            "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "task_compatibility": "compatible",
            "score": 1.5,
        },
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 580,
            "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "task_compatibility": "compatible",
            "score": 1.4,
        },
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})
    blackboard.set("grasp_candidates", candidates)
    blackboard.set("active_grasp_candidate", candidates[0])
    blackboard.set(
        "last_grasp_candidate_refresh",
        {
            "refresh_reason": "post_GoPregrasp",
            "improved_candidates": [
                {
                    "previous_status": "Failure",
                    "current_status": "Success",
                    "current": {
                        "contact_point_id": 1,
                        "arm": "left",
                        "rank": 1,
                    },
                }
            ],
        },
    )

    result = ReselectGraspAfterPregrasp().run(
        SkillContext(blackboard=blackboard, adapters=adapters, task_id="post-pregrasp-reselect")
    )

    assert result.status == SkillStatus.SUCCESS
    assert result.payload["reselected"] is True


def test_support_lift_pull_uses_fallback_motion_plan_when_combined_move_fails(blackboard):
    class _SDK:
        def __init__(self):
            self.pose = [0.10, -0.20, 0.90, 0.0, 0.0, 0.0, 1.0]
            self.commands = []

        def get_status(self):
            return {"eef_pose": list(self.pose)}

        def move_l(self, pose, speed=1.0):
            command = {"type": "move_l", "pose": list(pose), "speed": speed, "arm": "right"}
            self.commands.append(command)
            target_xyz = [round(float(value), 4) for value in list(pose)[:3]]
            if target_xyz == [0.12, -0.2, 0.95]:
                return {"ok": False, "action": "move_l", "command": command}
            self.pose = list(pose)
            return {"ok": True, "action": "move_l", "command": command}

        def refresh_world(self, _blackboard):
            return {"ok": True}

    sdk = _SDK()
    blackboard.set("servo_delta", [0.02, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0])

    result = SupportLiftPull().run(SkillContext(blackboard=blackboard, adapters={"sdk": sdk}, task_id="support-lift"))

    assert result.status == SkillStatus.SUCCESS
    assert result.payload["motion_plan"] == "lift_only"
    assert len(result.payload["motion_plan_attempts"]) >= 2
    assert result.payload["motion_plan_attempts"][0]["plan_name"] == "combined_full"
    assert result.payload["motion_plan_attempts"][0]["ok"] is False
    assert result.payload["motion_plan_attempts"][1]["plan_name"] == "lift_only"
    assert result.payload["motion_plan_attempts"][1]["ok"] is True
    assert len(sdk.commands) == 2
    assert [round(float(value), 4) for value in sdk.commands[-1]["pose"][:3]] == [0.1, -0.2, 0.95]


def test_support_lift_pull_failure_reports_support_completion_diagnostics(blackboard):
    class _SDK:
        def __init__(self):
            self.pose = [0.10, -0.20, 0.90, 0.0, 0.0, 0.0, 1.0]

        def get_status(self):
            return {"eef_pose": list(self.pose)}

        def get_trace_snapshot(self, label=""):
            return {
                "label": label,
                "object_pose": [0.01, 0.02, 0.83, 0.0, 0.0, 0.0, 1.0],
                "support_pose": [0.00, 0.00, 0.80, 0.0, 0.0, 0.0, 1.0],
                "object_to_support_pose_delta": {"dx": 0.01, "dy": 0.02, "dz": 0.03, "xy_norm": 0.0224},
            }

        def move_l(self, pose, speed=1.0):
            command = {"type": "move_l", "pose": list(pose), "speed": speed, "arm": "right"}
            return {"ok": False, "action": "move_l", "command": command}

    blackboard.set("servo_delta", [0.02, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0])

    result = SupportLiftPull().run(SkillContext(blackboard=blackboard, adapters={"sdk": _SDK()}, task_id="support-lift-fail"))

    assert result.status == SkillStatus.FAILURE
    diagnostics = result.payload["support_completion_diagnostics"]
    assert diagnostics["support_completion_subtype"] == "support_follow_up_motion_unreachable"
    assert diagnostics["support_motion_all_failed_at_first_step"] is True
    assert diagnostics["support_motion_plan_names"][0] == "combined_full"
    assert diagnostics["support_motion_requested_delta_xyz"] == [0.02, 0.0, 0.05]
    assert diagnostics["support_motion_start_pose"][:3] == [0.1, -0.2, 0.9]
    assert [round(float(value), 4) for value in diagnostics["support_motion_failed_target_pose"][:3]] == [0.12, -0.2, 0.95]


def test_prepare_support_regrasp_records_support_context_on_blackboard(blackboard):
    class _SDK:
        active_arm = "right"
        target_attr = "basket"
        object_attr = "can"
        pregrasp_distance = 0.1

        def get_status(self):
            return {"active_arm": "right"}

        def move_j(self, joints, speed=1.0):
            return {"ok": True, "action": "move_j", "command": {"type": "move_j", "joints": list(joints), "speed": speed}}

        def refresh_world(self, _blackboard):
            return {"ok": True}

    sdk = _SDK()
    blackboard.update_world(execution={"task_goal": {"task_name": "place_can_basket", "target_object": "can", "target_surface": "basket"}})

    result = PrepareSupportRegrasp().run(
        SkillContext(blackboard=blackboard, adapters={"sdk": sdk}, task_id="support-prepare")
    )

    assert result.status == SkillStatus.SUCCESS
    assert blackboard.get("support_arm") == "left"
    assert blackboard.get("support_target_frame") == "robotwin::basket"
    assert blackboard.get("support_pregrasp_pose_source") == "pending_world_refresh"
    assert blackboard.get("support_regrasp_substage") == "support_pregrasp_generation"
    assert blackboard.get("probe_support_regrasp_active") is True


def test_reselect_grasp_after_pregrasp_noops_without_flip(blackboard, adapters):
    candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
        }
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})
    blackboard.set("grasp_candidates", candidates)
    blackboard.set("active_grasp_candidate", candidates[0])
    blackboard.set("last_grasp_candidate_refresh", {"refresh_reason": "post_GoPregrasp", "improved_candidates": []})

    result = ReselectGraspAfterPregrasp().run(
        SkillContext(blackboard=blackboard, adapters=adapters, task_id="post-pregrasp-noop")
    )

    assert result.status == SkillStatus.SUCCESS
    assert result.payload["reselected"] is False
    assert blackboard.get("active_grasp_candidate")["contact_point_id"] == 0


def test_reselect_grasp_after_pregrasp_skips_incompatible_improved_candidate(blackboard, adapters):
    candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 520,
            "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "task_compatibility": "preferred",
            "score": 1.6,
        },
        {
            "variant_label": "contact_5",
            "contact_point_id": 5,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 430,
            "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "task_compatibility": "incompatible",
            "score": 1.8,
        },
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})
    blackboard.set("grasp_candidates", candidates)
    blackboard.set("active_grasp_candidate", candidates[0])
    blackboard.set(
        "last_grasp_candidate_refresh",
        {
            "refresh_reason": "post_GoPregrasp",
            "improved_candidates": [
                {
                    "previous_status": "Failure",
                    "current_status": "Success",
                    "current": {
                        "contact_point_id": 5,
                        "arm": "left",
                        "rank": 1,
                    },
                }
            ],
        },
    )

    result = ReselectGraspAfterPregrasp().run(
        SkillContext(blackboard=blackboard, adapters=adapters, task_id="post-pregrasp-skip-incompatible")
    )

    assert result.status == SkillStatus.SUCCESS
    assert result.payload["reselected"] is False
    assert result.payload["reason"] == "no_improved_candidate_to_promote"
    assert blackboard.get("active_grasp_candidate")["contact_point_id"] == 0


def test_reselect_grasp_after_pregrasp_avoids_recent_failed_active_candidate(blackboard, adapters):
    candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 450,
            "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "task_compatibility": "compatible",
            "score": 1.6,
        },
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 430,
            "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "task_compatibility": "compatible",
            "score": 1.55,
        },
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})
    blackboard.set("grasp_candidates", candidates)
    blackboard.set("active_grasp_candidate", candidates[0])
    blackboard.set("last_failed_grasp_candidate", dict(candidates[0]))
    blackboard.set("last_grasp_candidate_refresh", {"refresh_reason": "post_GoPregrasp", "improved_candidates": []})

    result = ReselectGraspAfterPregrasp().run(
        SkillContext(blackboard=blackboard, adapters=adapters, task_id="post-pregrasp-failed-active")
    )

    assert result.status == SkillStatus.SUCCESS
    assert result.payload["reselected"] is True
    assert result.payload["reselection_reason"] == "avoid_recent_failed_active_candidate"
    assert blackboard.get("active_grasp_candidate")["contact_point_id"] == 1
    assert blackboard.get("grasp_attempt_candidate")["contact_point_id"] == 1
    assert blackboard.get("grasp_attempt_reselected") is True
    assert blackboard.get("grasp_attempt_reselection_skill") == "ReselectGraspAfterPregrasp"
