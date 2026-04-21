from script_runtime.core import FailureCode, SkillContext, SkillStatus, TaskBlackboard, WorldState
from script_runtime.planning import annotate_grasp_candidates, sort_grasp_candidates_by_semantics
from script_runtime.skills.checks import CheckGrasp, ReselectGraspAfterPregrasp
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
    active = blackboard.get("active_grasp_candidate")
    assert active["contact_point_id"] == 1
    reordered = blackboard.get("grasp_candidates")
    assert reordered[0]["contact_point_id"] == 1


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
