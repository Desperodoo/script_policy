from script_runtime.core.skill_base import SkillContext
from script_runtime.skills.recovery.primitives import RetryWithNextCandidate


def test_retry_with_next_candidate_skips_planner_failures(blackboard):
    candidates = [
        {"variant_label": "current", "planner_status": "Failure", "pose": [0.1] * 7, "pregrasp_pose": [0.2] * 7},
        {"variant_label": "known_bad", "planner_status": "Failure", "pose": [0.3] * 7, "pregrasp_pose": [0.4] * 7},
        {"variant_label": "unknown_ok", "planner_status": "Unknown", "pose": [0.5] * 7, "pregrasp_pose": [0.6] * 7},
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})

    result = RetryWithNextCandidate().run(SkillContext(blackboard=blackboard, adapters={}, task_id="retry-skip"))

    assert result.status.value == "SUCCESS"
    assert result.payload["next_candidate"]["variant_label"] == "unknown_ok"
    assert len(result.payload["rejected_candidates"]) == 2
    assert blackboard.world_state.learned.grasp_candidates[0]["variant_label"] == "unknown_ok"
    assert blackboard.get("active_grasp_candidate")["variant_label"] == "unknown_ok"


def test_retry_with_next_candidate_fails_when_only_planner_failures_remain(blackboard):
    candidates = [
        {"variant_label": "current", "planner_status": "Failure", "pose": [0.1] * 7, "pregrasp_pose": [0.2] * 7},
        {"variant_label": "known_bad_1", "planner_status": "Failure", "pose": [0.3] * 7, "pregrasp_pose": [0.4] * 7},
        {"variant_label": "known_bad_2", "planner_status": "Fail", "pose": [0.5] * 7, "pregrasp_pose": [0.6] * 7},
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})

    result = RetryWithNextCandidate().run(SkillContext(blackboard=blackboard, adapters={}, task_id="retry-exhaust"))

    assert result.status.value == "FAILURE"
    assert result.message == "No planner-feasible next candidate available"


def test_retry_with_next_candidate_skips_semantically_incompatible_candidates(blackboard):
    candidates = [
        {"variant_label": "current", "planner_status": "Failure", "pose": [0.1] * 7, "pregrasp_pose": [0.2] * 7},
        {
            "variant_label": "wrong_side",
            "planner_status": "Success",
            "task_compatibility": "incompatible",
            "pose": [0.3] * 7,
            "pregrasp_pose": [0.4] * 7,
        },
        {
            "variant_label": "fallback_ok",
            "planner_status": "Unknown",
            "task_compatibility": "compatible",
            "pose": [0.5] * 7,
            "pregrasp_pose": [0.6] * 7,
        },
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})

    result = RetryWithNextCandidate().run(SkillContext(blackboard=blackboard, adapters={}, task_id="retry-semantic"))

    assert result.status.value == "SUCCESS"
    assert result.payload["next_candidate"]["variant_label"] == "fallback_ok"


def test_retry_with_next_candidate_rejects_current_active_even_if_not_first(blackboard):
    candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "pose": [0.1] * 7,
            "pregrasp_pose": [0.2] * 7,
        },
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "arm": "left",
            "planner_status": "Success",
            "pose": [0.3] * 7,
            "pregrasp_pose": [0.4] * 7,
        },
    ]
    blackboard.update_world(learned={"grasp_candidates": candidates})
    blackboard.set("active_grasp_candidate", candidates[1])

    result = RetryWithNextCandidate().run(SkillContext(blackboard=blackboard, adapters={}, task_id="retry-active"))

    assert result.status.value == "SUCCESS"
    assert result.payload["next_candidate"]["variant_label"] == "contact_0"
    assert result.payload["rejected_candidates"][0]["variant_label"] == "contact_1"
    assert blackboard.get("active_grasp_candidate")["variant_label"] == "contact_0"
