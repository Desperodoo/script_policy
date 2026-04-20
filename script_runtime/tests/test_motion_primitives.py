from script_runtime.core.skill_base import SkillContext
from script_runtime.core.result_types import SkillResult
from script_runtime.place import ClosedLoopPlaceModule
from script_runtime.planning import CandidateVariantSpec
from script_runtime.skills.motion.primitives import PlaceApproach, PlaceRelease


class _PlannerAwareSDK:
    def __init__(self):
        self.move_calls = []
        self.status = {"eef_pose": [0.0] * 7, "fault": False, "mode": "mock"}

    def _active_arm(self):
        return "left"

    def evaluate_pose_candidates(self, poses, kind="place_approach"):
        rows = []
        for pose in poses:
            if abs(float(pose[0]) - 0.4) < 1e-6:
                rows.append({"status": "Success", "waypoint_count": 18})
            else:
                rows.append({"status": "Failure", "waypoint_count": 0})
        return rows

    def move_l(self, pose, speed=1.0):
        self.move_calls.append({"pose": list(pose), "speed": speed})
        return {"ok": abs(float(pose[0]) - 0.4) < 1e-6, "command": {"type": "move_l", "pose": list(pose), "speed": speed}}

    def refresh_world(self, blackboard):
        return self.status

    def get_trace_snapshot(self, label=""):
        return {"label": label, "object_to_target_center_delta": {"xy_norm": 0.12}}

    def score_pose_candidate(self, pose, kind=""):
        return {
            "score_adjust": 1.5 if abs(float(pose[0]) - 0.4) < 1e-6 else -0.2,
            "predicted_object_to_target_center_delta": {"xy_norm": 0.02 if abs(float(pose[0]) - 0.4) < 1e-6 else 0.2},
        }


def test_place_approach_prefers_planner_ranked_candidate(monkeypatch, blackboard):
    sdk = _PlannerAwareSDK()
    blackboard.set("place_pose", [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
    blackboard.set("place_release_pose", [0.1, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0])

    def _fake_candidates(**kwargs):
        return [
            CandidateVariantSpec("primary", [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
            CandidateVariantSpec("left_arm_clear", [0.4, 0.2, 0.35, 0.0, 0.0, 0.0, 1.0]),
            CandidateVariantSpec("high_clearance", [0.2, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0]),
        ]

    monkeypatch.setattr(
        "script_runtime.place.heuristic.build_blended_release_candidates",
        _fake_candidates,
    )

    result = PlaceApproach().run(
        SkillContext(
            blackboard=blackboard,
            adapters={"sdk": sdk},
            task_id="place-approach-rank",
        )
    )

    assert result.status.value == "SUCCESS"
    assert result.payload["fallback_used"] == "left_arm_clear"
    assert sdk.move_calls[0]["pose"][0] == 0.4
    assert result.payload["planner_status"] == "Success"
    assert result.payload["candidate_ranking"][0]["label"] == "left_arm_clear"
    assert result.payload["candidate_ranking"][0]["score_metrics"]["predicted_object_to_target_center_delta"]["xy_norm"] == 0.02


def test_place_release_records_planner_ranked_attempts(monkeypatch, blackboard):
    sdk = _PlannerAwareSDK()
    blackboard.set("place_release_pose", [0.1, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0])
    blackboard.set("active_place_approach_pose", [0.1, 0.2, 0.35, 0.0, 0.0, 0.0, 1.0])

    def _fake_candidates(**kwargs):
        return [
            CandidateVariantSpec("primary", [0.1, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0]),
            CandidateVariantSpec("release_side_backoff", [0.4, 0.2, 0.28, 0.0, 0.0, 0.0, 1.0]),
        ]

    monkeypatch.setattr(
        "script_runtime.place.heuristic.build_arm_aware_release_candidates",
        _fake_candidates,
    )

    result = PlaceRelease().run(
        SkillContext(
            blackboard=blackboard,
            adapters={"sdk": sdk},
            task_id="place-release-rank",
        )
    )

    assert result.status.value == "SUCCESS"
    assert sdk.move_calls[0]["pose"][0] == 0.4
    assert result.payload["planner_status"] == "Success"
    assert result.payload["candidate_ranking"][0]["label"] == "release_side_backoff"
    assert result.payload["release_state_before"]["label"] == "place_release_before"
    assert result.payload["release_state_after"]["label"] == "place_release_after"
    assert result.payload["candidate_ranking"][0]["score_metrics"]["predicted_object_to_target_center_delta"]["xy_norm"] == 0.02
    assert result.payload["place_module"] == "heuristic_place_module"


class _CustomPlaceModule:
    name = "custom_place_module"

    def execute_place_approach(self, context, *, skill, sdk, target_pose):
        context.blackboard.set("custom_place_approach_target", list(target_pose))
        return SkillResult.success(place_module=self.name, custom=True)

    def execute_place_release(self, context, *, skill, sdk, target_pose):
        context.blackboard.set("custom_place_release_target", list(target_pose))
        return SkillResult.success(place_module=self.name, custom=True)


def test_place_skills_can_use_injected_place_module(blackboard):
    sdk = _PlannerAwareSDK()
    blackboard.set("place_pose", [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
    blackboard.set("place_release_pose", [0.1, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0])
    module = _CustomPlaceModule()
    context = SkillContext(
        blackboard=blackboard,
        adapters={"sdk": sdk, "place_module": module},
        task_id="custom-place-module",
    )

    approach = PlaceApproach().run(context)
    release = PlaceRelease().run(context)

    assert approach.status.value == "SUCCESS"
    assert release.status.value == "SUCCESS"
    assert approach.payload["place_module"] == "custom_place_module"
    assert release.payload["place_module"] == "custom_place_module"
    assert blackboard.get("custom_place_approach_target")[2] == 0.3
    assert blackboard.get("custom_place_release_target")[2] == 0.25


class _ClosedLoopSDK(_PlannerAwareSDK):
    def __init__(self):
        super().__init__()
        self.status["eef_pose"] = [0.4, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0]
        self.delta = {"dx": 0.10, "dy": 0.0, "dz": 0.0}

    def move_l(self, pose, speed=1.0):
        prev = list(self.status["eef_pose"])
        self.move_calls.append({"pose": list(pose), "speed": speed})
        self.status["eef_pose"] = list(pose)
        cmd = {"ok": True, "command": {"type": "move_l", "pose": list(pose), "speed": speed}}
        self.delta["dx"] += float(pose[0]) - float(prev[0])
        self.delta["dy"] += float(pose[1]) - float(prev[1])
        self.delta["dz"] += float(pose[2]) - float(prev[2])
        return cmd

    def get_trace_snapshot(self, label=""):
        dx = float(self.delta["dx"])
        dy = float(self.delta["dy"])
        dz = float(self.delta["dz"])
        return {
            "label": label,
            "eef_pose": list(self.status["eef_pose"]),
            "object_to_target_center_delta": {
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "xy_norm": abs(dx),
            },
        }


def test_closed_loop_place_module_refines_release_after_baseline(monkeypatch, blackboard):
    sdk = _ClosedLoopSDK()
    blackboard.set("place_release_pose", [0.4, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0])
    blackboard.set("active_place_approach_pose", [0.4, 0.2, 0.35, 0.0, 0.0, 0.0, 1.0])

    def _fake_candidates(**kwargs):
        return [
            CandidateVariantSpec("primary", [0.4, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0]),
            CandidateVariantSpec("fallback", [0.1, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0]),
        ]

    monkeypatch.setattr(
        "script_runtime.place.heuristic.build_arm_aware_release_candidates",
        _fake_candidates,
    )

    result = PlaceRelease().run(
        SkillContext(
            blackboard=blackboard,
            adapters={
                "sdk": sdk,
                "place_module": ClosedLoopPlaceModule(
                    max_alignment_steps=3,
                    xy_gain=0.5,
                    xy_step_limit=0.03,
                    target_xy_tolerance=0.03,
                    min_xy_improvement=0.0,
                ),
            },
            task_id="closed-loop-place-release",
        )
    )

    assert result.status.value == "SUCCESS"
    assert result.payload["place_module"] == "closed_loop_place_module"
    assert result.payload["baseline_place_module"] == "heuristic_place_module"
    assert result.payload["alignment_steps"]
    assert result.payload["correction_model_seed"]
    assert result.payload["correction_model_final"]
    assert result.payload["alignment_steps"][0]["response_matrix_before"]
    assert result.payload["alignment_steps"][0]["response_matrix_after"]
    assert result.payload["final_alignment_delta"]["xy_norm"] <= 0.03
    assert len(sdk.move_calls) >= 3


class _DivergentClosedLoopSDK(_ClosedLoopSDK):
    def move_l(self, pose, speed=1.0):
        prev = list(self.status["eef_pose"])
        self.move_calls.append({"pose": list(pose), "speed": speed})
        self.status["eef_pose"] = list(pose)
        cmd = {"ok": True, "command": {"type": "move_l", "pose": list(pose), "speed": speed}}
        # Simulate a wrong local transport model: moving the hand causes the
        # object center to drift farther from the target.
        self.delta["dx"] -= float(pose[0]) - float(prev[0])
        self.delta["dy"] -= float(pose[1]) - float(prev[1])
        self.delta["dz"] -= float(pose[2]) - float(prev[2])
        return cmd


def test_closed_loop_place_module_reverts_when_alignment_worsens(monkeypatch, blackboard):
    sdk = _DivergentClosedLoopSDK()
    blackboard.set("place_release_pose", [0.4, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0])
    blackboard.set("active_place_approach_pose", [0.4, 0.2, 0.35, 0.0, 0.0, 0.0, 1.0])

    def _fake_candidates(**kwargs):
        return [CandidateVariantSpec("primary", [0.4, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0])]

    monkeypatch.setattr(
        "script_runtime.place.heuristic.build_arm_aware_release_candidates",
        _fake_candidates,
    )

    result = PlaceRelease().run(
        SkillContext(
            blackboard=blackboard,
            adapters={"sdk": sdk, "place_module": ClosedLoopPlaceModule(max_alignment_steps=2, xy_gain=0.5)},
            task_id="closed-loop-revert",
        )
    )

    assert result.status.value == "SUCCESS"
    assert result.payload["closed_loop_status"] == "alignment_worsened_reverted"
    assert result.payload["alignment_steps"][0]["revert_ok"] is True
    assert result.payload["release_state_after"]["label"].endswith("_revert")
