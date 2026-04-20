from script_runtime.planning import normalize_planner_statuses
from script_runtime.adapters.robotwin_bridge import RoboTwinBridge
from script_runtime.core.blackboard import TaskBlackboard, WorldState
from types import SimpleNamespace


class _FakePose:
    def __init__(self, values):
        self.p = values[:3]
        self.q = values[3:7]


class _FunctionalActor:
    def __init__(self, center_pose, fp_pose):
        self._center_pose = _FakePose(center_pose)
        self._fp_pose = _FakePose(fp_pose)

    def get_pose(self):
        return self._center_pose

    def get_functional_point(self, *_args, **_kwargs):
        return self._fp_pose


class _NamedActor(_FunctionalActor):
    def __init__(self, name, center_pose, fp_pose):
        super().__init__(center_pose, fp_pose)
        self._name = name

    def get_name(self):
        return self._name


def test_normalize_planner_statuses_scalar_string_broadcasts():
    rows = normalize_planner_statuses("Success", 3)
    assert rows == ["Success", "Success", "Success"]


def test_normalize_planner_statuses_pads_short_arrays():
    rows = normalize_planner_statuses(["Failure"], 3)
    assert rows == ["Failure", "Failure", "Failure"]


def test_normalize_planner_statuses_broadcasts_uniform_short_failure_batches():
    rows = normalize_planner_statuses(["Failure"] * 10, 55)
    assert len(rows) == 55
    assert set(rows) == {"Failure"}


def test_normalize_planner_statuses_handles_none():
    rows = normalize_planner_statuses(None, 2)
    assert rows == ["Unknown", "Unknown"]


def test_resolve_active_grasp_candidate_preserves_selected_contact():
    bridge = RoboTwinBridge()
    blackboard = TaskBlackboard(WorldState())
    blackboard.set(
        "active_grasp_candidate",
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "pose": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
        },
    )
    candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0],
        },
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "pose": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.2, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0],
        },
    ]

    active = bridge._resolve_active_grasp_candidate(blackboard, candidates)

    assert active["variant_label"] == "contact_1"
    assert active["contact_point_id"] == 1


def test_center_aligned_functional_target_pose_offsets_target_for_center_success():
    bridge = RoboTwinBridge(task_name="place_container_plate", object_functional_point_id=0, target_functional_point_id=0)
    bridge.env = SimpleNamespace()
    bridge.object_attr = "container"
    bridge.target_attr = "plate"
    bridge.env.container = _FunctionalActor(
        center_pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    )
    bridge.env.plate = _FunctionalActor(
        center_pose=[10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    )

    target_pose = bridge._center_aligned_functional_target_pose()

    assert target_pose is not None
    assert target_pose[:3] == [11.0, 0.0, 0.0]
    assert bridge._place_functional_point_id() == 0


def test_center_aligned_target_uses_initial_object_geometry_when_available():
    bridge = RoboTwinBridge(task_name="place_container_plate", object_functional_point_id=0, target_functional_point_id=0)
    bridge.env = SimpleNamespace()
    bridge.object_attr = "container"
    bridge.target_attr = "plate"
    bridge.env.container = _FunctionalActor(
        center_pose=[5.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[7.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    )
    bridge.env.plate = _FunctionalActor(
        center_pose=[10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    )
    bridge.initial_object_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    bridge.initial_object_functional_pose = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

    target_pose = bridge._center_aligned_functional_target_pose()

    assert target_pose is not None
    assert target_pose[:3] == [11.0, 0.0, 0.0]


def test_center_success_place_pose_preserves_current_orientation_and_aligns_center():
    bridge = RoboTwinBridge(task_name="place_container_plate")
    bridge.env = SimpleNamespace()
    bridge.object_attr = "container"
    bridge.target_attr = "plate"
    bridge.get_status = lambda: {  # type: ignore[method-assign]
        "eef_pose": [1.0, 2.0, 3.0, 0.7, 0.0, 0.0, 0.7],
    }
    bridge.get_object_pose = lambda: [4.0, 5.0, 6.0, 1.0, 0.0, 0.0, 0.0]  # type: ignore[method-assign]
    bridge.env.plate = _FunctionalActor(
        center_pose=[10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0],
    )

    target_pose = bridge._compute_center_success_place_pose(pre_distance=0.12)

    assert target_pose is not None
    assert target_pose[:3] == [7.0, 17.0, 27.12]
    assert target_pose[3:7] == [0.7, 0.0, 0.0, 0.7]


def test_predict_object_center_uses_grasp_anchor_and_current_drift():
    bridge = RoboTwinBridge(task_name="place_container_plate")
    blackboard = TaskBlackboard(WorldState())
    blackboard.set("grasp_anchor_eef_pose", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    blackboard.set("grasp_anchor_object_pose", [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    bridge.blackboard = blackboard

    predicted, source, drift = bridge._predict_object_center_for_target_eef_pose(
        current_eef_pose=[0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        current_object_pose=[1.5, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        target_eef_pose=[0.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    )

    assert source == "anchor_rigid_with_drift"
    assert predicted == [1.5, 2.0, 0.0]
    assert drift == [0.5, 0.0, 0.0]


def test_is_grasped_does_not_treat_open_gripper_lifted_object_as_grasped():
    bridge = RoboTwinBridge(task_name="place_container_plate")
    bridge.env = SimpleNamespace()
    bridge.env.get_gripper_actor_contact_position = lambda _name: []
    bridge.object_attr = "container"
    bridge.env.container = _NamedActor(
        "container",
        center_pose=[0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0],
    )
    bridge.initial_object_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    bridge.get_status = lambda: {"gripper_width": 1.0}  # type: ignore[method-assign]

    assert bridge.is_grasped() is False


def test_is_grasped_does_not_treat_closed_gripper_lift_without_contact_as_grasped():
    bridge = RoboTwinBridge(task_name="place_container_plate")
    bridge.env = SimpleNamespace()
    bridge.env.get_gripper_actor_contact_position = lambda _name: []
    bridge.object_attr = "container"
    bridge.env.container = _NamedActor(
        "container",
        center_pose=[0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0],
    )
    bridge.initial_object_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    bridge.get_status = lambda: {"gripper_width": 0.0, "eef_pose": [0.0] * 7}  # type: ignore[method-assign]

    diagnostics = bridge.get_grasp_diagnostics()

    assert diagnostics["lifted"] is True
    assert diagnostics["has_contact"] is False
    assert diagnostics["is_grasped"] is False


def test_is_grasped_accepts_anchor_following_without_live_contact():
    bridge = RoboTwinBridge(task_name="place_container_plate")
    bridge.env = SimpleNamespace()
    bridge.env.get_gripper_actor_contact_position = lambda _name: []
    bridge.object_attr = "container"
    bridge.env.container = _NamedActor(
        "container",
        center_pose=[1.0, 1.0, 0.02, 1.0, 0.0, 0.0, 0.0],
        fp_pose=[1.0, 1.0, 0.02, 1.0, 0.0, 0.0, 0.0],
    )
    bridge.initial_object_pose = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    blackboard = TaskBlackboard(WorldState())
    blackboard.set("grasp_anchor_eef_pose", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    blackboard.set("grasp_anchor_object_pose", [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    bridge.blackboard = blackboard
    bridge.get_status = lambda: {"gripper_width": 0.0, "eef_pose": [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]}  # type: ignore[method-assign]

    diagnostics = bridge.get_grasp_diagnostics()

    assert diagnostics["has_contact"] is False
    assert diagnostics["anchor_following"] is True
    assert diagnostics["is_grasped"] is True


def test_score_pose_candidate_blends_predicted_alignment_with_current_error():
    bridge = RoboTwinBridge(task_name="place_container_plate")
    bridge._estimate_object_center_alignment = lambda _pose: {  # type: ignore[method-assign]
        "dx": 0.0,
        "dy": 0.0,
        "dz": 0.03,
        "xy_norm": 0.0,
        "current_dx": 0.18,
        "current_dy": 0.0,
        "current_dz": 0.03,
        "current_xy_norm": 0.18,
        "prediction_source": "anchor_rigid_with_drift",
        "predicted_object_pose": [0.0, 0.0, 0.0],
        "eef_travel_xy_norm": 0.18,
        "eef_travel_z": 0.02,
        "drift_dx": 0.17,
        "drift_dy": 0.06,
        "drift_dz": -0.2,
        "drift_xy_norm": 0.18,
    }

    scored = bridge.score_pose_candidate([0.0] * 7, kind="place_release")
    metrics = scored["predicted_object_to_target_center_delta"]

    assert 0.05 <= metrics["transport_confidence"] <= 0.95
    assert metrics["partial_correction_gain"] > 0.0
    assert metrics["realized_correction_fraction"] >= metrics["transport_confidence"]
    assert metrics["xy_norm"] > 0.0
    assert metrics["xy_norm"] < 0.18
    assert metrics["effective_dx"] > 0.0
    assert metrics["uncertainty_xy"] > 0.0
    assert metrics["correction_xy_norm"] > 0.0
    assert metrics["correction_risk_xy"] > 0.0
    assert scored["score_adjust"] < 0.0


def test_preferred_contact_family_uses_contact_groups_when_reference_available():
    family = RoboTwinBridge._preferred_contact_family(
        2,
        available_contact_ids=[0, 1, 2, 3],
        metadata={"contact_groups": [[0, 1], [2, 3]]},
    )

    assert family == {2, 3}


def test_preferred_contact_family_falls_back_to_empty_when_reference_missing_and_no_group():
    family = RoboTwinBridge._preferred_contact_family(
        2,
        available_contact_ids=[0, 1],
        metadata={"contact_groups": []},
    )

    assert family == set()


def test_annotate_task_specific_grasp_candidates_marks_incompatible_opposite_group():
    bridge = RoboTwinBridge(task_name="place_container_plate", active_arm="right")
    bridge._object_model_name = lambda: "002_bowl"  # type: ignore[method-assign]
    bridge._object_model_id = lambda: 1  # type: ignore[method-assign]
    bridge._load_object_point_metadata = lambda: {  # type: ignore[method-assign]
        "contact_description": "On the edge of the bowl",
        "contact_groups": [[0, 1], [2, 3]],
    }
    candidates = [
        {"variant_label": "contact_0", "contact_point_id": 0},
        {"variant_label": "contact_2", "contact_point_id": 2},
    ]

    annotated = bridge._annotate_task_specific_grasp_candidates(candidates, arm="right")

    assert annotated[0]["task_compatibility"] == "preferred"
    assert annotated[0]["affordance"]["notes"].startswith("On the edge of the bowl")
    assert annotated[1]["task_compatibility"] == "incompatible"
    assert annotated[1]["contact_group_index"] == 1


def test_annotate_task_specific_grasp_candidates_keeps_generic_compatibility_when_reference_unavailable():
    bridge = RoboTwinBridge(task_name="place_container_plate", active_arm="left")
    bridge._object_model_name = lambda: "021_cup"  # type: ignore[method-assign]
    bridge._object_model_id = lambda: 7  # type: ignore[method-assign]
    bridge._load_object_point_metadata = lambda: {  # type: ignore[method-assign]
        "contact_description": "On the edge of the cup",
        "contact_groups": [],
    }
    candidates = [
        {"variant_label": "contact_0", "contact_point_id": 0},
        {"variant_label": "contact_1", "contact_point_id": 1},
    ]

    annotated = bridge._annotate_task_specific_grasp_candidates(candidates, arm="left")

    assert annotated[0]["task_compatibility"] == "compatible"
    assert annotated[1]["task_compatibility"] == "compatible"
    assert "reference_contact_unavailable_in_current_instance" in annotated[0]["affordance"]["notes"]


def test_build_grasp_candidate_refresh_diagnostic_marks_failure_to_success_flip():
    bridge = RoboTwinBridge(task_name="place_container_plate", active_arm="left")

    previous_candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 520,
            "score": 1.6,
            "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
        },
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "arm": "left",
            "planner_status": "Failure",
            "planner_waypoint_count": None,
            "score": 0.1,
            "pose": [0.2, -0.1, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.1, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0],
        },
    ]
    current_candidates = [
        {
            "variant_label": "contact_0",
            "contact_point_id": 0,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 510,
            "score": 1.55,
            "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
        },
        {
            "variant_label": "contact_1",
            "contact_point_id": 1,
            "arm": "left",
            "planner_status": "Success",
            "planner_waypoint_count": 580,
            "score": 1.45,
            "pose": [0.21, -0.09, 0.2, 0.0, 0.0, 0.0, 1.0],
            "pregrasp_pose": [0.11, -0.09, 0.31, 0.0, 0.0, 0.0, 1.0],
        },
    ]

    diagnostic = bridge._build_grasp_candidate_refresh_diagnostic(
        previous_candidates=previous_candidates,
        current_candidates=current_candidates,
        previous_active=previous_candidates[0],
        current_active=current_candidates[1],
        refresh_reason="post_ExecuteGraspPhase",
        status={"eef_pose": [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0]},
        object_pose=[0.4, -0.1, 0.7, 1.0, 0.0, 0.0, 0.0],
        grasped=False,
    )

    assert diagnostic["refresh_reason"] == "post_ExecuteGraspPhase"
    assert diagnostic["candidate_count_before"] == 2
    assert diagnostic["candidate_count_after"] == 2
    assert diagnostic["current_active_candidate"]["contact_point_id"] == 1
    assert len(diagnostic["improved_candidates"]) == 1
    improved = diagnostic["improved_candidates"][0]
    assert improved["previous_status"] == "Failure"
    assert improved["current_status"] == "Success"
    assert improved["current"]["contact_point_id"] == 1
