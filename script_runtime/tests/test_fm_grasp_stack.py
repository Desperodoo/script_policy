import script_runtime.adapters.fm_grasp_stack as fm_grasp_stack_module
from script_runtime.adapters import FMFirstGraspStackAdapter, MockSDKBridge, build_default_fm_first_grasp_stack
from script_runtime.adapters.fm_grasp_stack import (
    BackendResult,
    ContactGraspNetBackend,
    FoundationPoseEstimator,
    GraspProposalBackend,
    GroundedSAM2Grounder,
    GroundingResult,
    ObjectPoseEstimator,
    PerceptionObservation,
    TargetGrounder,
)
from script_runtime.core import SkillContext
from pathlib import Path
import subprocess


class _Grounder(TargetGrounder):
    name = "dummy_grounder"

    def ground(self, observation, context=None):
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=GroundingResult(target_name="cup", source=self.name, score=0.9),
            diagnostics={"stage": "ground"},
        )


class _PoseEstimator(ObjectPoseEstimator):
    def __init__(self, name, pose=None):
        self.name = name
        self.pose = pose

    def estimate_pose(self, observation, *, context=None, grounding=None):
        if self.pose is None:
            return BackendResult(backend_name=self.name, available=True, ok=False, message="no_pose")
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=list(self.pose),
            diagnostics={"target_name": None if grounding is None else grounding.target_name},
        )


class _GraspBackend(GraspProposalBackend):
    def __init__(self, name, candidates):
        self.name = name
        self.candidates = candidates

    def propose_grasps(self, observation, *, context=None, grounding=None, object_pose=None):
        if not self.candidates:
            return BackendResult(backend_name=self.name, available=True, ok=False, message="no_candidates")
        payload = []
        for candidate in self.candidates:
            item = dict(candidate)
            item["proposal_backend"] = self.name
            item.setdefault("proposal_sources", [self.name])
            payload.append(item)
        return BackendResult(
            backend_name=self.name,
            available=True,
            ok=True,
            payload=payload,
            diagnostics={"candidate_count": len(payload), "target_name": None if grounding is None else grounding.target_name},
        )


def test_fm_first_stack_uses_first_successful_pose_backend(blackboard):
    stack = FMFirstGraspStackAdapter(
        target_grounders=[_Grounder()],
        pose_estimators=[
            _PoseEstimator("foundationpose", None),
            _PoseEstimator("robotwin_depth", [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
        ],
        grasp_backends=[],
    )

    pose = stack.get_object_pose(PerceptionObservation(task_goal={"target_object": "cup"}), context=SkillContext(blackboard=blackboard))

    assert pose == [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
    assert stack.last_pose_source == "robotwin_depth"
    assert stack.last_target_source == "dummy_grounder"
    assert stack.last_pose_diagnostics[0]["backend_name"] == "foundationpose"
    assert stack.last_pose_diagnostics[1]["backend_name"] == "robotwin_depth"


def test_fm_first_stack_merges_multiple_grasp_backends(blackboard):
    stack = FMFirstGraspStackAdapter(
        target_grounders=[_Grounder()],
        pose_estimators=[_PoseEstimator("robotwin_depth", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])],
        grasp_backends=[
            _GraspBackend(
                "contact_graspnet",
                [
                    {
                        "contact_point_id": 1,
                        "arm": "left",
                        "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Success",
                        "score": 0.4,
                    }
                ],
            ),
            _GraspBackend(
                "graspnet_baseline",
                [
                    {
                        "contact_point_id": 2,
                        "arm": "left",
                        "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Failure",
                        "score": 0.8,
                    }
                ],
            ),
        ],
    )

    context = SkillContext(blackboard=blackboard)
    candidates = stack.get_grasp_candidates(PerceptionObservation(task_goal={"target_object": "cup"}), context=context)

    assert candidates is not None
    assert [candidate["contact_point_id"] for candidate in candidates] == [1, 2]
    assert stack.last_grasp_source == "contact_graspnet"
    assert [row["backend_name"] for row in stack.last_grasp_diagnostics] == ["contact_graspnet", "graspnet_baseline"]


def test_fm_first_stack_marks_fallback_when_delegate_outranks_successful_fm_backend(blackboard):
    stack = FMFirstGraspStackAdapter(
        target_grounders=[_Grounder()],
        pose_estimators=[_PoseEstimator("robotwin_depth", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])],
        grasp_backends=[
            _GraspBackend(
                "contact_graspnet",
                [
                    {
                        "contact_point_id": 9,
                        "arm": "right",
                        "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Failure",
                        "score": 0.0,
                        "variant_label": "contact_graspnet_seg1_9",
                    }
                ],
            ),
            _GraspBackend(
                "depth_synthesized",
                [
                    {
                        "contact_point_id": 1,
                        "arm": "right",
                        "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Success",
                        "score": 0.0,
                        "variant_label": "contact_0",
                    }
                ],
            ),
        ],
    )

    candidates = stack.get_grasp_candidates(PerceptionObservation(task_goal={"target_object": "cup"}), context=SkillContext(blackboard=blackboard))

    assert candidates is not None
    assert stack.last_grasp_source == "depth_synthesized"
    assert stack.last_grasp_stage_summary["selected_backend_kind"] == "fallback_delegate"
    assert stack.last_grasp_stage_summary["fallback_reason"] == "fallback_selected_over_fm_backend"


def test_fm_first_stack_reranker_prefers_contact_graspnet_when_semantics_align(blackboard):
    blackboard.update_world(execution={"task_goal": {"task_name": "place_container_plate"}})
    stack = FMFirstGraspStackAdapter(
        target_grounders=[_Grounder()],
        pose_estimators=[_PoseEstimator("robotwin_depth", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])],
        grasp_backends=[
            _GraspBackend(
                "contact_graspnet",
                [
                    {
                        "arm": "right",
                        "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Success",
                        "planner_waypoint_count": 120,
                        "score": 0.2,
                        "semantic_priority": 0.9,
                        "affordance_type": "rim_grasp",
                        "semantic_source": "contact_graspnet_template_transfer",
                    }
                ],
            ),
            _GraspBackend(
                "depth_synthesized",
                [
                    {
                        "contact_point_id": 0,
                        "arm": "right",
                        "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Success",
                        "planner_waypoint_count": 160,
                        "score": 0.0,
                        "affordance_type": "rim_grasp",
                        "task_compatibility": "preferred",
                    }
                ],
            ),
        ],
    )

    candidates = stack.get_grasp_candidates(
        PerceptionObservation(task_goal={"target_object": "cup"}),
        context=SkillContext(blackboard=blackboard),
    )

    assert candidates is not None
    assert candidates[0]["proposal_backend"] == "contact_graspnet"
    assert stack.last_grasp_source == "contact_graspnet"
    assert stack.last_grasp_stage_summary["selected_backend_kind"] == "fm_backend"


def test_fm_first_stack_exposes_guided_stage_summary(blackboard):
    blackboard.update_world(execution={"task_goal": {"task_name": "place_container_plate"}})
    stack = FMFirstGraspStackAdapter(
        target_grounders=[_Grounder()],
        pose_estimators=[_PoseEstimator("robotwin_depth", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])],
        grasp_backends=[
            _GraspBackend(
                "contact_graspnet",
                [
                    {
                        "arm": "left",
                        "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Success",
                        "planner_waypoint_count": 110,
                        "score": 0.5,
                        "variant_label": "contact_graspnet_guided_c2",
                        "proposal_sources": ["contact_graspnet", "template_contact_2", "guided_contact_family"],
                        "semantic_source": "contact_graspnet_guided_contact_family",
                    },
                    {
                        "arm": "left",
                        "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Failure",
                        "planner_waypoint_count": None,
                        "score": 0.2,
                        "variant_label": "contact_graspnet_guided_c1",
                        "proposal_sources": ["contact_graspnet", "template_contact_1", "guided_contact_family"],
                        "semantic_source": "contact_graspnet_guided_contact_family",
                        "affordance": {"notes": "reference_contact_unavailable_in_current_instance"},
                    },
                    {
                        "arm": "left",
                        "pose": [0.15, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.15, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Failure",
                        "score": 0.1,
                        "variant_label": "contact_graspnet_seg1_0",
                        "proposal_sources": ["contact_graspnet"],
                    },
                ],
            )
        ],
    )

    candidates = stack.get_grasp_candidates(
        PerceptionObservation(task_goal={"target_object": "cup"}),
        context=SkillContext(blackboard=blackboard),
    )

    assert candidates is not None
    assert stack.last_grasp_stage_summary["guided_feasible_families"] == ["contact_graspnet_guided_c2"]
    assert stack.last_grasp_stage_summary["has_guided_feasible_family"] is True
    assert stack.last_grasp_stage_summary["raw_contact_candidate_count"] == 1
    assert stack.last_grasp_stage_summary["guided_candidate_count"] == 2
    assert stack.last_grasp_stage_summary["guided_rejection_reasons"][0]["reason"] == "reference_contact_unavailable"


def test_fm_first_stack_marks_guided_unavailable_when_only_fallback_delegate_survives(blackboard):
    stack = FMFirstGraspStackAdapter(
        target_grounders=[_Grounder()],
        pose_estimators=[_PoseEstimator("robotwin_depth", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])],
        grasp_backends=[
            _GraspBackend(
                "contact_graspnet",
                [
                    {
                        "arm": "left",
                        "pose": [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Failure",
                        "score": 0.2,
                        "variant_label": "contact_graspnet_guided_c1",
                        "proposal_sources": ["contact_graspnet", "template_contact_1", "guided_contact_family"],
                        "affordance": {"notes": "reference_contact_unavailable_in_current_instance"},
                    }
                ],
            ),
            _GraspBackend(
                "depth_synthesized",
                [
                    {
                        "contact_point_id": 0,
                        "arm": "left",
                        "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                        "pregrasp_pose": [0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                        "planner_status": "Success",
                        "score": 0.0,
                        "variant_label": "contact_0",
                    }
                ],
            ),
        ],
    )

    candidates = stack.get_grasp_candidates(PerceptionObservation(task_goal={"target_object": "cup"}), context=SkillContext(blackboard=blackboard))

    assert candidates is not None
    assert stack.last_grasp_stage_summary["selected_backend_kind"] == "fallback_delegate"
    assert stack.last_grasp_stage_summary["has_guided_feasible_family"] is False
    assert stack.last_grasp_stage_summary["guided_rejection_reasons"][0]["reason"] == "reference_contact_unavailable"


def test_contact_graspnet_candidate_selection_reserves_budget_for_guided_and_template():
    backend = ContactGraspNetBackend(max_candidates=6)
    candidates = []
    for index in range(8):
        candidates.append(
            {
                "variant_label": f"contact_graspnet_seg1_{index}",
                "planner_status": "Failure",
                "score": float(10 - index),
            }
        )
    candidates.extend(
        [
            {
                "variant_label": "contact_graspnet_template_c2_seg1_0",
                "planner_status": "Failure",
                "score": 0.5,
                "template_contact_point_id": 2,
            },
            {
                "variant_label": "contact_graspnet_guided_c2_bridge",
                "planner_status": "Failure",
                "score": 0.4,
                "template_contact_point_id": 2,
            },
            {
                "variant_label": "contact_graspnet_guided_c1",
                "planner_status": "Success",
                "score": 0.3,
                "template_contact_point_id": 1,
            },
        ]
    )

    selected = backend._select_runtime_candidates(candidates)
    labels = [row["variant_label"] for row in selected]

    assert len(selected) == 6
    assert "contact_graspnet_guided_c1" in labels
    assert "contact_graspnet_guided_c2_bridge" in labels
    assert "contact_graspnet_template_c2_seg1_0" in labels


def test_contact_graspnet_template_debug_reports_missing_feasible_templates():
    backend = ContactGraspNetBackend()

    class _SDK:
        def get_grasp_candidates(self, observation=None, context=None):
            return [
                {"variant_label": "contact_0", "arm": "right", "planner_status": "Success", "pose": [0]*7, "pregrasp_pose": [0]*7},
                {"variant_label": "contact_1", "arm": "left", "planner_status": "Failure", "pose": [0]*7, "pregrasp_pose": [0]*7},
            ]

    selected = backend._load_template_candidates(
        sdk=_SDK(),
        observation=PerceptionObservation(task_goal={"target_object": "cup"}),
        context=None,
        active_arm="left",
    )

    assert selected == []
    assert backend.last_template_source_debug["delegate_candidate_count"] == 2
    assert backend.last_template_source_debug["matching_arm_candidate_count"] == 1
    assert backend.last_template_source_debug["pose_ready_template_count"] == 1
    assert backend.last_template_source_debug["feasible_template_count"] == 0
    assert backend.last_template_source_debug["failure_reason"] == "no_feasible_template_candidates"


def test_contact_graspnet_guided_bridge_can_use_non_feasible_template_donors():
    backend = ContactGraspNetBackend(max_candidates=6)

    class _Actor:
        def get_contact_point(self, contact_id, mode):
            assert mode == "list"
            points = {
                0: [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0],
                1: [0.1, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0],
            }
            return points[int(contact_id)]

    class _SDK:
        active_arm = "left"

        def get_grasp_candidates(self, observation=None, context=None):
            return [
                {
                    "variant_label": "contact_0",
                    "contact_point_id": 0,
                    "arm": "left",
                    "planner_status": "Failure",
                    "pose": [0.0, 0.0, 0.92, 1.0, 0.0, 0.0, 0.0],
                    "pregrasp_pose": [0.0, 0.0, 1.02, 1.0, 0.0, 0.0, 0.0],
                },
                {
                    "variant_label": "contact_1",
                    "contact_point_id": 1,
                    "arm": "left",
                    "planner_status": "Failure",
                    "pose": [0.1, 0.0, 0.92, 1.0, 0.0, 0.0, 0.0],
                    "pregrasp_pose": [0.1, 0.0, 1.02, 1.0, 0.0, 0.0, 0.0],
                },
            ]

        def _object_actor(self):
            return _Actor()

        def _object_model_name(self):
            return "021_cup"

        def _object_model_id(self):
            return 7

        def _default_affordance_type_for_object(self):
            return "rim_grasp"

        def evaluate_pose_candidates(self, poses, kind="pregrasp"):
            return [{"status": "Success", "waypoint_count": 20}] * len(list(poses or []))

    observation = PerceptionObservation(task_goal={"target_object": "cup"})
    strict_templates = backend._load_template_candidates(
        sdk=_SDK(),
        observation=observation,
        context=None,
        active_arm="left",
    )
    donor_templates = backend._load_template_candidates(
        sdk=_SDK(),
        observation=observation,
        context=None,
        active_arm="left",
        require_planner_success=False,
        record_debug=False,
    )

    assert strict_templates == []
    assert len(donor_templates) == 2

    bridge_candidates = backend._build_guided_availability_bridge_candidates(
        sdk=_SDK(),
        templates=donor_templates,
        raw_contact_evidence=[
            {"contact_point": [0.02, 0.0, 0.8], "score": 0.6, "segment_id": "1", "source_index": 0},
            {"contact_point": [0.12, 0.0, 0.8], "score": 0.5, "segment_id": "1", "source_index": 1},
        ],
        active_arm="left",
        object_pose=[0.05, 0.0, 0.78, 1.0, 0.0, 0.0, 0.0],
        existing_candidates=[],
        pregrasp_distance=0.1,
    )

    labels = [row["variant_label"] for row in bridge_candidates]
    assert labels == ["contact_graspnet_guided_c0_bridge", "contact_graspnet_guided_c1_bridge"]
    assert all(row["planner_status"] == "Success" for row in bridge_candidates)
    assert all("guided_availability_bridge" in list(row.get("proposal_sources") or []) for row in bridge_candidates)
    assert all(row.get("semantic_source") == "contact_graspnet_guided_availability_bridge" for row in bridge_candidates)


def test_contact_graspnet_prefers_template_delegate_for_template_source_debug():
    class _TemplateDelegate:
        def get_grasp_candidates(self, observation=None, context=None):
            return [
                {
                    "variant_label": "depth_contact_0",
                    "contact_point_id": 0,
                    "arm": "left",
                    "planner_status": "Success",
                    "pose": [0.0] * 7,
                    "pregrasp_pose": [0.0] * 7,
                }
            ]

    class _SDK:
        def get_grasp_candidates(self, observation=None, context=None):
            return [
                {
                    "variant_label": "sdk_contact_0",
                    "contact_point_id": 9,
                    "arm": "left",
                    "planner_status": "Failure",
                    "pose": [1.0] * 7,
                    "pregrasp_pose": [1.0] * 7,
                }
            ]

    backend = ContactGraspNetBackend(template_delegate=_TemplateDelegate())
    selected = backend._load_template_candidates(
        sdk=_SDK(),
        observation=PerceptionObservation(task_goal={"target_object": "cup"}),
        context=None,
        active_arm="left",
    )

    assert [row["variant_label"] for row in selected] == ["depth_contact_0"]
    assert backend.last_template_source_debug["source_kind"] == "template_delegate"
    assert backend.last_template_source_debug["delegate_candidate_count"] == 1


def test_default_fm_first_stack_can_run_with_mock_oracle():
    sdk = MockSDKBridge()
    stack = build_default_fm_first_grasp_stack(oracle_backend=sdk, robotwin_depth_provider=None)

    pose = stack.get_object_pose(PerceptionObservation(task_goal={"target_object": "cup"}))
    candidates = stack.get_grasp_candidates(PerceptionObservation(task_goal={"target_object": "cup"}))

    assert pose is None
    assert candidates is None
    backend_names = [backend.name for backend in stack.grasp_backends]
    assert "contact_graspnet" in backend_names
    assert "graspnet_baseline" in backend_names
    assert "oracle_feasibility" in backend_names


def test_grounded_sam2_reports_repo_missing_with_clear_diagnostics():
    grounder = GroundedSAM2Grounder(repo_path="third_party/does_not_exist")

    result = grounder.ground(PerceptionObservation(task_goal={"target_object": "cup"}, rgb=[[[0, 0, 0]]]))

    assert result.available is False
    assert result.ok is False
    assert result.message == "repo_missing"
    assert result.diagnostics["repo_exists"] is False


def test_grounded_sam2_rerank_prefers_compact_prominent_box_for_container():
    import numpy as np

    grounder = GroundedSAM2Grounder(repo_path="third_party/Grounded-SAM-2")
    depth = np.full((120, 160), 1000.0, dtype=np.float64)
    depth[20:80, 110:145] = 820.0
    depth[30:95, 35:110] = 940.0
    observation = PerceptionObservation(
        rgb=np.zeros((120, 160, 3), dtype=np.uint8),
        depth=depth,
        task_goal={"target_object": "container", "target_surface": "plate"},
    )
    candidates = [
        {"index": 0, "label": "container", "score": 0.63, "box_xyxy": [35, 30, 110, 95], "exact_match": 1, "substring_match": 1},
        {"index": 1, "label": "container", "score": 0.61, "box_xyxy": [110, 20, 145, 80], "exact_match": 1, "substring_match": 1},
    ]

    ranked = grounder._rerank_candidates(candidates, observation=observation, target_name="container")

    assert ranked[0]["index"] == 1
    assert ranked[0]["overall_score"] > ranked[1]["overall_score"]
    assert ranked[0]["semantic_profile"] == "upright_container"


def test_grounded_sam2_rerank_prefers_large_flat_box_for_plate():
    import numpy as np

    grounder = GroundedSAM2Grounder(repo_path="third_party/Grounded-SAM-2")
    depth = np.full((120, 160), 1000.0, dtype=np.float64)
    depth[20:80, 110:145] = 820.0
    depth[30:95, 35:110] = 940.0
    observation = PerceptionObservation(
        rgb=np.zeros((120, 160, 3), dtype=np.uint8),
        depth=depth,
        task_goal={"target_object": "plate"},
    )
    candidates = [
        {"index": 0, "label": "plate", "score": 0.63, "box_xyxy": [35, 30, 110, 95], "exact_match": 1, "substring_match": 1},
        {"index": 1, "label": "plate", "score": 0.61, "box_xyxy": [110, 20, 145, 80], "exact_match": 1, "substring_match": 1},
    ]

    ranked = grounder._rerank_candidates(candidates, observation=observation, target_name="plate")

    assert ranked[0]["index"] == 0
    assert ranked[0]["semantic_profile"] == "flat_surface"


def test_grounded_sam2_rerank_penalizes_overlap_with_target_surface():
    import numpy as np

    grounder = GroundedSAM2Grounder(repo_path="third_party/Grounded-SAM-2")
    depth = np.full((120, 160), 1000.0, dtype=np.float64)
    depth[20:80, 110:145] = 820.0
    depth[30:95, 35:110] = 930.0
    observation = PerceptionObservation(
        rgb=np.zeros((120, 160, 3), dtype=np.uint8),
        depth=depth,
        task_goal={"target_object": "container", "target_surface": "plate"},
    )
    candidates = [
        {"index": 0, "label": "container", "score": 0.69, "box_xyxy": [35, 30, 110, 95], "exact_match": 1, "substring_match": 1},
        {"index": 1, "label": "container", "score": 0.61, "box_xyxy": [110, 20, 145, 80], "exact_match": 1, "substring_match": 1},
    ]
    avoid_candidates = [
        {"index": 0, "label": "plate", "score": 0.77, "box_xyxy": [30, 26, 112, 98], "exact_match": 1, "substring_match": 1},
    ]

    ranked = grounder._rerank_candidates(
        candidates,
        observation=observation,
        target_name="container",
        avoid_candidates=avoid_candidates,
    )

    assert ranked[0]["index"] == 1
    assert ranked[1]["surface_overlap_ratio"] > 0.8
    assert ranked[0]["surface_overlap_ratio"] < 0.1


def test_grounded_sam2_refines_box_to_depth_component_mask():
    import numpy as np

    grounder = GroundedSAM2Grounder(repo_path="third_party/Grounded-SAM-2")
    depth = np.full((120, 160), 1000.0, dtype=np.float64)
    depth[42:72, 98:123] = 790.0
    observation = PerceptionObservation(
        rgb=np.zeros((120, 160, 3), dtype=np.uint8),
        depth=depth,
        task_goal={"target_object": "container", "target_surface": "plate"},
    )

    summary = grounder._refine_candidate_mask(
        observation=observation,
        box_xyxy=[85, 28, 135, 95],
        profile={"kind": "upright_container"},
    )

    bbox_area = (135 - 85) * (95 - 28)
    assert summary["mask_source"] == "depth_refined_component"
    assert 0 < summary["foreground_pixels"] < bbox_area
    assert summary["foreground_ratio"] < 0.5


def test_grounded_sam2_prefers_sam2_instance_mask_when_available():
    import numpy as np

    grounder = GroundedSAM2Grounder(repo_path="third_party/Grounded-SAM-2")
    depth = np.full((120, 160), 1000.0, dtype=np.float64)
    depth[40:76, 102:128] = 785.0
    observation = PerceptionObservation(
        rgb=np.zeros((120, 160, 3), dtype=np.uint8),
        depth=depth,
        task_goal={"target_object": "container", "target_surface": "plate"},
    )
    sam2_mask = np.zeros((120, 160), dtype=bool)
    sam2_mask[43:73, 104:124] = True

    summary = grounder._refine_candidate_mask(
        observation=observation,
        box_xyxy=[90, 28, 136, 92],
        profile={"kind": "upright_container"},
        sam2_mask=sam2_mask,
    )

    assert summary["mask_source"] == "sam2_instance_mask"
    assert summary["foreground_pixels"] == int(np.count_nonzero(sam2_mask))
    assert summary["component_count"] >= 1


def test_grounded_sam2_rerank_prefers_mask_overlap_over_box_overlap(monkeypatch):
    import numpy as np

    grounder = GroundedSAM2Grounder(repo_path="third_party/Grounded-SAM-2")
    depth = np.full((120, 160), 1000.0, dtype=np.float64)
    observation = PerceptionObservation(
        rgb=np.zeros((120, 160, 3), dtype=np.uint8),
        depth=depth,
        task_goal={"target_object": "container", "target_surface": "plate"},
    )
    candidates = [
        {"index": 0, "label": "container", "score": 0.62, "box_xyxy": [40, 20, 120, 100], "exact_match": 1, "substring_match": 1},
        {"index": 1, "label": "container", "score": 0.78, "box_xyxy": [40, 20, 120, 100], "exact_match": 1, "substring_match": 1},
    ]
    avoid_candidates = [
        {"index": 10, "label": "plate", "score": 0.88, "box_xyxy": [40, 20, 120, 100], "exact_match": 1, "substring_match": 1},
    ]

    target_clear_mask = np.zeros((120, 160), dtype=bool)
    target_clear_mask[32:88, 96:122] = True
    target_overlap_mask = np.zeros((120, 160), dtype=bool)
    target_overlap_mask[36:94, 44:92] = True
    avoid_mask = np.zeros((120, 160), dtype=bool)
    avoid_mask[34:92, 42:90] = True

    def fake_predict_masks(*, observation, candidates):
        indexes = {int(row["index"]) for row in candidates}
        if indexes == {0, 1}:
            return {0: target_clear_mask, 1: target_overlap_mask}
        if indexes == {10}:
            return {10: avoid_mask}
        return {}

    monkeypatch.setattr(grounder, "_predict_sam2_masks", fake_predict_masks)

    ranked = grounder._rerank_candidates(
        candidates,
        observation=observation,
        target_name="container",
        avoid_candidates=avoid_candidates,
        avoid_target_name="plate",
    )

    assert ranked[0]["index"] == 0
    assert ranked[0]["surface_overlap_ratio"] < 0.15
    assert ranked[1]["surface_overlap_ratio"] > 0.7


def test_foundationpose_readiness_accepts_no_diffusion_weight_layout(tmp_path, monkeypatch):
    repo = tmp_path / "FoundationPose"
    weight_root = repo / "weights" / "no_diffusion"
    for run_name in ("2023-10-28-18-33-37", "2024-01-11-20-02-45"):
        run_dir = weight_root / run_name
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "config.yml").write_text("dummy: true\n", encoding="utf-8")
        (run_dir / "model_best.pth").write_bytes(b"stub")

    dependency_flags = {
        "open3d": True,
        "trimesh": True,
        "pytorch3d": True,
        "nvdiffrast": True,
        "omegaconf": True,
    }
    monkeypatch.setattr(
        fm_grasp_stack_module,
        "_python_import_status",
        lambda module_names, python_bin=None: {
            "python_bin": python_bin or "current-python",
            "python_bin_exists": True,
            "module_status": {name: dependency_flags.get(name, False) for name in module_names},
            "module_errors": {},
        },
    )

    readiness = FoundationPoseEstimator(repo_path=str(repo))._readiness()

    assert readiness["available"] is True
    assert readiness["message"] == "ready_for_external_run"
    assert readiness["required_weight_dirs"]["2023-10-28-18-33-37"].endswith("2023-10-28-18-33-37")
    assert readiness["required_weight_dirs"]["2024-01-11-20-02-45"].endswith("2024-01-11-20-02-45")


def test_contact_graspnet_readiness_accepts_nested_checkpoint_layout(tmp_path, monkeypatch):
    repo = tmp_path / "contact_graspnet"
    ckpt_dir = repo / "checkpoints" / "contact_graspnet_models" / "scene_test_2048_bs3_hor_sigma_001"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    (ckpt_dir / "config.yaml").write_text("dummy: true\n", encoding="utf-8")
    (ckpt_dir / "model.ckpt-1.index").write_text("stub\n", encoding="utf-8")
    (ckpt_dir / "model.ckpt-1.data-00000-of-00001").write_text("stub\n", encoding="utf-8")

    monkeypatch.setattr(
        fm_grasp_stack_module,
        "_python_import_status",
        lambda module_names, python_bin=None: {
            "python_bin": python_bin or "current-python",
            "python_bin_exists": True,
            "module_status": {name: name == "tensorflow" for name in module_names},
            "module_errors": {},
        },
    )

    readiness = ContactGraspNetBackend(repo_path=str(repo))._readiness()

    assert readiness["available"] is True
    assert readiness["message"] == "ready_for_external_run"
    assert readiness["preferred_checkpoint_dir"].endswith("scene_test_2048_bs3_hor_sigma_001")


def test_foundationpose_readiness_uses_external_python_status(tmp_path, monkeypatch):
    repo = tmp_path / "FoundationPose"
    weight_root = repo / "weights" / "no_diffusion"
    for run_name in ("2023-10-28-18-33-37", "2024-01-11-20-02-45"):
        run_dir = weight_root / run_name
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "config.yml").write_text("dummy: true\n", encoding="utf-8")
        (run_dir / "model_best.pth").write_bytes(b"stub")

    monkeypatch.setattr(
        fm_grasp_stack_module,
        "_python_import_status",
        lambda module_names, python_bin=None: {
            "python_bin": python_bin or "/tmp/fake-python",
            "python_bin_exists": True,
            "module_status": {name: True for name in module_names},
            "module_errors": {},
        },
    )

    readiness = FoundationPoseEstimator(repo_path=str(repo), python_bin="/tmp/fake-python")._readiness()

    assert readiness["available"] is True
    assert readiness["python_bin"] == "/tmp/fake-python"
    assert readiness["python_import_status"]["python_bin_exists"] is True


def test_contact_graspnet_readiness_uses_external_python_status(tmp_path, monkeypatch):
    repo = tmp_path / "contact_graspnet"
    ckpt_dir = repo / "checkpoints" / "contact_graspnet_models" / "scene_test_2048_bs3_hor_sigma_001"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    (ckpt_dir / "config.yaml").write_text("dummy: true\n", encoding="utf-8")
    (ckpt_dir / "model.ckpt-1.index").write_text("stub\n", encoding="utf-8")
    (ckpt_dir / "model.ckpt-1.data-00000-of-00001").write_text("stub\n", encoding="utf-8")

    monkeypatch.setattr(
        fm_grasp_stack_module,
        "_python_import_status",
        lambda module_names, python_bin=None: {
            "python_bin": python_bin or "/tmp/fake-python",
            "python_bin_exists": True,
            "module_status": {name: name == "tensorflow" for name in module_names},
            "module_errors": {},
        },
    )

    readiness = ContactGraspNetBackend(repo_path=str(repo), python_bin="/tmp/fake-python")._readiness()

    assert readiness["available"] is True
    assert readiness["python_bin"] == "/tmp/fake-python"
    assert readiness["python_import_status"]["module_status"]["tensorflow"] is True


def test_contact_graspnet_runtime_reads_summary_from_out_dir_not_last_cli_arg(tmp_path, monkeypatch):
    repo = tmp_path / "contact_graspnet"
    export_dir = tmp_path / "contact_graspnet"
    out_dir = export_dir / "contact_graspnet_headless"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "contact_graspnet_summary.json").write_text('{"ok": true, "grasp_total": 1}', encoding="utf-8")

    backend = ContactGraspNetBackend(repo_path=str(repo), python_bin="/tmp/fake-python")
    monkeypatch.setattr(fm_grasp_stack_module, "_backend_run_dir", lambda context, backend_name: export_dir)

    monkeypatch.setattr(
        backend,
        "_readiness",
        lambda: {
            "available": True,
            "message": "ready_for_external_run",
            "resolved_repo_path": str(repo),
            "preferred_checkpoint_dir": str(repo / "ckpt"),
        },
    )
    monkeypatch.setattr(
        backend,
        "_export_runtime_input",
        lambda **kwargs: {"ok": True, "npz_path": str(tmp_path / "input.npz")},
    )
    monkeypatch.setattr(
        backend,
        "_build_command",
        lambda **kwargs: [
            "/tmp/fake-python",
            "runner.py",
            "--out-dir",
            str(out_dir),
            "--filter-grasps",
        ],
    )
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args=args[0], returncode=0, stdout="ok", stderr=""),
    )
    monkeypatch.setattr(
        backend,
        "_load_runtime_candidates",
        lambda **kwargs: [
            {
                "pose": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                "pregrasp_pose": [0.1, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0],
                "arm": "right",
                "variant_label": "contact_0",
                "contact_point_id": 0,
                "planner_status": "Success",
            }
        ],
    )

    result = backend.propose_grasps(PerceptionObservation(task_goal={"target_object": "container"}))

    assert result.ok is True
    assert result.message == "runtime_success"
    assert result.diagnostics["summary"]["ok"] is True
    assert Path(result.diagnostics["summary_path"]) == out_dir / "contact_graspnet_summary.json"
    assert Path(result.diagnostics["output_dir"]) == out_dir


def test_contact_graspnet_runtime_builds_template_transfer_candidates(tmp_path, blackboard):
    import numpy as np

    output_dir = tmp_path / "contact_graspnet_headless"
    output_dir.mkdir(parents=True, exist_ok=True)
    grasp_mat = np.eye(4, dtype=np.float64)
    grasp_mat[:3, 3] = np.asarray([0.34, 0.20, 0.12], dtype=np.float64)
    np.savez(
        output_dir / "segment_1_grasps.npz",
        pred_grasps_cam=np.asarray([grasp_mat], dtype=np.float64),
        scores=np.asarray([0.31], dtype=np.float64),
        contact_pts=np.asarray([[0.30, 0.20, 0.12]], dtype=np.float64),
    )

    class _TemplateSDK:
        task_name = "place_container_plate"
        active_arm = "right"
        pregrasp_distance = 0.10

        def get_grasp_candidates(self, *_args, **_kwargs):
            return [
                {
                    "variant_label": "contact_0",
                    "contact_point_id": 0,
                    "arm": "right",
                    "pose": [0.18, 0.0, 0.12, 1.0, 0.0, 0.0, 0.0],
                    "pregrasp_pose": [0.08, 0.0, 0.12, 1.0, 0.0, 0.0, 0.0],
                    "planner_status": "Success",
                }
            ]

        def evaluate_pose_candidates(self, poses, kind="pregrasp"):
            rows = []
            for pose in poses:
                x = round(float(pose[0]), 2)
                if kind == "pregrasp":
                    ok = abs(x - 0.08) < 1e-2
                else:
                    ok = abs(x - 0.18) < 1e-2
                row = {"kind": kind, "status": "Success" if ok else "Failure", "pose": list(pose)}
                if ok:
                    row["waypoint_count"] = 24 if kind == "pregrasp" else 18
                rows.append(row)
            return rows

        def _object_model_name(self):
            return "021_cup"

        def _object_model_id(self):
            return 2

        def _default_affordance_type_for_object(self):
            return "rim_grasp"

    backend = ContactGraspNetBackend(repo_path=str(tmp_path))
    observation = PerceptionObservation(
        task_goal={"target_object": "container"},
        metadata={"camera_params": {"cam2world_cv": np.eye(4, dtype=np.float64).tolist()}},
    )
    context = SkillContext(blackboard=blackboard, adapters={"sdk": _TemplateSDK()})

    candidates = backend._load_runtime_candidates(
        output_dir=output_dir,
        observation=observation,
        context=context,
        object_pose=[0.30, 0.20, 0.0, 1.0, 0.0, 0.0, 0.0],
    )

    assert candidates
    assert candidates[0]["planner_status"] == "Success"
    assert candidates[0]["proposal_backend"] == "contact_graspnet"
    assert candidates[0]["variant_label"].startswith("contact_graspnet_template_c0_1_0")
    assert candidates[0]["affordance_type"] == "rim_grasp"
    assert candidates[0]["semantic_source"] == "contact_graspnet_template_transfer"
    assert candidates[0]["semantic_priority"] > 0.5


def test_delegate_grasp_backend_supports_zero_arg_delegate():
    class _ZeroArgDelegate:
        def get_grasp_candidates(self):
            return [
                {
                    "pose": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "pregrasp_pose": [0.1, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0],
                    "arm": "right",
                    "variant_label": "contact_0",
                    "planner_status": "Success",
                }
            ]

    backend = fm_grasp_stack_module.DelegateGraspProposalBackend("depth_synthesized", _ZeroArgDelegate())

    result = backend.propose_grasps(PerceptionObservation(task_goal={"target_object": "container"}))

    assert result.ok is True
    assert result.payload[0]["proposal_backend"] == "depth_synthesized"
