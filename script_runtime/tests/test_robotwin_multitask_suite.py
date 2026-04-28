import json
from pathlib import Path
from types import SimpleNamespace

from script_runtime.runners.evaluate_robotwin_multitask_suite import (
    RunSpec,
    _clear_previous_run_outputs,
    aggregate_runs,
    build_markdown_report,
    classify_failure_stage,
    expand_suite_entries,
    extract_run_summary,
    run_suite,
)


def _valid_task_config(task_name: str = "place_empty_cup") -> str:
    return f"""
runtime:
  task_id: ""
  trace_path: trace.jsonl
  artifact_dir: artifacts
robotwin:
  task_name: {task_name}
  object_attr: cup
  target_attr: coaster
  target_functional_point_id: 0
  object_functional_point_id: 0
  episode_name: {task_name}
task_goal:
  task_name: {task_name}
  target_object: cup
  target_surface: coaster
"""


def test_expand_suite_entries_supports_defaults_overrides_and_disabled_tasks(tmp_path: Path):
    valid_cfg = tmp_path / "valid.yaml"
    valid_cfg.write_text(_valid_task_config("place_empty_cup"), encoding="utf-8")

    invalid_cfg = tmp_path / "invalid.yaml"
    invalid_cfg.write_text(
        """
runtime:
  task_id: ""
robotwin:
  task_name: place_phone_stand
  object_attr: phone
  target_functional_point_id: 0
  object_functional_point_id: 0
  episode_name: place_phone_stand
task_goal:
  task_name: place_phone_stand
  target_object: phone
  target_surface: stand
""",
        encoding="utf-8",
    )

    suite_path = tmp_path / "suite.yaml"
    suite_config = {
        "suite_name": "demo_suite",
        "suite_role": "gate",
        "gate": True,
        "default_seeds": [1, 2],
        "default_no_video": True,
        "entries": [
            {
                "name": "task_default",
                "group": "regression_existing",
                "mode": "baseline",
                "enabled": True,
                "config_path": "valid.yaml",
                "canary": True,
                "canary_seeds": [2],
            },
            {
                "name": "task_override",
                "group": "config_first_place",
                "mode": "fm_first",
                "enabled": True,
                "config_path": "valid.yaml",
                "seeds": [7],
                "no_video": False,
                "task_contract": "articulated_probe",
                "probe_type": "articulated",
                "config_overrides": {
                    "task_goal": {"task_name": "open_microwave", "target_object": "microwave_handle", "target_surface": "microwave_door"},
                    "perception_stack": {"type": "fm_first", "enabled": {"contact_graspnet": True}},
                },
            },
            {
                "name": "task_disabled",
                "group": "deferred_complex_place",
                "mode": "baseline",
                "enabled": False,
                "config_path": "missing.yaml",
                "deferred_reason": "requires new task tree",
            },
            {
                "name": "task_invalid_contract",
                "group": "config_first_place",
                "mode": "baseline",
                "enabled": True,
                "config_path": "invalid.yaml",
            },
        ],
    }

    run_specs, skipped = expand_suite_entries(suite_config, suite_path=suite_path)

    assert [(spec.entry_name, spec.seed, spec.no_video) for spec in run_specs] == [
        ("task_default", 1, True),
        ("task_default", 2, True),
        ("task_override", 7, False),
    ]
    assert run_specs[0].suite_role == "gate"
    assert run_specs[0].gate is True
    assert run_specs[0].task_contract == "pick_place"
    assert run_specs[0].canary is False
    assert run_specs[1].canary is True
    assert run_specs[2].mode == "fm_first"
    assert run_specs[2].task_contract == "articulated_probe"
    assert run_specs[2].probe_type == "articulated"
    assert run_specs[2].config_overrides["perception_stack"]["type"] == "fm_first"
    assert {row["name"]: row["status"] for row in skipped} == {
        "task_disabled": "deferred",
        "task_invalid_contract": "unsupported_contract",
    }


def test_classify_failure_stage_detects_grasp_proposal_grasp_closure_lift_and_success_mismatch():
    grasp_proposal_rows = [
        {"skill_name": "GetObjectPose", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"skill_name": "GetGraspCandidates", "result": "FAILURE", "failure_code": "NO_GRASP_CANDIDATE", "inputs_summary": {"payload": {"message": "No grasp candidates"}}},
    ]
    assert (
        classify_failure_stage(
            grasp_proposal_rows,
            runtime_status="FAILURE",
            failure_code="NO_GRASP_CANDIDATE",
            env_success=False,
            message="No grasp candidates",
        )
        == "grasp_proposal"
    )

    grasp_closure_rows = [
        {"skill_name": "ExecuteGraspPhase", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}}},
    ]
    assert (
        classify_failure_stage(
            grasp_closure_rows,
            runtime_status="FAILURE",
            failure_code="GRASP_FAIL",
            env_success=False,
            message="Grasp failed",
        )
        == "grasp_closure"
    )

    lift_rows = [
        {"skill_name": "CheckGrasp", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {"grasp_confirmed": True}}},
        {"skill_name": "Lift", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"skill_name": "CheckGrasp", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {"message": "Dropped after lift"}}},
    ]
    assert (
        classify_failure_stage(
            lift_rows,
            runtime_status="FAILURE",
            failure_code="GRASP_FAIL",
            env_success=False,
            message="Dropped after lift",
        )
        == "lift_persistence"
    )

    success_mismatch_rows = [
        {
            "skill_name": "CheckTaskSuccess",
            "result": "FAILURE",
            "failure_code": "UNKNOWN",
            "inputs_summary": {"payload": {"message": "Environment success check failed", "env_result": {"ok": True, "success": False}}},
        }
    ]
    assert (
        classify_failure_stage(
            success_mismatch_rows,
            runtime_status="FAILURE",
            failure_code="UNKNOWN",
            env_success=False,
            message="Environment success check failed",
        )
        == "success_mismatch"
    )

    assert (
        classify_failure_stage(
            [],
            runtime_status="ERROR",
            failure_code="NONE",
            env_success=False,
            error_text='Failed to find a supported physical device "cuda:1"',
        )
        == "setup_or_contract"
    )

    assert (
        classify_failure_stage(
            [],
            runtime_status="FAILURE",
            failure_code="TIMEOUT",
            env_success=False,
            message="prepare_gripper_timeout exceeded timeout",
        )
        == "pregrasp_motion"
    )

    assert (
        classify_failure_stage(
            [
                {
                    "skill_name": "ExecuteGraspPhase",
                    "result": "FAILURE",
                    "failure_code": "GRASP_FAIL",
                    "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}},
                }
            ],
            runtime_status="FAILURE",
            failure_code="TIMEOUT",
            env_success=False,
            message="source_prepare_gripper_timeout exceeded timeout",
        )
        == "pregrasp_motion"
    )

    assert (
        classify_failure_stage(
            [
                {
                    "skill_name": "SupportLiftPull",
                    "result": "FAILURE",
                    "failure_code": "SDK_ERROR",
                    "inputs_summary": {
                        "payload": {
                            "message": "support lift pull failed",
                            "sdk_result": {"ok": False, "action": "move_l"},
                        }
                    },
                }
            ],
            runtime_status="FAILURE",
            failure_code="SDK_ERROR",
            env_success=False,
            message="support lift pull failed",
        )
        == "place_motion"
    )


def test_classify_failure_stage_can_use_terminal_failure_override():
    rows = [
        {"node_name": "source_execute_grasp_phase", "skill_name": "ExecuteGraspPhase", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {}}},
        {"node_name": "source_lift", "skill_name": "Lift", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"node_name": "check_receiver_grasp", "skill_name": "CheckGrasp", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {}}},
    ]

    assert (
        classify_failure_stage(
            rows,
            runtime_status="FAILURE",
            failure_code="GRASP_FAIL",
            env_success=False,
            message="Grasp not confirmed",
            failure_row_index=2,
            failure_row=rows[2],
        )
        == "lift_persistence"
    )


def test_extract_run_summary_prefers_terminal_failure_for_handover_probe():
    spec = RunSpec(
        suite_name="complex_probe",
        suite_role="probe",
        gate=False,
        entry_name="handover_block_probe",
        group="handover",
        mode="baseline",
        config_path="/tmp/handover_block.yaml",
        seed=1,
        no_video=True,
        task_contract="handover_probe",
        probe_type="handover",
    )
    config = {
        "robotwin": {
            "task_name": "handover_block",
            "object_attr": "block",
            "target_attr": "receiver",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "handover_block_probe",
        },
        "task_goal": {
            "task_name": "handover_block",
            "target_object": "block",
            "target_surface": "receiver",
        },
    }
    run_result = SimpleNamespace(status="FAILURE", message="Grasp not confirmed", failure_code="GRASP_FAIL")
    rows = [
        {"node_name": "source_execute_grasp_phase", "skill_name": "ExecuteGraspPhase", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {}}},
        {"node_name": "source_check_grasp", "skill_name": "CheckGrasp", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {"grasp_confirmed": True}}},
        {"node_name": "source_lift", "skill_name": "Lift", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"node_name": "receiver_execute_grasp_phase", "skill_name": "ExecuteGraspPhase", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {"grasp_diagnostics": {"is_grasped": True}}}},
        {"node_name": "receiver_check_grasp", "skill_name": "CheckGrasp", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {"grasp_confirmed": True}}},
        {"node_name": "receiver_lift", "skill_name": "Lift", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"node_name": "receiver_check_grasp_after_lift", "skill_name": "CheckGrasp", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {"grasp_confirmed": True}}},
        {"node_name": "switch_active_arm_to_source", "skill_name": "SwitchActiveArm", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"node_name": "open_source_gripper", "skill_name": "OpenGripper", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"node_name": "switch_active_arm_to_receiver_verify", "skill_name": "SwitchActiveArm", "result": "SUCCESS", "failure_code": "NONE", "inputs_summary": {"payload": {}}},
        {"node_name": "check_receiver_grasp", "skill_name": "CheckGrasp", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {"message": "receiver grasp dropped after transfer"}}},
    ]

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=run_result,
        runtime_artifacts={},
        rows=rows,
    )

    assert summary["initial_failure_skill"] == "ExecuteGraspPhase"
    assert summary["failure_skill"] == "CheckGrasp"
    assert summary["failure_node_name"] == "check_receiver_grasp"
    assert summary["failure_stage"] == "lift_persistence"
    assert summary["probe_stage"] == "ownership_transfer"
    assert summary["terminal_probe_stage"] == "ownership_transfer"
    assert summary["contract_gap_hint"] == "receiver_grasp_persistence_or_ownership_transfer_gap"


def test_extract_run_summary_ignores_handover_cleanup_failure_when_terminal_stage_is_empty():
    spec = RunSpec(
        suite_name="complex_probe",
        suite_role="probe",
        gate=False,
        entry_name="handover_block_probe",
        group="handover",
        mode="baseline",
        config_path="/tmp/handover_block.yaml",
        seed=1,
        no_video=True,
        task_contract="handover_probe",
        probe_type="handover",
    )
    config = {
        "robotwin": {
            "task_name": "handover_block",
            "object_attr": "block",
            "target_attr": "receiver",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "handover_block_probe",
        },
        "task_goal": {
            "task_name": "handover_block",
            "target_object": "block",
            "target_surface": "receiver",
        },
    }
    run_result = SimpleNamespace(status="FAILURE", message="Safe retreat failed", failure_code="SDK_ERROR")
    rows = [
        {"node_name": "source_execute_grasp_phase", "skill_name": "ExecuteGraspPhase", "result": "FAILURE", "failure_code": "GRASP_FAIL", "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}}},
        {"node_name": "safe_retreat", "skill_name": "SafeRetreat", "result": "FAILURE", "failure_code": "SDK_ERROR", "inputs_summary": {"payload": {"message": "Safe retreat failed"}}},
    ]

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=run_result,
        runtime_artifacts={},
        rows=rows,
    )

    assert summary["initial_failure_skill"] == "ExecuteGraspPhase"
    assert summary["failure_skill"] == "ExecuteGraspPhase"
    assert summary["failure_node_name"] == "source_execute_grasp_phase"
    assert summary["failure_stage"] == "grasp_closure"
    assert summary["probe_stage"] == "source_acquisition"
    assert summary["terminal_failure_skill"] == "SafeRetreat"
    assert summary["terminal_failure_node_name"] == "safe_retreat"


def test_extract_run_summary_keeps_source_frontier_when_handover_completion_fails_in_place_motion():
    spec = RunSpec(
        suite_name="complex_probe",
        suite_role="probe",
        gate=False,
        entry_name="handover_block_probe",
        group="handover",
        mode="baseline",
        config_path="/tmp/handover_block.yaml",
        seed=1,
        no_video=True,
        task_contract="handover_probe",
        probe_type="handover",
    )
    config = {
        "robotwin": {
            "task_name": "handover_block",
            "object_attr": "block",
            "target_attr": "receiver",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "handover_block_probe",
        },
        "task_goal": {
            "task_name": "handover_block",
            "target_object": "block",
            "target_surface": "receiver",
        },
    }
    run_result = SimpleNamespace(status="FAILURE", message="move_l failed", failure_code="SDK_ERROR")
    rows = [
        {
            "node_name": "source_execute_grasp_phase",
            "skill_name": "ExecuteGraspPhase",
            "result": "FAILURE",
            "failure_code": "GRASP_FAIL",
            "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}},
        },
        {
            "node_name": "receiver_check_grasp_after_lift",
            "skill_name": "CheckGrasp",
            "result": "FAILURE",
            "failure_code": "GRASP_FAIL",
            "inputs_summary": {"payload": {"message": "Grasp not confirmed"}},
        },
        {
            "node_name": "place_approach",
            "skill_name": "PlaceApproach",
            "result": "FAILURE",
            "failure_code": "SDK_ERROR",
            "inputs_summary": {"payload": {"message": "move_l failed"}},
        },
    ]

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=run_result,
        runtime_artifacts={},
        rows=rows,
    )

    assert summary["initial_failure_skill"] == "ExecuteGraspPhase"
    assert summary["failure_skill"] == "ExecuteGraspPhase"
    assert summary["failure_node_name"] == "source_execute_grasp_phase"
    assert summary["failure_stage"] == "grasp_closure"
    assert summary["probe_stage"] == "source_acquisition"
    assert summary["contract_gap_hint"] == "source_grasp_closure_or_candidate_family_gap"
    assert summary["terminal_failure_skill"] == "PlaceApproach"
    assert summary["terminal_failure_node_name"] == "place_approach"
    assert summary["terminal_probe_stage"] == "handover_completion"


def test_extract_run_summary_handles_optional_fm_first_fields():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="canary_compare",
        gate=False,
        entry_name="place_container_plate",
        group="regression_existing",
        mode="fm_first",
        config_path="config.yaml",
        seed=2,
        no_video=True,
        task_contract="pick_place",
        probe_type="",
        canary=True,
        canary_focus="lift_persistence",
    )
    config = {
        "robotwin": {
            "task_name": "place_container_plate",
            "object_attr": "container",
            "target_attr": "plate",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "place_container_plate",
        },
        "task_goal": {
            "task_name": "place_container_plate",
            "target_object": "container",
            "target_surface": "plate",
        },
    }
    rows = [
        {
            "skill_name": "GetGraspCandidates",
            "result": "SUCCESS",
            "failure_code": "NONE",
            "inputs_summary": {
                "payload": {
                    "selected_backend": "contact_graspnet",
                    "selected_backend_kind": "fm_backend",
                    "fallback_reason": "",
                    "guided_feasible_families": ["guided_c2"],
                    "grasp_candidate_stage_summary": {
                        "template_source_debug": {
                            "source_kind": "template_delegate",
                            "delegate_candidate_count": 2,
                        }
                    },
                    "grasp_candidates": [
                        {
                            "variant_label": "contact_graspnet_guided_c2",
                            "proposal_backend": "contact_graspnet",
                            "planner_status": "Success",
                            "object_model_name": "021_cup",
                            "score": 0.9,
                        }
                    ],
                }
            },
        },
        {
            "skill_name": "CheckTaskSuccess",
            "result": "SUCCESS",
            "failure_code": "NONE",
            "inputs_summary": {"payload": {"env_success": True, "env_result": {"ok": True, "success": True}}},
        },
    ]

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(status="SUCCESS", message="ok"),
        runtime_artifacts={"task_id": "demo_task", "run_dir": "/tmp/demo"},
        rows=rows,
    )

    assert summary["final_status"] == "success"
    assert summary["suite_role"] == "canary_compare"
    assert summary["task_contract"] == "pick_place"
    assert summary["canary"] is True
    assert summary["selected_backend_kind"] == "fm_backend"
    assert summary["guided_feasible_families"] == ["guided_c2"]
    assert summary["template_source_debug"]["source_kind"] == "template_delegate"
    assert summary["backend_compare_diagnostics"] == [
        {
            "backend_name": "contact_graspnet",
            "backend_kind": "fm_backend",
            "available": False,
            "runtime_ok": False,
            "message": "",
            "candidate_count": 1,
            "planner_feasible_candidate_count": 1,
            "planner_failed_candidate_count": 0,
            "compare_state": "backend_candidate_planner_feasible",
            "selection_outcome": "selected",
            "selected": True,
                "top_candidate": {
                    "variant_label": "contact_graspnet_guided_c2",
                    "variant_family": "guided_c2",
                "proposal_backend": "contact_graspnet",
                "planner_status": "Success",
                "planner_waypoint_count": None,
                "task_compatibility": "",
                "affordance_type": "",
                "semantic_source": "",
                "semantic_priority": 0.0,
                "score": 0.9,
                "object_model_name": "021_cup",
                "object_model_id": None,
                "contact_point_id": None,
                "template_contact_point_id": None,
            },
            "top_candidate_runtime_reason": "planner_success",
            "summary_path": "",
            "output_dir": "",
            "headless_summary": {},
        }
    ]
    assert summary["inspect_selected_backend"] == ""
    assert summary["inspect_fallback_reason"] == ""

    summary_without_optional = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(status="FAILURE", message="no guidance"),
        runtime_artifacts={"task_id": "demo_task", "run_dir": "/tmp/demo"},
        rows=[
            {
                "skill_name": "GetGraspCandidates",
                "result": "FAILURE",
                "failure_code": "NO_GRASP_CANDIDATE",
                "inputs_summary": {"payload": {"message": "No grasp candidates"}},
            }
        ],
    )

    assert summary_without_optional["selected_backend_kind"] == ""
    assert summary_without_optional["template_source_debug"] == {}


def test_extract_run_summary_exposes_inspect_backend_fields_from_fm_backend_summary(tmp_path: Path):
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="fm_compare",
        gate=False,
        entry_name="place_empty_cup_graspnet_baseline_compare",
        group="healthy5_graspnet_baseline",
        mode="fm_first",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="pick_place",
        probe_type="",
    )
    config = {
        "robotwin": {
            "task_name": "place_empty_cup",
            "object_attr": "cup",
            "target_attr": "coaster",
        },
        "task_goal": {
            "task_name": "place_empty_cup",
            "target_object": "cup",
            "target_surface": "coaster",
        },
    }
    summary_path = tmp_path / "fm_backend_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "selected_backend": "oracle_feasibility",
                "selected_backend_kind": "fallback_delegate",
                "fallback_reason": "integration_not_implemented",
                "backend_compare_diagnostics": [
                    {
                        "backend_name": "graspnet_baseline",
                        "backend_kind": "fm_backend",
                        "available": True,
                        "runtime_ok": True,
                        "message": "runtime_success",
                        "candidate_count": 4,
                        "planner_feasible_candidate_count": 0,
                        "planner_failed_candidate_count": 4,
                        "compare_state": "backend_candidate_present_but_planner_failed",
                        "selection_outcome": "planner_failed_before_selection",
                        "selected": False,
                        "top_candidate": {
                            "variant_label": "graspnet_baseline_seg0_0",
                            "variant_family": "graspnet_baseline_seg0_0",
                            "proposal_backend": "graspnet_baseline",
                            "planner_status": "Failure",
                        },
                        "top_candidate_runtime_reason": "planner_failed",
                        "summary_path": "/tmp/graspnet_baseline_summary.json",
                        "output_dir": "/tmp/graspnet_baseline",
                        "headless_summary": {"grasp_total": 4},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(status="SUCCESS", message="ok"),
        runtime_artifacts={
            "task_id": "fm-first-seed1",
            "run_dir": "/tmp/fm-first-seed1",
            "fm_backend_summary_json": str(summary_path),
        },
        rows=[
            {
                "skill_name": "GetGraspCandidates",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "selected_backend": "depth_synthesized",
                        "selected_backend_kind": "fallback_delegate",
                        "fallback_reason": "integration_not_implemented",
                        "grasp_candidates": [],
                    }
                },
            },
            {
                "skill_name": "CheckTaskSuccess",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {"payload": {"env_success": True, "env_result": {"ok": True, "success": True}}},
            },
        ],
    )

    assert summary["selected_backend"] == "depth_synthesized"
    assert summary["inspect_selected_backend"] == "oracle_feasibility"
    assert summary["inspect_selected_backend_kind"] == "fallback_delegate"
    assert summary["inspect_fallback_reason"] == "integration_not_implemented"
    assert summary["inspect_backend_compare_diagnostics"][0]["backend_name"] == "graspnet_baseline"
    assert summary["inspect_backend_compare_diagnostics"][0]["compare_state"] == "backend_candidate_present_but_planner_failed"
    assert summary["inspect_backend_compare_diagnostics"][0]["selection_outcome"] == "planner_failed_before_selection"


def test_extract_run_summary_can_derive_inspect_backend_compare_diagnostics_from_fm_inspect_json(tmp_path: Path):
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="fm_compare",
        gate=False,
        entry_name="place_empty_cup_graspgen_compare",
        group="healthy5_graspgen",
        mode="fm_first",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="pick_place",
        probe_type="",
    )
    config = {
        "robotwin": {
            "task_name": "place_empty_cup",
            "object_attr": "cup",
            "target_attr": "coaster",
        },
        "task_goal": {
            "task_name": "place_empty_cup",
            "target_object": "cup",
            "target_surface": "coaster",
        },
    }
    backend_summary_path = tmp_path / "fm_backend_summary.json"
    backend_summary_path.write_text(
        json.dumps(
            {
                "selected_backend": "depth_synthesized",
                "selected_backend_kind": "fallback_delegate",
                "fallback_reason": "no_candidates",
            }
        ),
        encoding="utf-8",
    )
    inspect_json_path = tmp_path / "fm_grasp_inspect.json"
    inspect_json_path.write_text(
        json.dumps(
            {
                "grasp_result": {
                    "payload": {
                        "selected_backend": "depth_synthesized",
                        "selected_backend_kind": "fallback_delegate",
                        "fallback_reason": "no_candidates",
                        "grasp_candidates": [
                            {
                                "variant_label": "contact_0",
                                "proposal_backend": "depth_synthesized",
                                "planner_status": "Success",
                                "score": 0.9,
                            },
                            {
                                "variant_label": "graspgen_seg0_0",
                                "proposal_backend": "graspgen",
                                "planner_status": "Failure",
                                "score": 0.7,
                                "planner_debug": {"pregrasp": {"status": "Failure"}},
                            },
                        ],
                    }
                },
                "grasp_candidate_diagnostics": [
                    {
                        "backend_name": "graspgen",
                        "ok": True,
                        "message": "runtime_success",
                        "diagnostics": {
                            "available": True,
                            "summary_path": "/tmp/graspgen_summary.json",
                            "output_dir": "/tmp/graspgen",
                            "summary": {"grasp_total": 12},
                        },
                    },
                    {
                        "backend_name": "depth_synthesized",
                        "ok": True,
                        "message": "",
                        "diagnostics": {"available": True},
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(status="SUCCESS", message="ok"),
        runtime_artifacts={
            "task_id": "fm-first-seed1",
            "run_dir": "/tmp/fm-first-seed1",
            "fm_backend_summary_json": str(backend_summary_path),
            "fm_grasp_inspect_json": str(inspect_json_path),
        },
        rows=[
            {
                "skill_name": "GetGraspCandidates",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "selected_backend": "depth_synthesized",
                        "selected_backend_kind": "fallback_delegate",
                        "fallback_reason": "no_candidates",
                        "grasp_candidates": [],
                    }
                },
            },
            {
                "skill_name": "CheckTaskSuccess",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {"payload": {"env_success": True, "env_result": {"ok": True, "success": True}}},
            },
        ],
    )

    by_backend = {row["backend_name"]: row for row in summary["inspect_backend_compare_diagnostics"]}
    assert by_backend["graspgen"]["compare_state"] == "backend_candidate_present_but_planner_failed"
    assert by_backend["graspgen"]["selection_outcome"] == "planner_failed_before_selection"
    assert by_backend["depth_synthesized"]["compare_state"] == "backend_candidate_planner_feasible"
    assert by_backend["depth_synthesized"]["selection_outcome"] == "selected"


def test_extract_run_summary_falls_back_to_run_result_failure_code_for_timeouts():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "place_can_basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="prepare_gripper_timeout exceeded timeout",
            failure_code="TIMEOUT",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[],
    )

    assert summary["failure_code"] == "TIMEOUT"
    assert summary["failure_stage"] == "pregrasp_motion"
    assert summary["final_status"] == "failure"


def test_extract_run_summary_prefers_runtime_timeout_over_earlier_trace_failure():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="handover_block_probe",
        group="handover_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="handover_probe",
        probe_type="handover",
    )
    config = {
        "robotwin": {
            "task_name": "handover_block",
            "object_attr": "box",
            "target_attr": "target_box",
            "target_functional_point_id": 1,
            "object_functional_point_id": 0,
            "episode_name": "handover_block",
        },
        "task_goal": {
            "task_name": "handover_block",
            "target_object": "box",
            "target_surface": "target_box",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="source_prepare_gripper_timeout exceeded timeout",
            failure_code="TIMEOUT",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "ExecuteGraspPhase",
                "result": "FAILURE",
                "failure_code": "GRASP_FAIL",
                "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}},
            }
        ],
    )

    assert summary["failure_code"] == "TIMEOUT"
    assert summary["failure_skill"] == "source_prepare_gripper_timeout"
    assert summary["message"] == "source_prepare_gripper_timeout exceeded timeout"
    assert summary["failure_stage"] == "pregrasp_motion"


def test_extract_run_summary_infers_timeout_failure_code_from_runtime_message():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="handover_block_probe",
        group="handover_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="handover_probe",
        probe_type="handover",
    )
    config = {
        "robotwin": {
            "task_name": "handover_block",
            "object_attr": "box",
            "target_attr": "target_box",
            "target_functional_point_id": 1,
            "object_functional_point_id": 0,
            "episode_name": "handover_block",
        },
        "task_goal": {
            "task_name": "handover_block",
            "target_object": "box",
            "target_surface": "target_box",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="receiver_go_pregrasp_timeout exceeded timeout",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[],
    )

    assert summary["failure_code"] == "TIMEOUT"
    assert summary["failure_skill"] == "receiver_go_pregrasp_timeout"
    assert summary["failure_stage"] == "pregrasp_motion"


def test_extract_run_summary_exposes_terminal_failure_alongside_primary_stage():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="canary_compare",
        gate=False,
        entry_name="place_container_plate_fm_first",
        group="canary_compare",
        mode="fm_first",
        config_path="config.yaml",
        seed=2,
        no_video=True,
        task_contract="pick_place",
        probe_type="",
        canary=True,
        canary_focus="lift_persistence",
    )
    config = {
        "robotwin": {
            "task_name": "place_container_plate",
            "object_attr": "cup",
            "target_attr": "plate",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "place_container_plate",
        },
        "task_goal": {
            "task_name": "place_container_plate",
            "target_object": "cup",
            "target_surface": "plate",
        },
    }

    rows = [
        {
            "skill_name": "ExecuteGraspPhase",
            "result": "FAILURE",
            "failure_code": "GRASP_FAIL",
            "inputs_summary": {"message": "Grasp phase did not secure object", "payload": {"grasped": False}},
        },
        {
            "skill_name": "RetryWithNextCandidate",
            "result": "FAILURE",
            "failure_code": "NO_GRASP_CANDIDATE",
            "inputs_summary": {"message": "No planner-feasible next candidate available", "payload": {}},
        },
    ]

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="No planner-feasible next candidate available",
            failure_code="GRASP_FAIL",
        ),
        runtime_artifacts={"task_id": "fm-first-seed2", "run_dir": "/tmp/fm-first-seed2"},
        rows=rows,
    )

    assert summary["failure_stage"] == "grasp_closure"
    assert summary["failure_skill"] == "ExecuteGraspPhase"
    assert summary["terminal_failure_code"] == "NO_GRASP_CANDIDATE"
    assert summary["terminal_failure_skill"] == "RetryWithNextCandidate"
    assert summary["terminal_failure_message"] == "No planner-feasible next candidate available"
    assert summary["terminal_failure_row_index"] == 1


def test_extract_run_summary_exposes_probe_stage_from_node_name():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="open_microwave_probe",
        group="articulated_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="articulated_probe",
        probe_type="articulated",
    )
    config = {
        "robotwin": {
            "task_name": "open_microwave",
            "object_attr": "microwave",
            "target_attr": "microwave",
            "target_functional_point_id": 4,
            "object_functional_point_id": 0,
            "episode_name": "open_microwave",
        },
        "task_goal": {
            "task_name": "open_microwave",
            "target_object": "microwave_handle",
            "target_surface": "microwave_door",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(status="FAILURE", message="target_pose invalid", failure_code="NO_IK"),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "PlaceApproach",
                "node_name": "articulated_handle_alignment",
                "result": "FAILURE",
                "failure_code": "NO_IK",
                "inputs_summary": {"payload": {"message": "target_pose invalid"}},
            }
        ],
    )

    assert summary["failure_node_name"] == "articulated_handle_alignment"
    assert summary["probe_stage"] == "pre_open_alignment"


def test_extract_run_summary_exposes_contract_gap_hint_for_staged_place_follow_up():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
            "target_functional_point_id": 0,
            "object_functional_point_id": 0,
            "episode_name": "place_can_basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    rows = [
        {"skill_name": "OpenGripper", "node_name": "post_place_open_gripper", "result": "SUCCESS", "failure_code": "NONE"},
        {"skill_name": "Retreat", "node_name": "post_place_retreat", "result": "SUCCESS", "failure_code": "NONE"},
        {
            "skill_name": "CheckTaskSuccess",
            "node_name": "check_task_success",
            "result": "FAILURE",
            "failure_code": "UNKNOWN",
            "inputs_summary": {"payload": {"message": "Environment success check failed"}},
        },
    ]

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(status="FAILURE", message="Environment success check failed", failure_code="UNKNOWN"),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=rows,
    )

    assert summary["probe_stage"] == "post_place_follow_up"
    assert summary["contract_gap_hint"] == "support_regrasp_and_basket_lift_missing"


def test_extract_run_summary_exposes_contract_gap_hint_for_support_regrasp_phase():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="Grasp phase did not secure object",
            failure_code="GRASP_FAIL",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "ExecuteGraspPhase",
                "node_name": "support_execute_grasp_phase",
                "result": "FAILURE",
                "failure_code": "GRASP_FAIL",
                "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}},
            }
        ],
    )

    assert summary["probe_stage"] == "support_regrasp"
    assert summary["contract_gap_hint"] == "support_regrasp_grasp_closure_gap"


def test_extract_run_summary_exposes_contract_gap_hint_for_support_pregrasp_motion_failure():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="support_go_pregrasp exceeded timeout",
            failure_code="TIMEOUT",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "PrepareSupportRegrasp",
                "node_name": "prepare_support_regrasp",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "support_arm": "left",
                        "support_target_frame": "robotwin::basket",
                        "support_pregrasp_pose_source": "active_grasp_candidate:contact_2",
                        "support_regrasp_substage": "support_pregrasp_generation",
                    }
                },
            },
            {
                "skill_name": "GoPregrasp",
                "node_name": "support_go_pregrasp",
                "result": "FAILURE",
                "failure_code": "TIMEOUT",
                "inputs_summary": {
                    "payload": {
                        "message": "support_go_pregrasp exceeded timeout",
                        "support_arm": "left",
                        "support_target_frame": "robotwin::basket",
                        "support_pregrasp_pose_source": "active_grasp_candidate:contact_2",
                        "support_regrasp_substage": "support_pregrasp_motion",
                    }
                },
            },
        ],
    )

    assert summary["probe_stage"] == "support_regrasp"
    assert summary["failure_stage"] == "pregrasp_motion"
    assert summary["contract_gap_hint"] == "support_pregrasp_reachability_gap"
    assert summary["support_arm"] == "left"
    assert summary["support_target_frame"] == "robotwin::basket"
    assert summary["support_pregrasp_pose_source"] == "active_grasp_candidate:contact_2"
    assert summary["support_regrasp_substage"] == "support_pregrasp_motion"


def test_extract_run_summary_aligns_place_can_basket_candidates_to_execute_attempt():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="Grasp phase did not secure object",
            failure_code="GRASP_FAIL",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "GetGraspCandidates",
                "node_name": "get_grasp_candidates",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "candidate_source": "oracle_fallback",
                        "grasp_candidates": [
                            {
                                "variant_label": "contact_1",
                                "contact_point_id": 1,
                                "planner_status": "Success",
                                "planner_waypoint_count": 900,
                                "task_compatibility": "preferred",
                                "score": 1.55,
                                "object_model_name": "071_can",
                            },
                            {
                                "variant_label": "contact_0",
                                "contact_point_id": 0,
                                "planner_status": "Success",
                                "planner_waypoint_count": 650,
                                "task_compatibility": "preferred",
                                "score": 1.60,
                                "object_model_name": "071_can",
                            },
                        ],
                    }
                },
            },
            {
                "skill_name": "ExecuteGraspPhase",
                "node_name": "probe_execute_grasp_phase",
                "result": "FAILURE",
                "failure_code": "GRASP_FAIL",
                "inputs_summary": {
                    "payload": {
                        "message": "Grasp phase did not secure object",
                        "grasp_attempt_initial_candidate": {
                            "label": "contact_0",
                            "variant_label": "contact_0",
                            "contact_point_id": 0,
                            "planner_status": "Success",
                            "planner_waypoint_count": 640,
                            "task_compatibility": "preferred",
                            "score": 1.60,
                            "object_model_name": "071_can",
                        },
                        "grasp_attempt_candidate": {
                            "label": "contact_0",
                            "variant_label": "contact_0",
                            "contact_point_id": 0,
                            "planner_status": "Success",
                            "planner_waypoint_count": 640,
                            "task_compatibility": "preferred",
                            "score": 1.60,
                            "object_model_name": "071_can",
                        },
                        "grasp_attempt_candidate_identity": "contact:left:0:contact_0",
                        "grasp_attempt_reselected": False,
                        "grasp_candidate_refresh": {
                            "refresh_reason": "post_ExecuteGraspPhase",
                            "previous_active_candidate": {
                                "label": "contact_0",
                                "variant_label": "contact_0",
                                "contact_point_id": 0,
                                "planner_status": "Success",
                                "planner_waypoint_count": 640,
                                "task_compatibility": "preferred",
                                "score": 1.60,
                                "object_model_name": "071_can",
                            },
                            "previous_candidates": [
                                {
                                    "label": "contact_0",
                                    "variant_label": "contact_0",
                                    "contact_point_id": 0,
                                    "planner_status": "Success",
                                    "planner_waypoint_count": 640,
                                    "task_compatibility": "preferred",
                                    "score": 1.60,
                                    "object_model_name": "071_can",
                                },
                                {
                                    "label": "contact_1",
                                    "variant_label": "contact_1",
                                    "contact_point_id": 1,
                                    "planner_status": "Success",
                                    "planner_waypoint_count": 910,
                                    "task_compatibility": "preferred",
                                    "score": 1.55,
                                    "object_model_name": "071_can",
                                },
                            ],
                        },
                    }
                },
            },
        ],
    )

    assert summary["probe_stage"] == "object_acquisition"
    assert summary["failure_stage"] == "grasp_closure"
    assert summary["contract_gap_hint"] == "object_grasp_closure_or_candidate_family_gap"
    assert summary["top_candidate"]["variant_family"] == "contact_0"
    assert summary["executed_candidate"]["variant_family"] == "contact_0"
    assert summary["attempt_initial_candidate"]["variant_family"] == "contact_0"
    assert summary["attempt_candidate_identity"] == "contact:left:0:contact_0"
    assert summary["attempt_reselected"] is False


def test_extract_run_summary_exposes_attempt_reselection_fields():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="support_go_pregrasp exceeded timeout",
            failure_code="TIMEOUT",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "ExecuteGraspPhase",
                "node_name": "support_execute_grasp_phase",
                "result": "FAILURE",
                "failure_code": "GRASP_FAIL",
                "inputs_summary": {
                    "payload": {
                        "grasp_attempt_initial_candidate": {
                            "variant_label": "contact_0",
                            "contact_point_id": 0,
                            "planner_status": "Success",
                            "planner_waypoint_count": 588,
                            "task_compatibility": "preferred",
                            "score": 1.6,
                        },
                        "grasp_attempt_candidate": {
                            "variant_label": "contact_1",
                            "contact_point_id": 1,
                            "planner_status": "Success",
                            "planner_waypoint_count": 465,
                            "task_compatibility": "preferred",
                            "score": 1.55,
                        },
                        "grasp_attempt_candidate_identity": "contact:left:1:contact_1",
                        "grasp_attempt_reselected": True,
                        "grasp_attempt_reselection_node": "support_reselect_grasp_after_pregrasp",
                        "grasp_attempt_reselection_skill": "ReselectGraspAfterPregrasp",
                        "grasp_attempt_forced_perception_rebuild": True,
                        "grasp_attempt_forced_perception_rebuild_reason": "post_execute_candidates_degraded",
                        "grasp_candidate_refresh": {
                            "previous_active_candidate": {
                                "variant_label": "contact_1",
                                "contact_point_id": 1,
                                "planner_status": "Success",
                                "planner_waypoint_count": 465,
                                "task_compatibility": "preferred",
                                "score": 1.55,
                            },
                            "previous_candidates": [
                                {
                                    "variant_label": "contact_0",
                                    "contact_point_id": 0,
                                    "planner_status": "Success",
                                    "planner_waypoint_count": 588,
                                    "task_compatibility": "preferred",
                                    "score": 1.6,
                                },
                                {
                                    "variant_label": "contact_1",
                                    "contact_point_id": 1,
                                    "planner_status": "Success",
                                    "planner_waypoint_count": 465,
                                    "task_compatibility": "preferred",
                                    "score": 1.55,
                                },
                            ],
                        },
                    }
                },
            },
            {
                "skill_name": "GoPregrasp",
                "node_name": "support_go_pregrasp",
                "result": "FAILURE",
                "failure_code": "TIMEOUT",
                "inputs_summary": {
                    "payload": {
                        "message": "support_go_pregrasp exceeded timeout",
                        "support_regrasp_substage": "support_pregrasp_motion",
                    }
                },
            },
        ],
    )

    assert summary["top_candidate"]["variant_family"] == "contact_0"
    assert summary["executed_candidate"]["variant_family"] == "contact_1"
    assert summary["attempt_reselected"] is True
    assert summary["attempt_reselection_node"] == "support_reselect_grasp_after_pregrasp"
    assert summary["attempt_reselection_skill"] == "ReselectGraspAfterPregrasp"
    assert summary["attempt_forced_perception_rebuild"] is True
    assert [row["variant_family"] for row in summary["top_candidates"][:2]] == ["contact_0", "contact_1"]


def test_extract_run_summary_uses_current_support_attempt_context_for_pregrasp_failure():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="move_l failed",
            failure_code="SDK_ERROR",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "ExecuteGraspPhase",
                "node_name": "probe_execute_grasp_phase",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "grasp_attempt_initial_candidate": {
                            "variant_label": "contact_1",
                            "contact_point_id": 1,
                            "arm": "left",
                            "planner_status": "Success",
                        },
                        "grasp_attempt_candidate": {
                            "variant_label": "contact_1",
                            "contact_point_id": 1,
                            "arm": "left",
                            "planner_status": "Success",
                        },
                        "grasp_attempt_candidate_identity": "contact:left:1:contact_1",
                        "grasp_attempt_reselected": False,
                        "grasp_candidate_refresh": {
                            "previous_active_candidate": {
                                "variant_label": "contact_1",
                                "contact_point_id": 1,
                                "arm": "left",
                                "planner_status": "Success",
                                "score": 1.55,
                            },
                            "previous_candidates": [
                                {
                                    "variant_label": "contact_0",
                                    "contact_point_id": 0,
                                    "arm": "left",
                                    "planner_status": "Success",
                                    "score": 1.6,
                                },
                                {
                                    "variant_label": "contact_1",
                                    "contact_point_id": 1,
                                    "arm": "left",
                                    "planner_status": "Success",
                                    "score": 1.55,
                                },
                            ],
                        },
                    }
                },
            },
            {
                "skill_name": "GetGraspCandidates",
                "node_name": "support_get_grasp_candidates",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "grasp_candidates": [
                            {
                                "variant_label": "contact_0",
                                "contact_point_id": 0,
                                "arm": "right",
                                "planner_status": "Success",
                                "score": 1.6,
                            },
                            {
                                "variant_label": "contact_1",
                                "contact_point_id": 1,
                                "arm": "right",
                                "planner_status": "Success",
                                "score": 1.55,
                            },
                        ]
                    }
                },
            },
            {
                "skill_name": "PrepareGripperForGrasp",
                "node_name": "support_prepare_gripper_for_grasp",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "grasp_candidate_refresh": {
                            "current_active_candidate": {
                                "variant_label": "contact_0",
                                "contact_point_id": 0,
                                "arm": "right",
                                "planner_status": "Success",
                                "score": 1.6,
                            },
                            "current_candidates": [
                                {
                                    "variant_label": "contact_0",
                                    "contact_point_id": 0,
                                    "arm": "right",
                                    "planner_status": "Success",
                                    "score": 1.6,
                                },
                                {
                                    "variant_label": "contact_1",
                                    "contact_point_id": 1,
                                    "arm": "right",
                                    "planner_status": "Success",
                                    "score": 1.55,
                                },
                            ],
                        }
                    }
                },
            },
            {
                "skill_name": "GoPregrasp",
                "node_name": "support_go_pregrasp",
                "result": "FAILURE",
                "failure_code": "SDK_ERROR",
                "inputs_summary": {
                    "payload": {
                        "message": "move_l failed",
                        "support_regrasp_substage": "support_pregrasp_motion",
                    }
                },
                "support_arm": "right",
            },
        ],
    )

    assert summary["probe_stage"] == "support_regrasp"
    assert summary["failure_stage"] == "pregrasp_motion"
    assert summary["contract_gap_hint"] == "support_pregrasp_reachability_gap"
    assert summary["top_candidate"]["variant_family"] == "contact_0"
    assert summary["executed_candidate"]["variant_family"] == "contact_0"
    assert summary["attempt_candidate_identity"] == "contact:right:0:contact_0"
    assert summary["attempt_reselected"] is False


def test_extract_run_summary_exposes_contract_gap_hint_for_staged_place_transfer_release_failure():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="open gripper failed",
            failure_code="SDK_ERROR",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "PlaceRelease",
                "node_name": "place_release",
                "result": "FAILURE",
                "failure_code": "SDK_ERROR",
                "inputs_summary": {"payload": {"message": "open gripper failed"}},
            }
        ],
    )

    assert summary["probe_stage"] == "staged_place_transfer"
    assert summary["failure_stage"] == "place_motion"
    assert summary["contract_gap_hint"] == "post_place_release_or_support_regrasp_missing"


def test_extract_run_summary_exposes_contract_gap_hint_for_support_lift_pull_failure():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="support lift pull failed",
            failure_code="SDK_ERROR",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "SupportLiftPull",
                "node_name": "support_lift_pull",
                "result": "FAILURE",
                "failure_code": "SDK_ERROR",
                "inputs_summary": {
                    "message": "support lift pull failed",
                    "payload": {
                        "sdk_result": {
                            "ok": False,
                            "action": "move_l",
                            "command": {"type": "move_l"},
                        },
                        "support_completion_diagnostics": {
                            "support_completion_subtype": "support_follow_up_motion_unreachable",
                            "support_completion_gap_reason": "support lift succeeded but every follow-up motion plan failed on its first step",
                            "support_completion_reference_frame": "support_pose",
                            "support_completion_geometry_converged": True,
                            "support_motion_plan_names": ["combined_full", "lift_only"],
                            "support_motion_all_failed_at_first_step": True,
                            "support_motion_requested_delta_xyz": [0.02, 0.0, 0.05],
                            "support_motion_start_pose": [0.1, -0.2, 0.9, 0.0, 0.0, 0.0, 1.0],
                            "support_motion_failed_target_pose": [0.12, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0],
                            "after_object_to_support_pose_delta": {"dx": 0.01, "dy": 0.02, "dz": 0.03, "xy_norm": 0.022},
                        },
                    },
                },
            }
        ],
    )

    assert summary["probe_stage"] == "support_regrasp"
    assert summary["failure_stage"] == "place_motion"
    assert summary["contract_gap_hint"] == "basket_lift_or_support_completion_gap"
    assert summary["support_completion_subtype"] == "support_follow_up_motion_unreachable"
    assert summary["support_completion_gap_reason"].startswith("support lift succeeded")
    assert summary["support_completion_reference_frame"] == "support_pose"
    assert summary["support_completion_geometry_converged"] is True
    assert summary["support_completion_diagnostics"]["support_motion_plan_names"] == ["combined_full", "lift_only"]


def test_extract_run_summary_exposes_contract_gap_hint_for_support_lift_failure():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="move_l failed",
            failure_code="SDK_ERROR",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "CheckGrasp",
                "node_name": "support_check_grasp",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {"payload": {"grasp_confirmed": True}},
            },
            {
                "skill_name": "Lift",
                "node_name": "support_lift",
                "result": "FAILURE",
                "failure_code": "SDK_ERROR",
                "inputs_summary": {
                    "message": "move_l failed",
                    "payload": {
                        "sdk_result": {
                            "ok": False,
                            "action": "move_l",
                            "command": {"type": "move_l"},
                        }
                    },
                },
            },
            {
                "skill_name": "SafeRetreat",
                "node_name": "support_safe_retreat",
                "result": "FAILURE",
                "failure_code": "SDK_ERROR",
                "inputs_summary": {"payload": {"message": "Safe retreat failed"}},
            },
        ],
    )

    assert summary["probe_stage"] == "support_regrasp"
    assert summary["failure_stage"] == "lift_persistence"
    assert summary["contract_gap_hint"] == "basket_lift_or_support_completion_gap"
    assert summary["support_completion_subtype"] == "support_lift_motion_unreachable"
    assert summary["support_completion_gap_reason"] == "support lift motion failed before support completion follow-up"
    assert summary["support_completion_reference_frame"] == "support_pose"
    assert summary["support_completion_geometry_converged"] is False


def test_extract_run_summary_anchors_support_check_task_success_to_prior_failed_support_frontier():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="task success contract failed after support lift",
            failure_code="UNKNOWN",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "SupportLiftPull",
                "node_name": "support_lift_pull",
                "result": "FAILURE",
                "failure_code": "SDK_ERROR",
                "inputs_summary": {
                    "payload": {
                        "message": "support lift pull failed",
                        "support_regrasp_substage": "support_lift_completion",
                    }
                },
            },
            {
                "skill_name": "CheckTaskSuccess",
                "node_name": "support_check_task_success",
                "result": "FAILURE",
                "failure_code": "UNKNOWN",
                "inputs_summary": {
                    "payload": {
                        "message": "task success contract failed after support lift",
                        "env_success": False,
                        "support_regrasp_substage": "support_lift_completion",
                    }
                },
            },
        ],
    )

    assert summary["probe_stage"] == "support_regrasp"
    assert summary["failure_skill"] == "SupportLiftPull"
    assert summary["failure_node_name"] == "support_lift_pull"
    assert summary["failure_stage"] == "place_motion"
    assert summary["contract_gap_hint"] == "basket_lift_or_support_completion_gap"
    assert summary["terminal_failure_skill"] == "CheckTaskSuccess"
    assert summary["terminal_failure_node_name"] == "support_check_task_success"


def test_extract_run_summary_keeps_terminal_support_check_task_success_when_prior_support_frontier_succeeded():
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="place_can_basket_probe",
        group="staged_place_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="staged_place_probe",
        probe_type="staged_place",
    )
    config = {
        "robotwin": {
            "task_name": "place_can_basket",
            "object_attr": "can",
            "target_attr": "basket",
        },
        "task_goal": {
            "task_name": "place_can_basket",
            "target_object": "can",
            "target_surface": "basket",
        },
    }

    summary = extract_run_summary(
        spec=spec,
        config=config,
        run_result=SimpleNamespace(
            status="FAILURE",
            message="Environment success check failed",
            failure_code="UNKNOWN",
        ),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "SupportLiftPull",
                "node_name": "support_lift_pull",
                "result": "SUCCESS",
                "failure_code": "NONE",
                "inputs_summary": {
                    "payload": {
                        "support_regrasp_substage": "support_lift_completion",
                    }
                },
            },
            {
                "skill_name": "CheckTaskSuccess",
                "node_name": "support_check_task_success",
                "result": "FAILURE",
                "failure_code": "UNKNOWN",
                "inputs_summary": {
                    "payload": {
                        "message": "Environment success check failed",
                        "env_success": False,
                        "support_regrasp_substage": "support_lift_completion",
                        "support_completion_diagnostics": {
                            "support_completion_subtype": "support_success_conditions_unmet",
                            "support_completion_gap_reason": "support geometry converged but environment success inputs still failed",
                            "support_completion_reference_frame": "support_pose",
                            "support_completion_geometry_converged": True,
                            "after_object_to_support_pose_delta": {
                                "dx": 0.01,
                                "dy": 0.0,
                                "dz": 0.02,
                                "xy_norm": 0.01,
                            },
                            "env_result": {"ok": True, "success": False, "task_name": "place_can_basket"},
                        },
                    }
                },
            },
        ],
    )

    assert summary["failure_skill"] == "CheckTaskSuccess"
    assert summary["failure_node_name"] == "support_check_task_success"
    assert summary["failure_stage"] == "success_mismatch"
    assert summary["probe_stage"] == "support_regrasp"
    assert summary["contract_gap_hint"] == "basket_lift_or_support_completion_gap"
    assert summary["terminal_failure_skill"] == "CheckTaskSuccess"
    assert summary["support_completion_subtype"] == "support_success_conditions_unmet"
    assert summary["support_completion_gap_reason"].startswith("support geometry converged")
    assert summary["support_completion_reference_frame"] == "support_pose"
    assert summary["support_completion_geometry_converged"] is True
    assert summary["support_completion_diagnostics"]["after_object_to_support_pose_delta"]["xy_norm"] == 0.01


def test_extract_run_summary_exposes_contract_gap_hint_for_handover_and_articulated_probes():
    handover_spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="handover_block_probe",
        group="handover_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="handover_probe",
        probe_type="handover",
    )
    handover_config = {
        "robotwin": {
            "task_name": "handover_block",
            "object_attr": "box",
            "target_attr": "target_box",
        },
        "task_goal": {
            "task_name": "handover_block",
            "target_object": "box",
            "target_surface": "target_box",
        },
    }
    handover_summary = extract_run_summary(
        spec=handover_spec,
        config=handover_config,
        run_result=SimpleNamespace(status="FAILURE", message="No planner-feasible next candidate available", failure_code="GRASP_FAIL"),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "ExecuteGraspPhase",
                "node_name": "source_execute_grasp_phase",
                "result": "FAILURE",
                "failure_code": "GRASP_FAIL",
                "inputs_summary": {"payload": {"message": "No planner-feasible next candidate available"}},
            }
        ],
    )
    assert handover_summary["probe_stage"] == "source_acquisition"
    assert handover_summary["contract_gap_hint"] == "source_grasp_closure_or_candidate_family_gap"

    articulated_spec = RunSpec(
        suite_name="demo_suite",
        suite_role="complex_probe",
        gate=False,
        entry_name="open_microwave_probe",
        group="articulated_probe",
        mode="baseline",
        config_path="config.yaml",
        seed=1,
        no_video=True,
        task_contract="articulated_probe",
        probe_type="articulated",
    )
    articulated_config = {
        "robotwin": {
            "task_name": "open_microwave",
            "object_attr": "microwave",
            "target_attr": "microwave",
        },
        "task_goal": {
            "task_name": "open_microwave",
            "target_object": "microwave_handle",
            "target_surface": "microwave_door",
        },
    }
    articulated_summary = extract_run_summary(
        spec=articulated_spec,
        config=articulated_config,
        run_result=SimpleNamespace(status="FAILURE", message="Grasp phase did not secure object", failure_code="GRASP_FAIL"),
        runtime_artifacts={"task_id": "probe-seed1", "run_dir": "/tmp/probe-seed1"},
        rows=[
            {
                "skill_name": "ExecuteGraspPhase",
                "node_name": "articulated_execute_grasp_phase",
                "result": "FAILURE",
                "failure_code": "GRASP_FAIL",
                "inputs_summary": {"payload": {"message": "Grasp phase did not secure object"}},
            }
        ],
    )
    assert articulated_summary["probe_stage"] == "handle_acquisition"
    assert articulated_summary["contract_gap_hint"] == "handle_grasp_closure_gap"


def test_run_suite_forces_isolated_for_gate_suites(tmp_path: Path):
    suite_path = tmp_path / "suite.yaml"
    valid_cfg = tmp_path / "valid.yaml"
    valid_cfg.write_text(_valid_task_config("place_empty_cup"), encoding="utf-8")

    suite_config = {
        "suite_name": "gate_suite",
        "suite_role": "gate",
        "gate": True,
        "require_isolated": True,
        "artifact_dir": str(tmp_path / "artifacts"),
        "default_seeds": [1],
        "default_no_video": True,
        "entries": [
            {"name": "task_default", "group": "regression_existing", "mode": "baseline", "enabled": True, "config_path": "valid.yaml"},
        ],
    }

    report = run_suite(
        suite_config,
        suite_path=suite_path,
        dry_run=True,
    )

    assert report["require_isolated"] is True
    assert report["requested_isolated"] is False
    assert report["isolated"] is True


def test_run_suite_isolates_run_exceptions(tmp_path: Path):
    suite_path = tmp_path / "suite.yaml"
    task_a = tmp_path / "task_a.yaml"
    task_b = tmp_path / "task_b.yaml"
    task_a.write_text(_valid_task_config("place_empty_cup"), encoding="utf-8")
    task_b.write_text(_valid_task_config("place_mouse_pad"), encoding="utf-8")

    suite_config = {
        "suite_name": "isolation_suite",
        "artifact_dir": str(tmp_path / "artifacts"),
        "default_seeds": [1],
        "default_no_video": True,
        "entries": [
            {"name": "task_a", "group": "regression_existing", "mode": "baseline", "enabled": True, "config_path": "task_a.yaml"},
            {"name": "task_b", "group": "regression_existing", "mode": "baseline", "enabled": True, "config_path": "task_b.yaml"},
        ],
    }

    class _FakeSession:
        def __init__(self, config):
            self.config = config
            self.runtime_artifacts = {}

        def run(self):
            task_name = self.config["robotwin"]["task_name"]
            task_id = self.config["runtime"]["task_id"]
            run_dir = Path(self.config["runtime"]["artifact_dir"]) / task_id
            run_dir.mkdir(parents=True, exist_ok=True)
            trace_path = run_dir / f"{task_id}_trace.jsonl"
            if task_name == "place_empty_cup":
                trace_path.write_text(
                    '{"skill_name":"CheckTaskSuccess","result":"SUCCESS","failure_code":"NONE","inputs_summary":{"payload":{"env_success":true,"env_result":{"ok":true,"success":true}}}}\n',
                    encoding="utf-8",
                )
                self.runtime_artifacts = {"task_id": task_id, "run_dir": str(run_dir), "trace_path": str(trace_path)}
                return SimpleNamespace(status="SUCCESS", message="ok")
            raise RuntimeError("boom during run")

        def shutdown(self):
            return {"ok": True}

    report = run_suite(
        suite_config,
        suite_path=suite_path,
        session_builder=lambda config: _FakeSession(config),
    )

    assert report["aggregate"]["run_count"] == 2
    assert {row["entry_name"]: row["final_status"] for row in report["runs"]} == {
        "task_a": "success",
        "task_b": "exception",
    }
    assert report["aggregate"]["per_task"]["place_empty_cup"]["success_count"] == 1
    assert report["aggregate"]["per_task"]["place_mouse_pad"]["run_count"] == 1


def test_run_suite_rebuilds_summary_from_existing_run_summaries(tmp_path: Path):
    suite_path = tmp_path / "suite.yaml"
    task_a = tmp_path / "task_a.yaml"
    task_a.write_text(_valid_task_config("place_empty_cup"), encoding="utf-8")
    artifact_dir = tmp_path / "artifacts"
    legacy_summary_path = artifact_dir / "run_summaries" / "legacy_task.json"
    legacy_summary_path.parent.mkdir(parents=True, exist_ok=True)
    legacy_summary_path.write_text(
        json.dumps(
            {
                "task": "place_mouse_pad",
                "entry_name": "legacy_task",
                "seed": 1,
                "failure_stage": "success",
                "final_status": "success",
                "failure_code": "NONE",
                "run_dir": "/tmp/legacy_task",
                "message": "legacy success",
                "selected_backend": "graspgen",
                "selected_backend_kind": "fm_backend",
                "env_success": True,
                "mode": "fm_first",
                "task_contract": "pick_place",
                "probe_type": "",
                "canary": False,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    suite_config = {
        "suite_name": "compare_suite",
        "suite_role": "fm_compare",
        "artifact_dir": str(artifact_dir),
        "default_seeds": [1],
        "default_no_video": True,
        "entries": [
            {"name": "task_a", "group": "regression_existing", "mode": "baseline", "enabled": True, "config_path": "task_a.yaml"},
        ],
    }

    class _FakeSession:
        def __init__(self, config):
            self.config = config
            self.runtime_artifacts = {}

        def run(self):
            task_name = self.config["robotwin"]["task_name"]
            task_id = self.config["runtime"]["task_id"]
            run_dir = Path(self.config["runtime"]["artifact_dir"]) / task_id
            run_dir.mkdir(parents=True, exist_ok=True)
            trace_path = run_dir / f"{task_id}_trace.jsonl"
            trace_path.write_text(
                '{"skill_name":"CheckTaskSuccess","result":"SUCCESS","failure_code":"NONE","inputs_summary":{"payload":{"env_success":true,"env_result":{"ok":true,"success":true}}}}\n',
                encoding="utf-8",
            )
            self.runtime_artifacts = {"task_id": task_id, "run_dir": str(run_dir), "trace_path": str(trace_path)}
            return SimpleNamespace(status="SUCCESS", message=f"{task_name} ok")

        def shutdown(self):
            return {"ok": True}

    report = run_suite(
        suite_config,
        suite_path=suite_path,
        session_builder=lambda config: _FakeSession(config),
    )

    assert report["invocation_run_count"] == 1
    assert report["aggregate"]["run_count"] == 2
    assert report["aggregate"]["env_success_count"] == 2
    assert set(report["aggregate"]["per_task"]) == {"place_empty_cup", "place_mouse_pad"}
    assert report["aggregate"]["per_task"]["place_empty_cup"]["run_count"] == 1
    assert report["aggregate"]["per_task"]["place_mouse_pad"]["run_count"] == 1
    assert report["aggregate"]["selected_backend_counts"] == {"graspgen": 1}


def test_clear_previous_run_outputs_removes_stale_run_dir_and_summary(tmp_path: Path):
    suite_artifact_dir = tmp_path / "artifacts"
    run_dir = suite_artifact_dir / "runs" / "demo_task"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "stale_trace.jsonl").write_text("old\n", encoding="utf-8")
    summary_path = suite_artifact_dir / "run_summaries" / "demo_task.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text('{"final_status":"success"}', encoding="utf-8")

    _clear_previous_run_outputs(suite_artifact_dir=suite_artifact_dir, task_id="demo_task")

    assert not run_dir.exists()
    assert not summary_path.exists()


def test_aggregate_and_markdown_report_expose_backend_selection_summary():
    runs = [
        {
            "task": "place_empty_cup",
            "entry_name": "place_empty_cup_fm_first",
            "seed": 1,
            "failure_stage": "success_mismatch",
            "final_status": "success_mismatch",
            "failure_code": "UNKNOWN",
            "run_dir": "/tmp/place_empty_cup",
            "message": "Environment success check failed",
            "selected_backend": "contact_graspnet",
            "selected_backend_kind": "fm_backend",
            "fallback_reason": "",
            "env_success": False,
            "mode": "fm_first",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "guided_c2"},
            "executed_candidate": {"variant_family": "guided_c2"},
            "inspect_backend_compare_diagnostics": [
                {
                    "backend_name": "contact_graspnet",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "selected",
                    "top_candidate": {"variant_family": "guided_c2", "planner_status": "Success"},
                }
            ],
        },
        {
            "task": "place_mouse_pad",
            "entry_name": "place_mouse_pad_fm_first",
            "seed": 1,
            "failure_stage": "success",
            "final_status": "success",
            "failure_code": "NONE",
            "run_dir": "/tmp/place_mouse_pad",
            "message": "pick_place_root complete",
            "selected_backend": "depth_synthesized",
            "selected_backend_kind": "fallback_delegate",
            "fallback_reason": "fallback_selected_over_fm_backend",
            "env_success": True,
            "mode": "fm_first",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "contact_1"},
            "executed_candidate": {"variant_family": "contact_1"},
            "inspect_backend_compare_diagnostics": [
                {
                    "backend_name": "graspnet_baseline",
                    "compare_state": "backend_candidate_present_but_planner_failed",
                    "selection_outcome": "planner_failed_before_selection",
                    "top_candidate": {"variant_family": "graspnet_baseline_seg0_0", "planner_status": "Failure"},
                }
            ],
        },
        {
            "task": "place_phone_stand",
            "entry_name": "place_phone_stand_fm_first",
            "seed": 1,
            "failure_stage": "success",
            "final_status": "success",
            "failure_code": "NONE",
            "run_dir": "/tmp/place_phone_stand",
            "message": "pick_place_root complete",
            "selected_backend": "depth_synthesized",
            "selected_backend_kind": "fallback_delegate",
            "fallback_reason": "no_runtime_candidates",
            "env_success": True,
            "mode": "fm_first",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "contact_0"},
            "executed_candidate": {"variant_family": "contact_0"},
            "inspect_backend_compare_diagnostics": [
                {
                    "backend_name": "graspgen",
                    "compare_state": "backend_runtime_ok_but_no_candidate",
                    "selection_outcome": "no_runtime_candidates",
                    "top_candidate": {},
                }
            ],
        },
    ]

    aggregate = aggregate_runs(runs, skipped_entries=[])

    assert aggregate["selected_backend_counts"] == {
        "contact_graspnet": 1,
        "depth_synthesized": 2,
    }
    assert aggregate["selected_backend_kind_counts"] == {
        "fallback_delegate": 2,
        "fm_backend": 1,
    }
    assert aggregate["fallback_reason_counts"] == {
        "fallback_selected_over_fm_backend": 1,
        "no_runtime_candidates": 1,
    }
    assert aggregate["backend_compare_state_counts"] == {
        "contact_graspnet": {"backend_candidate_planner_feasible": 1},
        "graspgen": {"backend_runtime_ok_but_no_candidate": 1},
        "graspnet_baseline": {"backend_candidate_present_but_planner_failed": 1},
    }
    assert aggregate["backend_selection_outcome_counts"] == {
        "contact_graspnet": {"selected": 1},
        "graspgen": {"no_runtime_candidates": 1},
        "graspnet_baseline": {"planner_failed_before_selection": 1},
    }

    report = {
        "suite_name": "demo_fm_compare",
        "suite_role": "fm_compare",
        "gate": False,
        "suite_path": "/tmp/demo_suite.yaml",
        "artifact_dir": "/tmp/demo_artifacts",
        "require_isolated": True,
        "isolated": True,
        "runs": runs,
        "aggregate": aggregate,
        "skipped_entries": [],
    }
    markdown = build_markdown_report(report)

    assert "## Human Summary" in markdown
    assert "本轮 3 次运行中" in markdown
    assert "当前最需要看的失败前沿" in markdown
    assert "## Machine Details" in markdown
    assert "## Backend Selection" in markdown
    assert "| depth_synthesized | 2 |" in markdown
    assert "| fallback_delegate | 2 |" in markdown
    assert "- `fallback_selected_over_fm_backend`: `1`" in markdown
    assert "- `no_runtime_candidates`: `1`" in markdown
    assert "## FM Backend Compare" in markdown
    assert "| graspnet_baseline | 0 | 0 | 1 | 0 | 0 | 0 |" in markdown
    assert "| graspgen | 0 | 1 | 0 | 0 | 0 | 0 |" in markdown
    assert "## FM Compare Details" in markdown
    assert "| place_mouse_pad | 1 | graspnet_baseline | backend_candidate_present_but_planner_failed | planner_failed_before_selection | graspnet_baseline_seg0_0 | Failure |" in markdown
    assert "`Top Candidate` 是排序最靠前的抓取候选" in markdown


def test_build_markdown_report_exposes_fm_compare_matrix_by_task_seed():
    runs = [
        {
            "task": "place_empty_cup",
            "entry_name": "place_empty_cup_contact_graspnet_compare",
            "seed": 2,
            "failure_stage": "success",
            "final_status": "success",
            "failure_code": "NONE",
            "run_dir": "/tmp/place_empty_cup_contact",
            "message": "pick_place_root complete",
            "selected_backend": "contact_graspnet",
            "selected_backend_kind": "fm_backend",
            "fallback_reason": "",
            "env_success": True,
            "mode": "fm_first",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "guided_c0"},
            "executed_candidate": {"variant_family": "guided_c0"},
            "inspect_backend_compare_diagnostics": [
                {
                    "backend_name": "contact_graspnet",
                    "backend_kind": "fm_backend",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "selected",
                    "top_candidate": {"variant_family": "guided_c0", "planner_status": "Success"},
                },
                {
                    "backend_name": "depth_synthesized",
                    "backend_kind": "fallback_delegate",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "ranked_below_selected_backend",
                    "top_candidate": {"variant_family": "contact_0", "planner_status": "Success"},
                },
            ],
        },
        {
            "task": "place_empty_cup",
            "entry_name": "place_empty_cup_graspgen_compare",
            "seed": 2,
            "failure_stage": "success",
            "final_status": "success",
            "failure_code": "NONE",
            "run_dir": "/tmp/place_empty_cup_graspgen",
            "message": "pick_place_root complete",
            "selected_backend": "graspgen",
            "selected_backend_kind": "fm_backend",
            "fallback_reason": "",
            "env_success": True,
            "mode": "fm_first",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "graspgen_guided_c0"},
            "executed_candidate": {"variant_family": "graspgen_guided_c0"},
            "inspect_backend_compare_diagnostics": [
                {
                    "backend_name": "graspgen",
                    "backend_kind": "fm_backend",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "selected",
                    "top_candidate": {"variant_family": "graspgen_guided_c0", "planner_status": "Success"},
                },
                {
                    "backend_name": "depth_synthesized",
                    "backend_kind": "fallback_delegate",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "ranked_below_selected_backend",
                    "top_candidate": {"variant_family": "contact_0", "planner_status": "Success"},
                },
            ],
        },
        {
            "task": "place_empty_cup",
            "entry_name": "place_empty_cup_graspnet_baseline_compare",
            "seed": 2,
            "failure_stage": "success",
            "final_status": "success",
            "failure_code": "NONE",
            "run_dir": "/tmp/place_empty_cup_graspnet_baseline",
            "message": "pick_place_root complete",
            "selected_backend": "graspnet_baseline",
            "selected_backend_kind": "fm_backend",
            "fallback_reason": "",
            "env_success": True,
            "mode": "fm_first",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "graspnet_baseline_guided_c0"},
            "executed_candidate": {"variant_family": "graspnet_baseline_guided_c0"},
            "inspect_backend_compare_diagnostics": [
                {
                    "backend_name": "graspnet_baseline",
                    "backend_kind": "fm_backend",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "selected",
                    "top_candidate": {"variant_family": "graspnet_baseline_guided_c0", "planner_status": "Success"},
                },
                {
                    "backend_name": "depth_synthesized",
                    "backend_kind": "fallback_delegate",
                    "compare_state": "backend_candidate_planner_feasible",
                    "selection_outcome": "ranked_below_selected_backend",
                    "top_candidate": {"variant_family": "contact_0", "planner_status": "Success"},
                },
            ],
        },
    ]

    aggregate = aggregate_runs(runs, skipped_entries=[])
    report = {
        "suite_name": "demo_fm_compare_matrix",
        "suite_role": "fm_compare",
        "gate": False,
        "suite_path": "/tmp/demo_suite.yaml",
        "artifact_dir": "/tmp/demo_artifacts",
        "require_isolated": True,
        "isolated": True,
        "runs": runs,
        "aggregate": aggregate,
        "skipped_entries": [],
    }

    markdown = build_markdown_report(report)

    assert "## FM Compare Matrix" in markdown
    assert "| place_empty_cup | 2 | selected:guided_c0 | selected:graspnet_baseline_guided_c0 | selected:graspgen_guided_c0 | contact_graspnet:contact_graspnet, graspgen:graspgen, graspnet_baseline:graspnet_baseline | below:contact_0 |" in markdown


def test_extract_run_summary_adds_human_summary_without_removing_machine_fields():
    rows = [
        {
            "skill_name": "GetGraspCandidates",
            "node_name": "get_grasp_candidates",
            "result": "SUCCESS",
            "failure_code": "NONE",
            "inputs_summary": {
                "payload": {
                    "grasp_candidates": [
                        {
                            "variant_label": "contact_1",
                            "variant_family": "contact_1",
                            "pose": [0, 0, 0, 1, 0, 0, 0],
                            "pregrasp_pose": [0, 0, 0.1, 1, 0, 0, 0],
                            "planner_status": "Success",
                        }
                    ],
                    "candidate_source": "oracle_fallback",
                    "selected_backend": "oracle_feasibility_first",
                }
            },
        },
        {
            "skill_name": "ExecuteGraspPhase",
            "node_name": "execute_grasp_phase",
            "result": "FAILURE",
            "failure_code": "GRASP_FAIL",
            "inputs_summary": {
                "payload": {
                    "message": "Grasp phase did not secure object",
                    "grasp_attempt_candidate": {
                        "variant_label": "contact_1",
                        "variant_family": "contact_1",
                        "pose": [0, 0, 0, 1, 0, 0, 0],
                    },
                    "grasp_attempt_candidate_identity": "contact:left:1:contact_1",
                }
            },
        },
    ]
    spec = RunSpec(
        suite_name="demo_suite",
        suite_role="gate",
        gate=True,
        entry_name="place_container_plate",
        group="regression_existing",
        mode="baseline",
        config_path="tasks/place_container_plate_robotwin.yaml",
        seed=2,
        no_video=True,
        task_contract="pick_place",
    )
    summary = extract_run_summary(
        spec=spec,
        config={
            "robotwin": {"task_name": "place_container_plate", "object_attr": "container", "target_attr": "plate"},
            "task_goal": {"task_name": "place_container_plate", "target_object": "container", "target_surface": "plate"},
        },
        run_result=SimpleNamespace(status="FAILURE", message="Grasp failed"),
        runtime_artifacts={"task_id": "demo_task", "run_dir": "/tmp/demo_task"},
        rows=rows,
    )

    assert summary["failure_stage"] == "grasp_closure"
    assert summary["selected_backend"] == "oracle_feasibility_first"
    assert summary["attempt_candidate_identity"] == "contact:left:1:contact_1"
    assert "human_summary" in summary
    assert "place_container_plate seed=2 未通过" in summary["human_summary"]
    assert "抓取闭合阶段" in summary["human_summary"]
    assert "contact_1" in summary["human_summary"]


def test_markdown_human_summary_handles_all_success_complex_probe_and_empty_reports():
    success_runs = [
        {
            "task": "place_empty_cup",
            "entry_name": "place_empty_cup",
            "seed": 1,
            "failure_stage": "success",
            "final_status": "success",
            "failure_code": "NONE",
            "run_dir": "/tmp/place_empty_cup",
            "message": "complete",
            "selected_backend": "oracle_feasibility_first",
            "selected_backend_kind": "",
            "env_success": True,
            "mode": "baseline",
            "task_contract": "pick_place",
            "probe_type": "",
            "canary": False,
            "top_candidate": {"variant_family": "contact_0"},
            "executed_candidate": {"variant_family": "contact_0"},
        }
    ]
    success_report = {
        "suite_name": "success_suite",
        "suite_role": "gate",
        "gate": True,
        "suite_path": "/tmp/suite.yaml",
        "artifact_dir": "/tmp/artifacts",
        "require_isolated": True,
        "isolated": True,
        "runs": success_runs,
        "aggregate": aggregate_runs(success_runs, skipped_entries=[]),
        "skipped_entries": [],
    }
    success_md = build_markdown_report(success_report)
    assert "全部通过" in success_md
    assert "## Machine Details" in success_md

    complex_runs = [
        {
            "task": "place_can_basket",
            "entry_name": "place_can_basket_probe",
            "seed": 1,
            "failure_stage": "success_mismatch",
            "final_status": "success_mismatch",
            "failure_code": "UNKNOWN",
            "run_dir": "/tmp/place_can_basket",
            "message": "Environment success check failed",
            "selected_backend": "oracle_feasibility_first",
            "selected_backend_kind": "",
            "env_success": False,
            "mode": "baseline",
            "task_contract": "staged_place_probe",
            "probe_type": "staged_place",
            "probe_stage": "support_regrasp",
            "contract_gap_hint": "basket_lift_or_support_completion_gap",
            "canary": False,
            "top_candidate": {"variant_family": "contact_0"},
            "executed_candidate": {"variant_family": "contact_0"},
        }
    ]
    complex_report = {
        "suite_name": "complex_probe",
        "suite_role": "complex_probe",
        "gate": False,
        "suite_path": "/tmp/complex.yaml",
        "artifact_dir": "/tmp/complex_artifacts",
        "require_isolated": True,
        "isolated": True,
        "runs": complex_runs,
        "aggregate": aggregate_runs(complex_runs, skipped_entries=[]),
        "skipped_entries": [],
    }
    complex_md = build_markdown_report(complex_report)
    assert "动作完成但环境未判成功阶段" in complex_md
    assert "basket_lift_or_support_completion_gap" in complex_md

    empty_report = {
        "suite_name": "empty_suite",
        "suite_role": "gate",
        "gate": True,
        "suite_path": "/tmp/empty.yaml",
        "artifact_dir": "/tmp/empty_artifacts",
        "require_isolated": False,
        "isolated": False,
        "runs": [],
        "aggregate": aggregate_runs([], skipped_entries=[]),
        "skipped_entries": [],
    }
    empty_md = build_markdown_report(empty_report)
    assert "本次没有实际运行记录" in empty_md
