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

    assert "## Backend Selection" in markdown
    assert "| depth_synthesized | 2 |" in markdown
    assert "| fallback_delegate | 2 |" in markdown
    assert "- `fallback_selected_over_fm_backend`: `1`" in markdown
    assert "- `no_runtime_candidates`: `1`" in markdown
