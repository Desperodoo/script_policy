from pathlib import Path
from types import SimpleNamespace

from script_runtime.runners.evaluate_robotwin_multitask_suite import (
    RunSpec,
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
        "default_seeds": [1, 2],
        "default_no_video": True,
        "entries": [
            {
                "name": "task_default",
                "group": "regression_existing",
                "mode": "baseline",
                "enabled": True,
                "config_path": "valid.yaml",
            },
            {
                "name": "task_override",
                "group": "config_first_place",
                "mode": "baseline",
                "enabled": True,
                "config_path": "valid.yaml",
                "seeds": [7],
                "no_video": False,
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


def test_extract_run_summary_handles_optional_fm_first_fields():
    spec = RunSpec(
        suite_name="demo_suite",
        entry_name="place_container_plate",
        group="regression_existing",
        mode="fm_first",
        config_path="config.yaml",
        seed=2,
        no_video=True,
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
    assert summary["selected_backend_kind"] == "fm_backend"
    assert summary["guided_feasible_families"] == ["guided_c2"]
    assert summary["template_source_debug"]["source_kind"] == "template_delegate"

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
