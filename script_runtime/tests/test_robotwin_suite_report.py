from script_runtime.runners.evaluate_robotwin_multitask_suite import aggregate_runs
from script_runtime.validation.robotwin_suite_report import (
    build_markdown_report,
    build_run_human_summary,
    build_suite_human_summary,
)


def test_robotwin_suite_report_starts_with_human_summary():
    runs = [
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
    report = {
        "suite_name": "complex_probe",
        "suite_role": "complex_probe",
        "gate": False,
        "suite_path": "/tmp/complex.yaml",
        "artifact_dir": "/tmp/complex_artifacts",
        "require_isolated": True,
        "isolated": True,
        "runs": runs,
        "aggregate": aggregate_runs(runs, skipped_entries=[]),
        "skipped_entries": [],
    }

    markdown = build_markdown_report(report)

    assert markdown.startswith("# RoboTwin Multitask Suite: complex_probe\n\n## Human Summary")
    assert "当前最需要看的失败前沿" in markdown
    assert "## Machine Details" in markdown
    assert "`Top Candidate` 是排序最靠前的抓取候选" in markdown


def test_robotwin_suite_report_preserves_fm_compare_tables():
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
        }
    ]
    report = {
        "suite_name": "fm_compare",
        "suite_role": "fm_compare",
        "gate": False,
        "suite_path": "/tmp/fm.yaml",
        "artifact_dir": "/tmp/fm_artifacts",
        "require_isolated": True,
        "isolated": True,
        "runs": runs,
        "aggregate": aggregate_runs(runs, skipped_entries=[]),
        "skipped_entries": [],
    }

    markdown = build_markdown_report(report)

    assert "## FM Backend Compare" in markdown
    assert "| contact_graspnet | 0 | 0 | 0 | 1 | 1 | 0 |" in markdown
    assert "## FM Compare Matrix" in markdown
    assert "selected:guided_c0" in markdown
    assert "below:contact_0" in markdown


def test_robotwin_suite_report_builds_run_and_suite_human_summaries():
    row = {
        "task": "place_container_plate",
        "seed": 2,
        "failure_stage": "grasp_closure",
        "final_status": "failure",
        "env_success": False,
        "selected_backend": "oracle_feasibility_first",
        "contract_gap_hint": "object_grasp_closure_or_candidate_family_gap",
        "executed_candidate": {"variant_family": "contact_1"},
    }
    run_summary = build_run_human_summary(row)
    report = {
        "gate": True,
        "runs": [row],
        "aggregate": aggregate_runs([row], skipped_entries=[]),
    }

    suite_summary = build_suite_human_summary(report)

    assert "place_container_plate seed=2 未通过" in run_summary
    assert "抓取闭合阶段" in run_summary
    assert "contact_1" in run_summary
    assert suite_summary["run_count"] == 1
    assert "正式门禁报告" in suite_summary["verdict"]
