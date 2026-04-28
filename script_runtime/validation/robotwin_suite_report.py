"""Human-first reports for RoboTwin multitask suites."""

from __future__ import annotations

import json
from typing import Any, Dict, List, Tuple

SUCCESS_STAGE = "success"
UNKNOWN_STAGE = "unknown_failure"

STAGE_HUMAN_LABELS = {
    SUCCESS_STAGE: "任务通过",
    "grounding_or_pose": "目标定位或物体位姿",
    "grasp_proposal": "抓取候选生成",
    "grasp_closure": "抓取闭合",
    "lift_persistence": "抬起后保持抓取",
    "place_motion": "放置运动",
    "pregrasp_motion": "抓取前靠近运动",
    "success_mismatch": "动作完成但环境未判成功",
    "setup_or_contract": "环境或任务配置",
    "unsupported_contract": "暂不支持的任务合同",
    "exception": "运行异常",
    UNKNOWN_STAGE: "未知失败",
}

FALLBACK_GRASP_BACKENDS = {
    "oracle_feasibility",
    "oracle_feasibility_first",
    "depth_synthesized",
}


def _backend_kind_from_name(backend_name: str) -> str:
    name = str(backend_name or "").strip()
    if name in FALLBACK_GRASP_BACKENDS:
        return "fallback_delegate"
    return "fm_backend"


def _human_stage_label(stage: Any) -> str:
    text = str(stage or UNKNOWN_STAGE)
    return STAGE_HUMAN_LABELS.get(text, text or STAGE_HUMAN_LABELS[UNKNOWN_STAGE])


def _human_candidate_label(candidate: Dict[str, Any] | None, identity: str = "") -> str:
    item = dict(candidate or {})
    label = str(item.get("variant_family") or item.get("variant_label") or "").strip()
    if label:
        return label
    if identity:
        return identity
    contact_id = item.get("contact_point_id")
    if contact_id is not None:
        arm = str(item.get("arm") or "").strip()
        return f"{arm} arm contact {contact_id}".strip()
    return "未记录"


def build_run_human_summary(row: Dict[str, Any]) -> str:
    """Build one human-readable sentence for a run summary."""

    task = str(row.get("task") or row.get("entry_name") or "当前任务")
    seed = row.get("seed", "")
    seed_text = f" seed={seed}" if seed not in ("", None) else ""
    final_status = str(row.get("final_status") or "")
    stage = str(row.get("failure_stage") or UNKNOWN_STAGE)
    stage_label = _human_stage_label(stage)
    executed = _human_candidate_label(
        dict(row.get("executed_candidate") or {}),
        identity=str(row.get("attempt_candidate_identity") or ""),
    )
    backend = str(row.get("selected_backend") or "").strip()
    hint = str(row.get("contract_gap_hint") or "").strip()

    if final_status == "success" or stage == SUCCESS_STAGE or bool(row.get("env_success", False)):
        base = f"{task}{seed_text} 通过；实际执行的抓取候选是 {executed}。"
    else:
        base = f"{task}{seed_text} 未通过；主要卡在{stage_label}阶段，实际执行的抓取候选是 {executed}。"
    details = []
    if backend:
        details.append(f"候选来源/后端：{backend}")
    if hint:
        details.append(f"机器提示：{hint}")
    if details:
        base += " " + "；".join(details) + "。"
    return base


def build_suite_human_summary(report: Dict[str, Any]) -> Dict[str, Any]:
    """Build the short front-page summary for a suite report."""

    aggregate = dict(report.get("aggregate") or {})
    runs = [dict(row or {}) for row in list(report.get("runs") or [])]
    run_count = int(aggregate.get("run_count", 0) or 0)
    success_count = int(aggregate.get("success_count", 0) or 0)
    env_success_count = int(aggregate.get("env_success_count", 0) or 0)
    gate = bool(report.get("gate", False))
    stage_counts = dict(aggregate.get("failure_cluster_counts") or {})
    failed_runs = [
        row
        for row in runs
        if str(row.get("failure_stage") or SUCCESS_STAGE) != SUCCESS_STAGE
        or str(row.get("final_status") or "") not in {"success", ""}
        or ("env_success" in row and not bool(row.get("env_success", False)))
    ]

    if run_count == 0:
        verdict = "本次没有实际运行记录。"
    elif not failed_runs:
        verdict = f"本轮 {run_count} 次运行全部通过，环境成功 {env_success_count}/{run_count}。"
    else:
        verdict = f"本轮 {run_count} 次运行中，runtime 成功 {success_count}/{run_count}，环境成功 {env_success_count}/{run_count}。"
    if gate:
        verdict += " 这是正式门禁报告，需要优先确认失败项是否会阻塞后续推进。"

    focus = "当前没有失败前沿。"
    next_step = "可以继续保护性回归，或进入下一项功能推进。"
    if failed_runs:
        ordered_stages = sorted(
            ((stage, int(count)) for stage, count in stage_counts.items() if stage != SUCCESS_STAGE),
            key=lambda item: (-item[1], item[0]),
        )
        main_stage = ordered_stages[0][0] if ordered_stages else str(failed_runs[0].get("failure_stage") or UNKNOWN_STAGE)
        main_stage_label = _human_stage_label(main_stage)
        representative = next(
            (row for row in failed_runs if str(row.get("failure_stage") or "") == main_stage),
            failed_runs[0],
        )
        task = str(representative.get("task") or representative.get("entry_name") or "未知任务")
        seed = representative.get("seed", "")
        seed_text = f" seed={seed}" if seed not in ("", None) else ""
        focus = f"当前最需要看的失败前沿是：{task}{seed_text} 的{main_stage_label}阶段。"
        if str(representative.get("run_dir") or "").strip():
            focus += f" 对应 run 目录：{representative.get('run_dir')}。"
        next_step = "下一步先看该 run 的 trace 和真实视角产物，再决定是否修抓取、运动或任务成功条件。"

    return {
        "verdict": verdict,
        "main_focus": focus,
        "next_step": next_step,
        "run_count": run_count,
        "success_count": success_count,
        "env_success_count": env_success_count,
    }


def build_markdown_report(report: Dict[str, Any]) -> str:
    """Render a human-first markdown report while preserving machine tables."""

    aggregate = dict(report.get("aggregate") or {})
    per_task = dict(aggregate.get("per_task") or {})
    stage_counts = dict(aggregate.get("failure_cluster_counts") or {})
    selected_backend_counts = dict(aggregate.get("selected_backend_counts") or {})
    selected_backend_kind_counts = dict(aggregate.get("selected_backend_kind_counts") or {})
    fallback_reason_counts = dict(aggregate.get("fallback_reason_counts") or {})
    backend_compare_state_counts = dict(aggregate.get("backend_compare_state_counts") or {})
    backend_selection_outcome_counts = dict(aggregate.get("backend_selection_outcome_counts") or {})
    fm_candidates = list(aggregate.get("fm_first_candidates") or [])
    deferred = list(report.get("skipped_entries") or [])
    human_summary = dict(report.get("human_summary") or build_suite_human_summary(report))
    lines = [
        f"# RoboTwin Multitask Suite: {report.get('suite_name', '')}",
        "",
        "## Human Summary",
        "",
        f"- {human_summary.get('verdict', '')}",
        f"- {human_summary.get('main_focus', '')}",
        f"- {human_summary.get('next_step', '')}",
        "",
        "## Machine Details",
        "",
        f"- Suite role: `{report.get('suite_role', '')}`",
        f"- Counts toward gate: `{str(bool(report.get('gate', False))).lower()}`",
        f"- Suite config: `{report.get('suite_path', '')}`",
        f"- Artifact dir: `{report.get('artifact_dir', '')}`",
        f"- Requires isolated mode: `{str(bool(report.get('require_isolated', False))).lower()}`",
        f"- Isolated subprocess mode: `{str(bool(report.get('isolated', False))).lower()}`",
        f"- Run count: `{aggregate.get('run_count', 0)}`",
        f"- Runtime success count: `{aggregate.get('success_count', 0)}`",
        f"- Env success count: `{aggregate.get('env_success_count', 0)}`",
        "",
        "## Per Task",
        "",
        "| Task | Runs | Runtime Success | Env Success | Most Common Stage | Stage Counts |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for task, summary in sorted(per_task.items()):
        lines.append(
            "| {task} | {run_count} | {success_count} | {env_success_count} | {common_stage} | `{stage_counts}` |".format(
                task=task,
                run_count=summary.get("run_count", 0),
                success_count=summary.get("success_count", 0),
                env_success_count=summary.get("env_success_count", 0),
                common_stage=summary.get("most_common_failure_stage", ""),
                stage_counts=json.dumps(summary.get("failure_stage_counts", {}), ensure_ascii=False, sort_keys=True),
            )
        )
    lines.extend(["", "## Backend Selection", "", "| Selected Backend | Count |", "| --- | --- |"])
    if selected_backend_counts:
        for backend, count in sorted(selected_backend_counts.items(), key=lambda item: (-int(item[1]), str(item[0]))):
            lines.append(f"| {backend} | {count} |")
    else:
        lines.append("| none | 0 |")

    lines.extend(["", "| Backend Kind | Count |", "| --- | --- |"])
    if selected_backend_kind_counts:
        for kind, count in sorted(selected_backend_kind_counts.items(), key=lambda item: (-int(item[1]), str(item[0]))):
            lines.append(f"| {kind} | {count} |")
    else:
        lines.append("| none | 0 |")

    lines.extend(["", "Fallback reasons:"])
    if fallback_reason_counts:
        for reason, count in sorted(fallback_reason_counts.items(), key=lambda item: (-int(item[1]), str(item[0]))):
            lines.append(f"- `{reason}`: `{count}`")
    else:
        lines.append("- None.")

    if backend_compare_state_counts:
        lines.extend(
            [
                "",
                "## FM Backend Compare",
                "",
                "| Backend | Not Ready | No Candidate | Planner Failed | Planner Feasible | Selected | Ranked Below |",
                "| --- | --- | --- | --- | --- | --- | --- |",
            ]
        )
        ordered_backends = sorted(
            backend_compare_state_counts.keys(),
            key=lambda name: (0 if name not in FALLBACK_GRASP_BACKENDS else 1, str(name)),
        )
        for backend_name in ordered_backends:
            state_counts = dict(backend_compare_state_counts.get(backend_name) or {})
            outcome_counts = dict(backend_selection_outcome_counts.get(backend_name) or {})
            lines.append(
                "| {backend} | {not_ready} | {no_candidate} | {planner_failed} | {planner_feasible} | {selected} | {ranked_below} |".format(
                    backend=backend_name,
                    not_ready=state_counts.get("backend_not_ready", 0),
                    no_candidate=state_counts.get("backend_runtime_ok_but_no_candidate", 0),
                    planner_failed=state_counts.get("backend_candidate_present_but_planner_failed", 0),
                    planner_feasible=state_counts.get("backend_candidate_planner_feasible", 0),
                    selected=outcome_counts.get("selected", 0),
                    ranked_below=outcome_counts.get("ranked_below_selected_backend", 0),
                )
            )

    def _candidate_label(compare_row: Dict[str, Any]) -> str:
        top_candidate = dict(compare_row.get("top_candidate") or {})
        return str(top_candidate.get("variant_family") or top_candidate.get("variant_label") or "").strip()

    def _matrix_cell(compare_row: Dict[str, Any] | None) -> str:
        if not compare_row:
            return "not_run"
        selection = str(compare_row.get("selection_outcome") or "").strip()
        compare_state = str(compare_row.get("compare_state") or "").strip()
        candidate_label = _candidate_label(compare_row)
        if selection == "selected":
            return f"selected:{candidate_label or 'n/a'}"
        if selection == "ranked_below_selected_backend":
            return f"below:{candidate_label or 'n/a'}"
        if selection == "planner_failed_before_selection":
            return f"planner_failed:{candidate_label or 'n/a'}"
        if selection == "no_runtime_candidates":
            return "no_candidate"
        if selection == "not_ready":
            return "not_ready"
        return compare_state or "unknown"

    matrix_rows: Dict[Tuple[str, int], Dict[str, Any]] = {}
    for row in list(report.get("runs") or []):
        compare_rows = list(row.get("inspect_backend_compare_diagnostics") or row.get("backend_compare_diagnostics") or [])
        if not compare_rows:
            continue
        task_name = str(row.get("task") or "")
        seed = int(row.get("seed") or 0)
        key = (task_name, seed)
        record = matrix_rows.setdefault(
            key,
            {"task": task_name, "seed": seed, "backend_cells": {}, "selected_by_run": {}, "depth_delegate": ""},
        )
        compare_by_backend = {
            str(compare_row.get("backend_name") or "").strip(): dict(compare_row or {})
            for compare_row in compare_rows
            if str(compare_row.get("backend_name") or "").strip()
        }
        fm_backend_names = [
            backend_name
            for backend_name, compare_row in compare_by_backend.items()
            if str(compare_row.get("backend_kind") or _backend_kind_from_name(backend_name)) == "fm_backend"
        ]
        if len(fm_backend_names) == 1:
            backend_name = fm_backend_names[0]
            record["backend_cells"][backend_name] = _matrix_cell(compare_by_backend.get(backend_name))
            record["selected_by_run"][backend_name] = str(row.get("selected_backend") or "")
        if not record["depth_delegate"]:
            record["depth_delegate"] = _matrix_cell(compare_by_backend.get("depth_synthesized"))
    if matrix_rows:
        lines.extend(
            [
                "",
                "## FM Compare Matrix",
                "",
                "| Task | Seed | Contact-GraspNet | GraspNetBaseline | GraspGen | Selected By Run | Depth Delegate |",
                "| --- | --- | --- | --- | --- | --- | --- |",
            ]
        )
        for _, record in sorted(matrix_rows.items()):
            backend_cells = dict(record.get("backend_cells") or {})
            selected_by_run = dict(record.get("selected_by_run") or {})
            selected_summary = ", ".join(
                f"{backend}:{selected or 'unknown'}" for backend, selected in sorted(selected_by_run.items())
            ) or "none"
            lines.append(
                "| {task} | {seed} | {contact} | {graspnet_baseline} | {graspgen} | {selected} | {depth_delegate} |".format(
                    task=record.get("task", ""),
                    seed=record.get("seed", 0),
                    contact=backend_cells.get("contact_graspnet", "not_run"),
                    graspnet_baseline=backend_cells.get("graspnet_baseline", "not_run"),
                    graspgen=backend_cells.get("graspgen", "not_run"),
                    selected=selected_summary,
                    depth_delegate=record.get("depth_delegate", "") or "not_run",
                )
            )

    lines.extend(["", "## Failure Clusters", "", "| Stage | Count | Representative Run |", "| --- | --- | --- |"])
    representatives = dict(aggregate.get("representative_runs_by_stage") or {})
    for stage, count in sorted(stage_counts.items(), key=lambda item: (-int(item[1]), str(item[0]))):
        rep = dict(representatives.get(stage) or {})
        rep_text = "{task} seed={seed} final={status}".format(
            task=rep.get("task", ""),
            seed=rep.get("seed", ""),
            status=rep.get("final_status", ""),
        ).strip()
        lines.append(f"| {stage} | {count} | `{rep_text}` |")
    lines.extend(["", "## Recommended FM-First Follow-Up", ""])
    if fm_candidates:
        for row in fm_candidates:
            lines.append(
                "- `{task}`: baseline failure score `{score}`, common stage `{stage}`.".format(
                    task=row.get("task", ""),
                    score=row.get("score", 0),
                    stage=row.get("most_common_failure_stage", ""),
                )
            )
    else:
        lines.append("- No strong FM-first candidate yet from the current baseline runs.")

    lines.extend(["", "## Deferred Or Unsupported", ""])
    if deferred:
        for row in deferred:
            lines.append(
                "- `{name}` ({group}, {status}): {reason}".format(
                    name=row.get("name", ""),
                    group=row.get("group", ""),
                    status=row.get("status", ""),
                    reason=row.get("reason") or row.get("deferred_reason") or "",
                )
            )
    else:
        lines.append("- None.")

    lines.extend(
        [
            "",
            "## Per Run",
            "",
            "术语说明：`Top Candidate` 是排序最靠前的抓取候选；`Executed Candidate` 是实际执行的抓取候选；`Hint` 是失败原因的机器分类提示。",
            "",
            "| Task | Seed | Mode | Contract | Probe Type | Probe Stage | Hint | Backend | Canary | Final Status | Env Success | Stage | Top Candidate | Executed Candidate | Run Dir |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        ]
    )
    for row in list(report.get("runs") or []):
        top = dict(row.get("top_candidate") or {})
        executed = dict(row.get("executed_candidate") or {})
        lines.append(
            "| {task} | {seed} | {mode} | {contract} | {probe_type} | {probe_stage} | {hint} | {backend} | {canary} | {status} | {env_success} | {stage} | {top_label} | {exec_label} | `{run_dir}` |".format(
                task=row.get("task", ""),
                seed=row.get("seed", ""),
                mode=row.get("mode", ""),
                contract=row.get("task_contract", ""),
                probe_type=row.get("probe_type", ""),
                probe_stage=row.get("probe_stage", ""),
                hint=row.get("contract_gap_hint", ""),
                backend=row.get("selected_backend", ""),
                canary=str(bool(row.get("canary", False))).lower(),
                status=row.get("final_status", ""),
                env_success=str(bool(row.get("env_success", False))).lower(),
                stage=row.get("failure_stage", ""),
                top_label=top.get("variant_family") or "",
                exec_label=executed.get("variant_family") or "",
                run_dir=row.get("run_dir", ""),
            )
        )
    compare_detail_rows: List[str] = []
    for row in list(report.get("runs") or []):
        compare_rows = list(row.get("inspect_backend_compare_diagnostics") or row.get("backend_compare_diagnostics") or [])
        for compare_row in compare_rows:
            top_candidate = dict(compare_row.get("top_candidate") or {})
            compare_detail_rows.append(
                "| {task} | {seed} | {backend} | {state} | {selection} | {top_candidate} | {planner_status} |".format(
                    task=row.get("task", ""),
                    seed=row.get("seed", ""),
                    backend=compare_row.get("backend_name", ""),
                    state=compare_row.get("compare_state", ""),
                    selection=compare_row.get("selection_outcome", ""),
                    top_candidate=top_candidate.get("variant_family") or top_candidate.get("variant_label") or "",
                    planner_status=top_candidate.get("planner_status", ""),
                )
            )
    if compare_detail_rows:
        lines.extend(
            [
                "",
                "## FM Compare Details",
                "",
                "| Task | Seed | Backend | State | Selection | Top Candidate | Planner Status |",
                "| --- | --- | --- | --- | --- | --- | --- |",
                *compare_detail_rows,
            ]
        )
    return "\n".join(lines) + "\n"
