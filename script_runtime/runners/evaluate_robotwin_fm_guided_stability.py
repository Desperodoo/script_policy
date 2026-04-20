"""Run multi-seed FM-first RoboTwin evaluations and summarize guided grasp stability."""

from __future__ import annotations

import argparse
import copy
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List

from script_runtime.session import build_robotwin_pick_place_session, load_runtime_config


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return dict(data) if isinstance(data, dict) else {}


def _trace_rows(trace_path: Path) -> List[Dict[str, Any]]:
    if not trace_path.exists():
        return []
    rows: List[Dict[str, Any]] = []
    for line in trace_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except Exception:
            continue
    return rows


def _skill_name(row: Dict[str, Any]) -> str:
    return str(row.get("skill_name") or row.get("skill") or "")


def _skill_rows(rows: Iterable[Dict[str, Any]], skill_name: str) -> List[Dict[str, Any]]:
    return [dict(row) for row in rows if _skill_name(row) == skill_name]


def _payload(row: Dict[str, Any]) -> Dict[str, Any]:
    payload = dict(row.get("inputs_summary", {}) or {}).get("payload")
    if isinstance(payload, dict):
        return dict(payload)
    payload = row.get("payload")
    return dict(payload or {}) if isinstance(payload, dict) else {}


def _normalize_candidate_label(label: Any) -> str:
    text = str(label or "")
    guided = re.match(r"contact_graspnet_guided_(c\d+)", text)
    if guided:
        return f"guided_{guided.group(1)}"
    template = re.match(r"contact_graspnet_template_(c\d+)_", text)
    if template:
        return f"template_{template.group(1)}"
    raw = re.match(r"contact_graspnet_seg\d+_\d+$", text)
    if raw:
        return "raw_contact_graspnet"
    return text or "none"


def _candidate_brief(candidate: Dict[str, Any]) -> Dict[str, Any]:
    row = dict(candidate or {})
    return {
        "variant_label": str(row.get("variant_label") or ""),
        "variant_family": _normalize_candidate_label(row.get("variant_label")),
        "proposal_backend": str(row.get("proposal_backend") or ""),
        "planner_status": str(row.get("planner_status") or ""),
        "planner_waypoint_count": row.get("planner_waypoint_count"),
        "task_compatibility": str(row.get("task_compatibility") or ""),
        "affordance_type": str(row.get("affordance_type") or ""),
        "semantic_source": str(row.get("semantic_source") or ""),
        "semantic_priority": float(row.get("semantic_priority") or 0.0),
        "score": float(row.get("score") or 0.0),
        "object_model_name": str(row.get("object_model_name") or ""),
        "object_model_id": row.get("object_model_id"),
        "contact_point_id": row.get("contact_point_id"),
        "template_contact_point_id": row.get("template_contact_point_id"),
    }


def _load_contact_graspnet_summary(run_dir: Path) -> Dict[str, Any]:
    summary_path = (
        run_dir
        / "fm_runtime"
        / "contact_graspnet"
        / "contact_graspnet_headless"
        / "contact_graspnet_summary.json"
    )
    data = _load_json(summary_path)
    groups = list(data.get("groups") or [])
    grasp_total = int(data.get("grasp_total") or sum(int(group.get("grasp_count") or 0) for group in groups))
    grasp_group_count = int(data.get("grasp_group_count") or len(groups))
    return {
        "summary_path": str(summary_path) if summary_path.exists() else "",
        "grasp_total": grasp_total,
        "grasp_group_count": grasp_group_count,
    }


def _check_grasp_brief(row: Dict[str, Any]) -> Dict[str, Any]:
    payload = _payload(row)
    report = dict(payload.get("grasp_semantic_report") or {})
    affordance = dict(report.get("affordance") or {})
    diagnostics = dict(report.get("grasp_diagnostics") or {})
    return {
        "result": str(row.get("result") or ""),
        "grasp_confirmed": bool(payload.get("grasp_confirmed", False)),
        "grasp_semantics_ok": bool(payload.get("grasp_semantics_ok", False)),
        "candidate_label": str(report.get("candidate_label") or ""),
        "candidate_family": _normalize_candidate_label(report.get("candidate_label")),
        "task_compatibility": str(report.get("task_compatibility") or ""),
        "affordance_type": str(affordance.get("affordance_type") or ""),
        "semantic_source": str(affordance.get("semantic_source") or ""),
        "semantic_priority": float(affordance.get("semantic_priority") or 0.0),
        "object_model_name": str(report.get("object_model_name") or ""),
        "contact_point_count": diagnostics.get("contact_point_count"),
        "lifted": bool(diagnostics.get("lifted", False)),
        "anchor_following": bool(diagnostics.get("anchor_following", False)),
        "is_grasped": bool(diagnostics.get("is_grasped", False)),
    }


def _extract_run_summary(*, seed: int, run_result: Any, runtime_artifacts: Dict[str, Any]) -> Dict[str, Any]:
    trace_path = Path(str(runtime_artifacts.get("trace_path") or ""))
    run_dir = Path(str(runtime_artifacts.get("run_dir") or ""))
    rows = _trace_rows(trace_path) if trace_path else []

    grasp_rows = _skill_rows(rows, "GetGraspCandidates")
    first_grasp_row = {} if not grasp_rows else grasp_rows[0]
    first_grasp_payload = _payload(first_grasp_row)
    grasp_candidates = list(first_grasp_payload.get("grasp_candidates") or [])
    top_candidate = {} if not grasp_candidates else dict(grasp_candidates[0] or {})

    execute_rows = _skill_rows(rows, "ExecuteGraspPhase")
    first_execute_row = {} if not execute_rows else execute_rows[0]
    first_execute_payload = _payload(first_execute_row)
    execute_refresh = dict(first_execute_payload.get("grasp_candidate_refresh") or {})
    executed_candidate = dict(execute_refresh.get("previous_active_candidate") or {})

    check_grasp_rows = _skill_rows(rows, "CheckGrasp")
    first_check = {} if not check_grasp_rows else check_grasp_rows[0]
    last_check = {} if not check_grasp_rows else check_grasp_rows[-1]

    task_success_rows = _skill_rows(rows, "CheckTaskSuccess")
    task_success_payload = _payload(task_success_rows[-1]) if task_success_rows else {}
    before_snapshot = dict(task_success_payload.get("before_settle_snapshot") or {})
    after_snapshot = dict(task_success_payload.get("after_settle_snapshot") or {})
    before_delta = dict(before_snapshot.get("object_to_target_center_delta") or {})
    after_delta = dict(after_snapshot.get("object_to_target_center_delta") or {})

    backend_counts = Counter(str(row.get("proposal_backend") or "unknown") for row in grasp_candidates)
    guided_labels = [
        str(row.get("variant_label") or "")
        for row in grasp_candidates
        if str(row.get("variant_label") or "").startswith("contact_graspnet_guided_")
    ]
    guided_feasible_families = sorted(
        {
            _normalize_candidate_label(row.get("variant_label"))
            for row in grasp_candidates
            if str(row.get("variant_label") or "").startswith("contact_graspnet_guided_")
            and str(row.get("planner_status") or "") == "Success"
        }
    )
    contact_graspnet_summary = _load_contact_graspnet_summary(run_dir)

    active_arm = ""
    if grasp_candidates:
        active_arm = str(grasp_candidates[0].get("arm") or "")
    if not active_arm:
        active_arm = str(runtime_artifacts.get("active_arm") or "")

    return {
        "seed": int(seed),
        "task_id": str(runtime_artifacts.get("task_id") or ""),
        "status": str(getattr(getattr(run_result, "status", None), "value", "") or getattr(run_result, "status", "") or ""),
        "message": str(getattr(run_result, "message", "") or ""),
        "active_arm": active_arm,
        "selected_backend": str(first_grasp_payload.get("selected_backend") or ""),
        "selected_backend_kind": str(first_grasp_payload.get("selected_backend_kind") or ""),
        "fallback_reason": str(first_grasp_payload.get("fallback_reason") or ""),
        "candidate_count": len(grasp_candidates),
        "backend_candidate_counts": dict(sorted(backend_counts.items())),
        "guided_candidate_labels": guided_labels,
        "guided_feasible_families": list(first_grasp_payload.get("guided_feasible_families") or guided_feasible_families),
        "has_guided_feasible_family": bool(first_grasp_payload.get("has_guided_feasible_family", bool(guided_feasible_families))),
        "grasp_candidate_stage_summary": first_grasp_payload.get("grasp_candidate_stage_summary") or {},
        "contact_graspnet_grasp_total": int(contact_graspnet_summary.get("grasp_total") or 0),
        "contact_graspnet_group_count": int(contact_graspnet_summary.get("grasp_group_count") or 0),
        "contact_graspnet_summary_path": str(contact_graspnet_summary.get("summary_path") or ""),
        "top_candidate": _candidate_brief(top_candidate),
        "top_candidates": [_candidate_brief(row) for row in grasp_candidates[:6]],
        "executed_candidate": {
            "variant_label": str(executed_candidate.get("label") or ""),
            "variant_family": _normalize_candidate_label(executed_candidate.get("label")),
            "planner_status": str(executed_candidate.get("planner_status") or ""),
            "planner_waypoint_count": executed_candidate.get("planner_waypoint_count"),
            "task_compatibility": str(executed_candidate.get("task_compatibility") or ""),
            "score": float(executed_candidate.get("score") or 0.0),
        },
        "execute_grasped": bool(first_execute_payload.get("grasped", False)),
        "execute_grasp_diagnostics": dict(first_execute_payload.get("grasp_diagnostics") or {}),
        "first_check_grasp": _check_grasp_brief(first_check) if first_check else {},
        "last_check_grasp": _check_grasp_brief(last_check) if last_check else {},
        "env_success": bool(task_success_payload.get("env_success", False)),
        "env_result": dict(task_success_payload.get("env_result") or {}),
        "before_xy_norm": before_delta.get("xy_norm"),
        "after_xy_norm": after_delta.get("xy_norm"),
        "run_dir": str(run_dir),
        "trace_path": str(trace_path),
        "realview_contact_sheet_png": str(runtime_artifacts.get("realview_contact_sheet_png") or ""),
        "rollout_gif": str(runtime_artifacts.get("rollout_gif") or ""),
        "grounding_json": str(runtime_artifacts.get("grounding_json") or ""),
    }


def _aggregate_runs(runs: List[Dict[str, Any]]) -> Dict[str, Any]:
    executed_counts = Counter()
    top_counts = Counter()
    guided_feasible_counts = Counter()
    env_success_count = 0
    by_executed: Dict[str, Dict[str, Any]] = defaultdict(
        lambda: {"count": 0, "env_success_count": 0, "post_lift_grasp_count": 0, "seeds": []}
    )
    by_top: Dict[str, Dict[str, Any]] = defaultdict(lambda: {"count": 0, "env_success_count": 0, "seeds": []})

    for row in runs:
        if row.get("error"):
            continue
        env_success = bool(row.get("env_success", False))
        if env_success:
            env_success_count += 1
        executed = str(dict(row.get("executed_candidate") or {}).get("variant_family") or "none")
        top = str(dict(row.get("top_candidate") or {}).get("variant_family") or "none")
        executed_counts[executed] += 1
        top_counts[top] += 1
        for label in list(row.get("guided_feasible_families") or []):
            guided_feasible_counts[str(label)] += 1
        by_executed[executed]["count"] += 1
        by_executed[executed]["seeds"].append(int(row.get("seed", -1)))
        by_top[top]["count"] += 1
        by_top[top]["seeds"].append(int(row.get("seed", -1)))
        if env_success:
            by_executed[executed]["env_success_count"] += 1
            by_top[top]["env_success_count"] += 1
        last_check = dict(row.get("last_check_grasp") or {})
        if bool(last_check.get("grasp_confirmed", False)) and bool(last_check.get("lifted", False)):
            by_executed[executed]["post_lift_grasp_count"] += 1

    return {
        "run_count": len(runs),
        "successful_env_count": env_success_count,
        "top_variant_counts": dict(top_counts),
        "executed_variant_counts": dict(executed_counts),
        "guided_feasible_variant_counts": dict(guided_feasible_counts),
        "by_top_variant": dict(by_top),
        "by_executed_variant": dict(by_executed),
    }


def _build_markdown(report: Dict[str, Any]) -> str:
    lines = [
        "# FM Guided Seed Sweep",
        "",
        f"- Config: `{report['config_path']}`",
        f"- Seeds: `{', '.join(str(seed) for seed in report['seeds'])}`",
        f"- Runs: `{report['aggregate']['run_count']}`",
        f"- Env success: `{report['aggregate']['successful_env_count']}`",
        "",
        "## Per Seed",
        "",
        "| Seed | Arm | Model | Top | Executed | Top Backend | Backend Kind | Guided Feasible | CGN Grasps | Env Success | Lift Check | After XY | Run Dir |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in report["runs"]:
        if row.get("error"):
            lines.append(
                f"| {row['seed']} |  |  | error | error |  |  |  | 0 | false | false |  | `{row.get('error')}` |"
            )
            continue
        top = dict(row.get("top_candidate") or {})
        executed = dict(row.get("executed_candidate") or {})
        last_check = dict(row.get("last_check_grasp") or {})
        lines.append(
            "| {seed} | {arm} | {model} | {top_label} | {exec_label} | {backend} | {backend_kind} | {guided} | {cgn_grasps} | {env_success} | {lift_ok} | {after_xy} | `{run_dir}` |".format(
                seed=row.get("seed"),
                arm=row.get("active_arm") or "",
                model=top.get("object_model_name") or last_check.get("object_model_name") or "",
                top_label=top.get("variant_family") or "",
                exec_label=executed.get("variant_family") or "",
                backend=row.get("selected_backend") or "",
                backend_kind=row.get("selected_backend_kind") or "",
                guided=",".join(list(row.get("guided_feasible_families") or [])),
                cgn_grasps=row.get("contact_graspnet_grasp_total") or 0,
                env_success=str(bool(row.get("env_success", False))).lower(),
                lift_ok=str(bool(last_check.get("grasp_confirmed", False)) and bool(last_check.get("lifted", False))).lower(),
                after_xy="" if row.get("after_xy_norm") is None else f"{float(row['after_xy_norm']):.4f}",
                run_dir=row.get("run_dir") or "",
            )
        )
    lines.extend(
        [
            "",
            "## Aggregates",
            "",
            f"- Top variant counts: `{json.dumps(report['aggregate']['top_variant_counts'], ensure_ascii=False, sort_keys=True)}`",
            f"- Executed variant counts: `{json.dumps(report['aggregate']['executed_variant_counts'], ensure_ascii=False, sort_keys=True)}`",
            f"- Guided feasible counts: `{json.dumps(report['aggregate']['guided_feasible_variant_counts'], ensure_ascii=False, sort_keys=True)}`",
            "",
            "## By Executed Variant",
            "",
        ]
    )
    for label, row in sorted(report["aggregate"]["by_executed_variant"].items()):
        lines.append(
            "- `{label}`: runs=`{count}`, env_success=`{env_success}`, post_lift_grasp=`{post_lift}`, seeds=`{seeds}`".format(
                label=label,
                count=row.get("count", 0),
                env_success=row.get("env_success_count", 0),
                post_lift=row.get("post_lift_grasp_count", 0),
                seeds=",".join(str(seed) for seed in row.get("seeds", [])),
            )
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate FM-guided RoboTwin grasp stability across seeds.")
    parser.add_argument(
        "--config",
        default="script_runtime/configs/tasks/place_container_plate_robotwin_fm_first.yaml",
        help="Path to the runtime YAML config.",
    )
    parser.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4, 5, 6],
        help="RoboTwin seeds to evaluate.",
    )
    parser.add_argument(
        "--task-id-prefix",
        default="fm_guided_seed_sweep",
        help="Prefix for per-seed task ids and the aggregate report directory.",
    )
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="Disable RoboTwin frame capture during seed sweeps.",
    )
    args = parser.parse_args()

    base_config = load_runtime_config(args.config)
    runtime = dict(base_config.get("runtime") or {})
    artifact_root = Path(str(runtime.get("artifact_dir") or "script_runtime/artifacts")).expanduser()
    summary_dir = artifact_root / str(args.task_id_prefix)
    summary_dir.mkdir(parents=True, exist_ok=True)

    runs: List[Dict[str, Any]] = []
    for seed in list(args.seeds):
        config = copy.deepcopy(base_config)
        runtime_cfg = config.setdefault("runtime", {})
        robotwin_cfg = config.setdefault("robotwin", {})
        runtime_cfg["task_id"] = f"{args.task_id_prefix}_seed{seed}"
        robotwin_cfg["seed"] = int(seed)
        if args.no_video:
            robotwin_cfg["capture_video"] = False
        session = build_robotwin_pick_place_session(config)
        try:
            try:
                result = session.run()
                runs.append(
                    _extract_run_summary(
                        seed=int(seed),
                        run_result=result,
                        runtime_artifacts=dict(session.runtime_artifacts or {}),
                    )
                )
            except Exception as exc:
                runs.append(
                    {
                        "seed": int(seed),
                        "task_id": runtime_cfg["task_id"],
                        "error": repr(exc),
                        "run_dir": str(dict(session.runtime_artifacts or {}).get("run_dir") or ""),
                    }
                )
        finally:
            session.shutdown()

    report = {
        "config_path": str(Path(args.config).expanduser()),
        "seeds": [int(seed) for seed in list(args.seeds)],
        "task_id_prefix": str(args.task_id_prefix),
        "runs": runs,
        "aggregate": _aggregate_runs(runs),
    }
    summary_json = summary_dir / f"{args.task_id_prefix}_summary.json"
    summary_md = summary_dir / f"{args.task_id_prefix}_summary.md"
    summary_json.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    summary_md.write_text(_build_markdown(report), encoding="utf-8")
    print(json.dumps({"summary_json": str(summary_json), "summary_md": str(summary_md), "report": report}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
