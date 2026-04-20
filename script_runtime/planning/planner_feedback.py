"""Planner feedback normalization and ranking helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class PlannerEvaluatedVariant:
    label: str
    pose: Optional[List[float]]
    planner_status: str = "Unknown"
    planner_waypoint_count: Optional[int] = None
    planner_debug: Optional[Dict[str, Any]] = None
    score: float = 0.0
    score_adjust: float = 0.0
    score_metrics: Optional[Dict[str, Any]] = None


def normalize_planner_statuses(raw_status: Any, expected_count: int) -> List[str]:
    if raw_status is None:
        return ["Unknown"] * expected_count

    if isinstance(raw_status, str):
        return [raw_status] * expected_count

    try:
        status_arr = np.asarray(raw_status, dtype=object)
    except Exception:
        try:
            status_list = list(raw_status)
        except Exception:
            return [str(raw_status)] * expected_count
        return normalize_planner_statuses(status_list, expected_count)

    if status_arr.ndim == 0:
        return [str(status_arr.item())] * expected_count

    status_list = [str(item) for item in status_arr.reshape(-1).tolist()]
    if len(status_list) == 1 and expected_count > 1:
        return status_list * expected_count
    if 1 < len(status_list) < expected_count:
        unique_statuses = {item for item in status_list}
        if len(unique_statuses) == 1:
            only = next(iter(unique_statuses))
            if only in {"Failure", "Fail", "Success", "Unavailable"}:
                return status_list + [only] * (expected_count - len(status_list))
    if len(status_list) < expected_count:
        return status_list + ["Unknown"] * (expected_count - len(status_list))
    return status_list[:expected_count]


def evaluate_and_rank_pose_variants(
    variants: List[Any],
    sdk: Any | None,
    *,
    kind: str,
) -> List[PlannerEvaluatedVariant]:
    """Evaluate candidate poses with the planner and return a ranked list.

    The caller passes in objects that expose `.label` and `.pose` attributes.
    This keeps the helper usable for multiple candidate-family builders without
    coupling it to a specific dataclass implementation.
    """

    ranked: List[PlannerEvaluatedVariant] = []
    valid_rows: List[tuple[int, str, List[float]]] = []
    for index, variant in enumerate(variants):
        label = str(getattr(variant, "label", f"candidate_{index}"))
        pose = getattr(variant, "pose", None)
        base_score = -0.01 * index
        if pose is None:
            ranked.append(
                PlannerEvaluatedVariant(
                    label=label,
                    pose=None,
                    planner_status="Unavailable",
                    score=base_score - 5.0,
                )
            )
            continue
        pose_list = [float(v) for v in list(pose)[:7]]
        score_adjust = 0.0
        score_metrics = None
        if sdk is not None and hasattr(sdk, "score_pose_candidate"):
            try:
                extra = dict(sdk.score_pose_candidate(pose_list, kind=kind) or {})
            except Exception as exc:
                extra = {"score_adjust": 0.0, "score_metrics": {"score_error": repr(exc)}}
            score_adjust = float(extra.get("score_adjust", 0.0))
            score_metrics = dict(extra)
            score_metrics.pop("score_adjust", None)
        ranked.append(
            PlannerEvaluatedVariant(
                label=label,
                pose=pose_list,
                score=base_score + score_adjust,
                score_adjust=score_adjust,
                score_metrics=score_metrics,
            )
        )
        valid_rows.append((index, label, pose_list))

    if not valid_rows:
        return ranked

    if sdk is None or not hasattr(sdk, "evaluate_pose_candidates"):
        return sorted(ranked, key=lambda row: row.score, reverse=True)

    pose_inputs = [row[2] for row in valid_rows]
    try:
        evaluations = sdk.evaluate_pose_candidates(pose_inputs, kind=kind)
    except Exception as exc:
        return [
            PlannerEvaluatedVariant(
                label=row.label,
                pose=row.pose,
                planner_status="Error" if row.pose is not None else row.planner_status,
                planner_debug={"message": repr(exc)} if row.pose is not None else row.planner_debug,
                score=row.score - (0.6 if row.pose is not None else 0.0),
            )
            for row in ranked
        ]

    by_index: Dict[int, PlannerEvaluatedVariant] = {index: row for index, row in enumerate(ranked)}
    for eval_row, (original_index, label, pose_list) in zip(evaluations, valid_rows):
        status = str(eval_row.get("status", "Unknown"))
        waypoint_count = eval_row.get("waypoint_count")
        planner_debug = eval_row.get("planner_debug")
        score = float(by_index[original_index].score)
        if status == "Success":
            score += 2.0
            if waypoint_count is not None:
                score -= min(float(waypoint_count), 400.0) / 1000.0
        elif status in {"Failure", "Fail"}:
            score -= 1.0
        elif status in {"Unavailable", "Error"}:
            score -= 0.4
        else:
            score -= 0.15
        by_index[original_index] = PlannerEvaluatedVariant(
            label=label,
            pose=pose_list,
            planner_status=status,
            planner_waypoint_count=None if waypoint_count is None else int(waypoint_count),
            planner_debug=planner_debug,
            score=score,
            score_adjust=by_index[original_index].score_adjust,
            score_metrics=by_index[original_index].score_metrics,
        )

    return sorted(by_index.values(), key=lambda row: row.score, reverse=True)
