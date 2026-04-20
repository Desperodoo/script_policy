"""Default heuristic place module.

This preserves the current candidate-ranking behavior while moving the
implementation behind a pluggable module boundary. Future modules can replace
this with closed-loop alignment or learned local policies.
"""

from __future__ import annotations

from typing import Any, Dict, List

from script_runtime.core.failure_codes import FailureCode
from script_runtime.core.result_types import SkillResult
from script_runtime.core.skill_base import SkillContext, request_world_refresh
from script_runtime.planning import (
    build_arm_aware_release_candidates,
    build_blended_release_candidates,
    evaluate_and_rank_pose_variants,
)


def _trace_snapshot(sdk: Any, label: str) -> Dict[str, Any]:
    if hasattr(sdk, "get_trace_snapshot"):
        try:
            return dict(sdk.get_trace_snapshot(label=label) or {})
        except Exception as exc:
            return {"label": label, "snapshot_error": repr(exc)}
    return {"label": label}


def _active_arm(sdk: Any) -> str:
    if hasattr(sdk, "_active_arm"):
        try:
            return str(sdk._active_arm())
        except Exception:
            return "right"
    return "right"


def _ranking_rows(rows: List[Any], *, limit: int, include_debug: bool = False) -> List[Dict[str, Any]]:
    ranking: List[Dict[str, Any]] = []
    for row in rows[:limit]:
        record = {
            "label": row.label,
            "planner_status": row.planner_status,
            "planner_waypoint_count": row.planner_waypoint_count,
            "planner_score": row.score,
            "score_adjust": row.score_adjust,
            "score_metrics": row.score_metrics,
        }
        if include_debug:
            record["planner_debug"] = row.planner_debug
        ranking.append(record)
    return ranking


class HeuristicPlaceModule:
    name = "heuristic_place_module"

    def execute_place_approach(
        self,
        context: SkillContext,
        *,
        skill: Any,
        sdk: Any,
        target_pose: list[float],
    ) -> SkillResult:
        release_pose = context.blackboard.get("place_release_pose")
        approach_candidates = build_blended_release_candidates(
            base_target=target_pose,
            release_pose=release_pose,
            approach_pose=context.blackboard.get("active_place_approach_pose") or context.blackboard.get("place_pose"),
            active_arm=_active_arm(sdk),
        )
        ranked_candidates = evaluate_and_rank_pose_variants(approach_candidates, sdk, kind="place_approach")
        attempted = []
        for variant in ranked_candidates:
            label, candidate = variant.label, variant.pose
            if candidate is None:
                continue
            result = sdk.move_l(candidate, speed=skill.speed)
            attempted.append(
                {
                    "label": label,
                    "pose": candidate,
                    "ok": bool(result.get("ok", False)),
                    "planner_status": variant.planner_status,
                    "planner_waypoint_count": variant.planner_waypoint_count,
                    "planner_debug": variant.planner_debug,
                    "planner_score": variant.score,
                }
            )
            if result.get("ok", False):
                context.blackboard.set("active_place_approach_pose", candidate)
                request_world_refresh(context, sdk, reason="post_PlaceApproach")
                return SkillResult.success(
                    command=result["command"],
                    place_module=self.name,
                    fallback_used=label if label != "primary" else "",
                    planner_status=variant.planner_status,
                    planner_waypoint_count=variant.planner_waypoint_count,
                    planner_debug=variant.planner_debug,
                    candidate_ranking=_ranking_rows(ranked_candidates, limit=10),
                )

        return SkillResult.failure(
            FailureCode.SDK_ERROR,
            message="move_l failed",
            place_module=self.name,
            sdk_result={
                "attempted_targets": attempted,
                "candidate_ranking": _ranking_rows(ranked_candidates, limit=12, include_debug=True),
            },
        )

    def execute_place_release(
        self,
        context: SkillContext,
        *,
        skill: Any,
        sdk: Any,
        target_pose: list[float],
    ) -> SkillResult:
        release_state_before = _trace_snapshot(sdk, "place_release_before")
        approach_pose = context.blackboard.get("active_place_approach_pose") or context.blackboard.get("place_pose")
        release_candidates = build_arm_aware_release_candidates(
            base_target=target_pose,
            approach_pose=approach_pose,
            active_arm=_active_arm(sdk),
        )
        ranked_candidates = evaluate_and_rank_pose_variants(release_candidates, sdk, kind="place_release")
        attempted = []
        for variant in ranked_candidates:
            label, candidate = variant.label, variant.pose
            if candidate is None:
                continue
            result = sdk.move_l(candidate, speed=skill.speed)
            attempted.append(
                {
                    "label": label,
                    "pose": candidate,
                    "ok": bool(result.get("ok", False)),
                    "planner_status": variant.planner_status,
                    "planner_waypoint_count": variant.planner_waypoint_count,
                    "planner_debug": variant.planner_debug,
                    "planner_score": variant.score,
                }
            )
            if result.get("ok", False):
                context.blackboard.set("active_place_release_pose", candidate)
                request_world_refresh(context, sdk, reason="post_PlaceRelease")
                return SkillResult.success(
                    command=result["command"],
                    place_module=self.name,
                    fallback_used=label if label != "primary" else "",
                    planner_status=variant.planner_status,
                    planner_waypoint_count=variant.planner_waypoint_count,
                    planner_debug=variant.planner_debug,
                    candidate_ranking=_ranking_rows(ranked_candidates, limit=10),
                    release_state_before=release_state_before,
                    release_state_after=_trace_snapshot(sdk, "place_release_after"),
                )

        return SkillResult.failure(
            FailureCode.SDK_ERROR,
            message="move_l failed",
            place_module=self.name,
            sdk_result={
                "attempted_targets": attempted,
                "candidate_ranking": _ranking_rows(ranked_candidates, limit=12, include_debug=True),
            },
            release_state_before=release_state_before,
        )
