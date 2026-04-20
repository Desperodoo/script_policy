"""Closed-loop place module built on object-target alignment feedback."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np

from script_runtime.core.result_types import SkillResult, SkillStatus
from script_runtime.core.skill_base import SkillContext, request_world_refresh

from .heuristic import HeuristicPlaceModule, _trace_snapshot


def _delta_from_snapshot(snapshot: Dict[str, Any]) -> Optional[Dict[str, float]]:
    delta = snapshot.get("object_to_target_center_delta")
    if not isinstance(delta, dict):
        return None
    try:
        dx = float(delta.get("dx", 0.0))
        dy = float(delta.get("dy", 0.0))
        dz = float(delta.get("dz", 0.0))
        xy_norm = float(delta.get("xy_norm", (dx * dx + dy * dy) ** 0.5))
    except Exception:
        return None
    return {
        "dx": dx,
        "dy": dy,
        "dz": dz,
        "xy_norm": xy_norm,
    }


def _clip(value: float, limit: float) -> float:
    limit = abs(float(limit))
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return float(value)


def _clip_xy(vec_xy: np.ndarray, limit: float) -> np.ndarray:
    limit = abs(float(limit))
    vec = np.asarray(vec_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(vec))
    if norm <= limit or norm < 1e-9:
        return vec
    return vec * (limit / norm)


def _selected_candidate_metrics(payload: Dict[str, Any]) -> Dict[str, Any]:
    ranking = list(payload.get("candidate_ranking") or [])
    selected_label = str(payload.get("fallback_used") or "primary")
    for row in ranking:
        if str(row.get("label", "")) != selected_label:
            continue
        metrics = ((row.get("score_metrics") or {}).get("predicted_object_to_target_center_delta") or {})
        if isinstance(metrics, dict):
            return metrics
    return {}


@dataclass
class AdaptiveCorrectionModel:
    response_matrix: np.ndarray
    response_ridge: float = 0.06
    response_ema: float = 0.6
    response_diag_min: float = 0.03
    response_diag_max: float = 0.75
    response_cross_limit: float = 0.25

    @classmethod
    def from_payload(
        cls,
        payload: Dict[str, Any],
        *,
        response_prior: float,
        response_ridge: float,
        response_ema: float,
        response_diag_min: float,
        response_diag_max: float,
        response_cross_limit: float,
    ) -> "AdaptiveCorrectionModel":
        metrics = _selected_candidate_metrics(payload)
        seeded = float(metrics.get("realized_correction_fraction", metrics.get("transport_confidence", response_prior)))
        seeded = float(np.clip(seeded, response_diag_min, response_diag_max))
        return cls(
            response_matrix=np.eye(2, dtype=np.float64) * seeded,
            response_ridge=float(response_ridge),
            response_ema=float(response_ema),
            response_diag_min=float(response_diag_min),
            response_diag_max=float(response_diag_max),
            response_cross_limit=float(response_cross_limit),
        )

    def command_xy(self, delta: Dict[str, float], *, xy_step_limit: float) -> tuple[np.ndarray, Dict[str, Any]]:
        error_xy = np.asarray([float(delta["dx"]), float(delta["dy"])], dtype=np.float64)
        desired_error_delta = -error_xy
        regularized = self.response_matrix + np.eye(2, dtype=np.float64) * float(self.response_ridge)
        try:
            raw_command = np.linalg.solve(regularized, desired_error_delta)
        except np.linalg.LinAlgError:
            raw_command = desired_error_delta
        clipped_command = _clip_xy(raw_command, xy_step_limit)
        predicted_error_delta = self.response_matrix @ clipped_command
        predicted_after_error = error_xy + predicted_error_delta
        return clipped_command, {
            "error_xy": error_xy.tolist(),
            "desired_error_delta": desired_error_delta.tolist(),
            "raw_command_xy": raw_command.tolist(),
            "clipped_command_xy": clipped_command.tolist(),
            "predicted_error_delta": predicted_error_delta.tolist(),
            "predicted_after_error": predicted_after_error.tolist(),
        }

    def update(self, *, commanded_xy: np.ndarray, before_delta: Dict[str, float], after_delta: Dict[str, float]) -> Dict[str, Any]:
        u = np.asarray(commanded_xy, dtype=np.float64).reshape(2)
        before_xy = np.asarray([float(before_delta["dx"]), float(before_delta["dy"])], dtype=np.float64)
        after_xy = np.asarray([float(after_delta["dx"]), float(after_delta["dy"])], dtype=np.float64)
        observed_error_delta = after_xy - before_xy
        denom = float(np.dot(u, u))
        if denom < 1e-9:
            return {
                "observed_error_delta": observed_error_delta.tolist(),
                "response_matrix_after": self.response_matrix.tolist(),
                "response_update_skipped": True,
            }
        sample = np.outer(observed_error_delta, u) / denom
        sample = np.asarray(sample, dtype=np.float64)
        sample[0, 0] = float(np.clip(sample[0, 0], self.response_diag_min, self.response_diag_max))
        sample[1, 1] = float(np.clip(sample[1, 1], self.response_diag_min, self.response_diag_max))
        sample[0, 1] = float(np.clip(sample[0, 1], -self.response_cross_limit, self.response_cross_limit))
        sample[1, 0] = float(np.clip(sample[1, 0], -self.response_cross_limit, self.response_cross_limit))
        self.response_matrix = self.response_ema * self.response_matrix + (1.0 - self.response_ema) * sample
        self.response_matrix[0, 0] = float(np.clip(self.response_matrix[0, 0], self.response_diag_min, self.response_diag_max))
        self.response_matrix[1, 1] = float(np.clip(self.response_matrix[1, 1], self.response_diag_min, self.response_diag_max))
        self.response_matrix[0, 1] = float(np.clip(self.response_matrix[0, 1], -self.response_cross_limit, self.response_cross_limit))
        self.response_matrix[1, 0] = float(np.clip(self.response_matrix[1, 0], -self.response_cross_limit, self.response_cross_limit))
        return {
            "observed_error_delta": observed_error_delta.tolist(),
            "response_sample": sample.tolist(),
            "response_matrix_after": self.response_matrix.tolist(),
        }


class ClosedLoopPlaceModule:
    name = "closed_loop_place_module"

    def __init__(
        self,
        *,
        fallback_module: HeuristicPlaceModule | None = None,
        max_alignment_steps: int = 3,
        xy_gain: float = 0.45,
        z_gain: float = 0.2,
        xy_step_limit: float = 0.03,
        z_step_limit: float = 0.01,
        alignment_speed: float = 0.12,
        target_xy_tolerance: float = 0.03,
        target_z_tolerance: float = 0.015,
        min_xy_improvement: float = 0.0001,
        max_xy_worsening: float = 0.002,
        response_prior: float = 0.18,
        response_ridge: float = 0.06,
        response_ema: float = 0.6,
        response_diag_min: float = 0.03,
        response_diag_max: float = 0.75,
        response_cross_limit: float = 0.25,
    ):
        self.fallback_module = fallback_module or HeuristicPlaceModule()
        self.max_alignment_steps = int(max_alignment_steps)
        self.xy_gain = float(xy_gain)
        self.z_gain = float(z_gain)
        self.xy_step_limit = float(xy_step_limit)
        self.z_step_limit = float(z_step_limit)
        self.alignment_speed = float(alignment_speed)
        self.target_xy_tolerance = float(target_xy_tolerance)
        self.target_z_tolerance = float(target_z_tolerance)
        self.min_xy_improvement = float(min_xy_improvement)
        self.max_xy_worsening = float(max_xy_worsening)
        self.response_prior = float(response_prior)
        self.response_ridge = float(response_ridge)
        self.response_ema = float(response_ema)
        self.response_diag_min = float(response_diag_min)
        self.response_diag_max = float(response_diag_max)
        self.response_cross_limit = float(response_cross_limit)

    def execute_place_approach(
        self,
        context: SkillContext,
        *,
        skill: Any,
        sdk: Any,
        target_pose: list[float],
    ) -> SkillResult:
        result = self.fallback_module.execute_place_approach(context, skill=skill, sdk=sdk, target_pose=target_pose)
        self._annotate_result(result)
        result.payload.setdefault("closed_loop_status", "approach_passthrough")
        return result

    def execute_place_release(
        self,
        context: SkillContext,
        *,
        skill: Any,
        sdk: Any,
        target_pose: list[float],
    ) -> SkillResult:
        result = self.fallback_module.execute_place_release(context, skill=skill, sdk=sdk, target_pose=target_pose)
        self._annotate_result(result)
        if result.status is not SkillStatus.SUCCESS:
            result.payload.setdefault("closed_loop_status", "baseline_release_failed")
            return result

        baseline_after = dict(result.payload.get("release_state_after") or _trace_snapshot(sdk, "place_release_after_baseline"))
        result.payload["baseline_release_state_after"] = baseline_after
        alignment_steps = []
        final_snapshot = baseline_after
        closed_loop_status = "skipped_missing_feedback"
        best_snapshot = baseline_after
        best_delta = _delta_from_snapshot(best_snapshot)
        best_pose = list((best_snapshot.get("eef_pose") or context.blackboard.get("active_place_release_pose") or target_pose)[:7])
        correction_model = AdaptiveCorrectionModel.from_payload(
            result.payload,
            response_prior=self.response_prior,
            response_ridge=self.response_ridge,
            response_ema=self.response_ema,
            response_diag_min=self.response_diag_min,
            response_diag_max=self.response_diag_max,
            response_cross_limit=self.response_cross_limit,
        )
        result.payload["correction_model_seed"] = correction_model.response_matrix.tolist()

        for step_idx in range(max(self.max_alignment_steps, 0)):
            before = _trace_snapshot(sdk, f"closed_loop_release_step_{step_idx}_before")
            before_delta = _delta_from_snapshot(before)
            if before_delta is None:
                closed_loop_status = "missing_alignment_feedback"
                break
            if (
                before_delta["xy_norm"] <= self.target_xy_tolerance
                and abs(before_delta["dz"]) <= self.target_z_tolerance
            ):
                final_snapshot = before
                closed_loop_status = "already_aligned"
                break

            eef_pose = list(before.get("eef_pose") or [])
            if len(eef_pose) < 7:
                closed_loop_status = "missing_eef_pose"
                break

            command_pose = list(eef_pose)
            command_xy, prediction = correction_model.command_xy(before_delta, xy_step_limit=self.xy_step_limit)
            command_pose[0] += float(command_xy[0])
            command_pose[1] += float(command_xy[1])
            command_pose[2] += _clip(-self.z_gain * before_delta["dz"], self.z_step_limit)

            move_result = sdk.move_l(command_pose, speed=min(skill.speed, self.alignment_speed))
            step_record: Dict[str, Any] = {
                "step_index": step_idx,
                "before_snapshot": before,
                "before_delta": before_delta,
                "command_pose": command_pose,
                "command_delta_xy": command_xy.tolist(),
                "response_matrix_before": correction_model.response_matrix.tolist(),
                "prediction": prediction,
                "move_ok": bool(move_result.get("ok", False)),
                "command": move_result.get("command"),
            }
            if not move_result.get("ok", False):
                alignment_steps.append(step_record)
                closed_loop_status = "alignment_move_failed"
                break

            request_world_refresh(context, sdk, reason=f"post_closed_loop_release_step_{step_idx}")
            context.blackboard.set("active_place_release_pose", command_pose)

            after = _trace_snapshot(sdk, f"closed_loop_release_step_{step_idx}_after")
            after_delta = _delta_from_snapshot(after)
            step_record["after_snapshot"] = after
            step_record["after_delta"] = after_delta
            if before_delta is not None and after_delta is not None:
                step_record["xy_improvement"] = float(before_delta["xy_norm"] - after_delta["xy_norm"])
                step_record.update(
                    correction_model.update(
                        commanded_xy=command_xy,
                        before_delta=before_delta,
                        after_delta=after_delta,
                    )
                )
            alignment_steps.append(step_record)
            final_snapshot = after

            if after_delta is None:
                closed_loop_status = "missing_post_alignment_feedback"
                break
            if best_delta is not None and after_delta["xy_norm"] > best_delta["xy_norm"] + self.max_xy_worsening:
                revert_result = sdk.move_l(best_pose, speed=min(skill.speed, self.alignment_speed))
                step_record["revert_ok"] = bool(revert_result.get("ok", False))
                step_record["revert_command"] = revert_result.get("command")
                if step_record["revert_ok"]:
                    request_world_refresh(context, sdk, reason=f"post_closed_loop_release_revert_{step_idx}")
                    context.blackboard.set("active_place_release_pose", best_pose)
                    revert_snapshot = _trace_snapshot(sdk, f"closed_loop_release_step_{step_idx}_revert")
                    step_record["revert_snapshot"] = revert_snapshot
                    final_snapshot = revert_snapshot
                    closed_loop_status = "alignment_worsened_reverted"
                else:
                    closed_loop_status = "alignment_worsened_revert_failed"
                break
            if (
                after_delta["xy_norm"] <= self.target_xy_tolerance
                and abs(after_delta["dz"]) <= self.target_z_tolerance
            ):
                best_snapshot = after
                best_delta = after_delta
                best_pose = list(command_pose)
                closed_loop_status = "aligned"
                break
            if best_delta is None or after_delta["xy_norm"] < best_delta["xy_norm"]:
                best_snapshot = after
                best_delta = after_delta
                best_pose = list(command_pose)
            if step_record.get("xy_improvement", 0.0) <= self.min_xy_improvement:
                closed_loop_status = "alignment_stalled"
                break
        else:
            closed_loop_status = "max_alignment_steps_reached"

        result.payload["alignment_steps"] = alignment_steps
        result.payload["closed_loop_status"] = closed_loop_status
        result.payload["correction_model_final"] = correction_model.response_matrix.tolist()
        result.payload["release_state_after"] = final_snapshot
        final_delta = _delta_from_snapshot(final_snapshot)
        if final_delta is not None:
            result.payload["final_alignment_delta"] = final_delta
        return result

    def _annotate_result(self, result: SkillResult) -> None:
        result.payload["place_module"] = self.name
        result.payload.setdefault("baseline_place_module", getattr(self.fallback_module, "name", "heuristic_place_module"))
