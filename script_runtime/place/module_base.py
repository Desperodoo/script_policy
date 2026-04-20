"""Pluggable place-module interface for approach / release execution."""

from __future__ import annotations

from typing import Any, Protocol

from script_runtime.core.result_types import SkillResult
from script_runtime.core.skill_base import SkillContext


class PlaceModule(Protocol):
    """Interface for place-stage execution backends.

    A module owns the implementation details of place approach / release.
    This keeps the high-level task tree stable while allowing us to swap in:
    - heuristic candidate ranking
    - closed-loop visual alignment
    - learned residual / local policy execution
    """

    name: str

    def execute_place_approach(
        self,
        context: SkillContext,
        *,
        skill: Any,
        sdk: Any,
        target_pose: list[float],
    ) -> SkillResult:
        raise NotImplementedError

    def execute_place_release(
        self,
        context: SkillContext,
        *,
        skill: Any,
        sdk: Any,
        target_pose: list[float],
    ) -> SkillResult:
        raise NotImplementedError


def resolve_place_module(context: SkillContext) -> PlaceModule:
    module = context.adapters.get("place_module")
    if module is not None:
        return module
    from .heuristic import HeuristicPlaceModule

    return HeuristicPlaceModule()
