"""Shared helpers for complex probe task trees."""

from __future__ import annotations

# Complex probe suites prioritize stable, classifiable execution over tight
# wall-clock gating. Their motion and refresh steps can legitimately take
# longer than the place-only baseline, especially for dual-arm and articulated
# tasks that refresh richer candidate sets between stages.
PROBE_PREPARE_GRIPPER_TIMEOUT_S = 30.0
PROBE_GO_PREGRASP_TIMEOUT_S = 20.0
PROBE_EXECUTE_GRASP_TIMEOUT_S = 20.0
