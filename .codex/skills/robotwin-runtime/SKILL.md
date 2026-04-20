---
name: robotwin-runtime
description: Use when working on RoboTwin or robotwin2.0 integration in this repository, including environment setup, dependency checks, bridge design, runtime adapter implementation, rollout wiring, and mapping RoboTwin task semantics into script_runtime world state, skills, and success checks.
---

# Robotwin Runtime

This skill is for `script_policy` repository work that touches RoboTwin setup or integration.

## Goals

- Keep `script_runtime` as the execution core
- Treat RoboTwin as the primary simulation validation environment
- Keep `arm_control_sdk` as the real-robot boundary
- Avoid re-coupling the runtime to ROS deployment code

## Workflow

1. Read `.codex/PROMPT_RULES.md`, `.codex/WORKSPACE.md`, `.codex/MEMORY.md`, and `.codex/ROBOTWIN_PLAN.md`.
2. Read `.codex/OPEN_SOURCE_SCRIPT_POLICY_INDEX.md` before implementing a feature from scratch.
3. Inspect `third_party/RoboTwin` before making assumptions about APIs or install steps.
4. Prefer adapting RoboTwin into `script_runtime/adapters/robotwin_bridge.py` instead of changing runtime core abstractions.
5. Map RoboTwin state into existing runtime contracts first:
   - object pose
   - goal/place pose
   - grasp state
   - robot status
   - task success
6. Keep ManiSkill code only as reference for bridge structure and validation patterns.
7. For complex task failures, do not skip the grasp semantics stage:
   - candidate affordance
   - task compatibility
   - semantic grasp validation
8. When a real rollout is available, review visual artifacts before concluding which subsystem is broken.

## Implementation guidance

- New sim integration code should live under `script_runtime/`.
- Put simulator-specific logic in adapters and validation helpers, not in generic skills when possible.
- If RoboTwin exposes richer semantics, prefer extending blackboard fields in a backward-compatible way.
- Preserve trace output and failure code behavior.
- For any sub-feature such as behavior tree, planning, grasp proposal, or grounding, check the open-source index first and prefer reuse/reference over reimplementation.
- If a failure might be caused by grasping the wrong object part, do not optimize place/release first. Add or improve affordance semantics and semantic grasp checks before tuning downstream modules.

## Validation guidance

- First validate environment import and one official task.
- Then validate a minimal runtime session with stubbed or direct bridge calls.
- Then add a real rollout runner.
- Keep outputs inspectable with JSON, trace, and rendered media.
- For complex RoboTwin tasks, the validation loop is incomplete unless at least one real-view artifact was reviewed.
