---
name: grasp-debug-loop
description: Use when debugging manipulation failures in this repository that may involve target selection, object pose, grasp affordance semantics, grasp verification, or visual artifact review. This skill is specifically for avoiding the common trap of optimizing place/release behavior before confirming that the object was grasped in a task-compatible way.
---

# Grasp Debug Loop

This skill is for `script_policy` work where task success may be blocked by the grasp stage, even if downstream metrics look plausible.

## Read first

1. `.codex/PROMPT_RULES.md`
2. `.codex/WORKSPACE.md`
3. `.codex/MEMORY.md`
4. `.codex/ROBOTWIN_PLAN.md`
5. `.codex/OPEN_SOURCE_SCRIPT_POLICY_INDEX.md`

## Core idea

Do not assume a task is in a "place problem" just because it fails at the end.

For complex tasks, always stage-gate in this order:

1. target object grounding
2. object pose quality
3. grasp candidate generation
4. grasp affordance semantics
5. semantic grasp validation
6. only then place/release optimization

## Workflow

1. Inspect the latest run directory under `script_runtime/artifacts/...`.
2. Review visual artifacts before changing code:
   - rollout gif
   - realview contact sheet
   - key skill snapshots when available
3. Cross-check trace with the images:
   - selected grasp candidate
   - active arm
   - whether the object part being held matches task intent
4. If the object is not being grasped in a task-compatible way:
   - do not prioritize place tuning
   - first add or improve:
     - grasp affordance annotations
     - semantic grasp checks
     - candidate diagnostics
5. If the same fix pattern appears across multiple runs or tasks:
   - extract it into a registry, adapter, or planning utility
   - do not keep it as a task-local patch

## Validation

- Run targeted unit tests.
- Run at least one real RoboTwin task rollout when feasible.
- Keep visual outputs enabled for the validation run.
- Update `.codex` with:
  - the actual bottleneck
  - what visual evidence confirmed it
  - what new reusable scaffold was added
