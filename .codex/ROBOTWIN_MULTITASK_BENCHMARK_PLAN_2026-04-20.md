# RoboTwin Multitask Benchmark Notes

## What Was Added

- A batch runner: `script_runtime/runners/evaluate_robotwin_multitask_suite.py`
- A default place-only baseline suite:
  `script_runtime/configs/robotwin_multitask_place_suite.yaml`
- Three new config-first RoboTwin tasks:
  - `place_phone_stand_robotwin.yaml`
  - `place_shoe_robotwin.yaml`
  - `place_object_stand_robotwin.yaml`
- Unit tests for:
  - suite expansion
  - failure-stage taxonomy
  - FM-first optional summary fields
  - runner exception isolation

## Current Scope

- First round is still `PickPlaceTask` only.
- The suite is intentionally broad and shallow:
  - measure where the framework breaks
  - avoid single-seed overfitting
  - keep deferred complex tasks visible but not in the first gate

## Default Command

```bash
conda run -n script_policy python -m script_runtime.runners.evaluate_robotwin_multitask_suite \
  --suite script_runtime/configs/robotwin_multitask_place_suite.yaml
```

## Cheap Smoke Command

```bash
conda run -n script_policy python -m script_runtime.runners.evaluate_robotwin_multitask_suite \
  --suite script_runtime/configs/robotwin_multitask_place_suite.yaml \
  --task-filter place_empty_cup \
  --seeds 1 \
  --no-video
```

## Important Mapping Note

- `place_object_stand` uses `displaystand` as the actual RoboTwin target actor attribute.
- The suite should follow RoboTwin source actor names, otherwise we only test config mistakes instead of framework contract gaps.
