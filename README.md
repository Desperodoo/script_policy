# script_policy

独立的 script policy 执行平台仓库。

当前仓库来源于 `rl-vla` 中已施工的 `script_runtime/` 与 `arm_control_sdk/` 迁移，目标是围绕：
- `arm_control_sdk` 真机执行边界
- skill-based script runtime
- 面向 `robotwin2.0` 的仿真验证链

来继续演进一套可迁移到真实机械臂的 script policy 框架。

## 当前仓库包含什么

- `script_runtime/`
  - skill / task / blackboard / trace / recovery / adapter 主干
  - 当前仍保留 ManiSkill 验证代码，作为历史验证参考
- `arm_control_sdk/`
  - 当前真机执行边界
  - Python pybind 构建入口和现有二进制依赖
- `docs/`
  - 从 `rl-vla` 迁移过来的交接文档与平台计划

## 当前建议的阅读顺序

1. `docs/HANDOFF_FROM_RL_VLA_2026-04-15.md`
2. `script_runtime/README.md`
3. `docs/SCRIPT_POLICY_PLATFORM_PLAN_FROM_RL_VLA.md`

## 当前工程判断

- 这次迁移的重点是保留“执行层骨架 + SDK 边界”
- ManiSkill 代码保留，但后续主验证面应逐步切到 `robotwin2.0`
- 当前最值得优先施工的是 `robotwin` 桥接和 perception contract 升级，而不是继续扩展 cube 风格任务

## 最小启动方式

先验证 runtime 基础装配：

```bash
python -m script_runtime.runners.sdk_pick_place --dry-run
```

后续建议新增：

```text
script_runtime/adapters/robotwin_bridge.py
script_runtime/runners/robotwin_pick_place.py
script_runtime/validation/robotwin_rollout.py
```
