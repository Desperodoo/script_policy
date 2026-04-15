# Codex Workspace

这个 `.codex/` 是 `script_policy` 仓库的本地工作区，用来沉淀：
- 项目记忆
- 当前主线计划
- 仓库结构速记
- 面向后续反复施工的本地 skills

当前仓库的核心目标：
- 以 `script_runtime/` 为执行层主干
- 以 `arm_control_sdk/` 为真机执行边界
- 以 `RoboTwin / robotwin2.0` 为主要仿真验证面
- 逐步建立从仿真到真机的 script policy 开发链

优先阅读顺序：
1. `WORKSPACE.md`
2. `MEMORY.md`
3. `ROBOTWIN_PLAN.md`
4. `OPEN_SOURCE_SCRIPT_POLICY_INDEX.md`
4. `../docs/HANDOFF_FROM_RL_VLA_2026-04-15.md`

本地 skills：
- `skills/robotwin-runtime/`
  - 用于 RoboTwin 环境、桥接、验证链施工

当前新增准则：
- 做新功能前，先查 `OPEN_SOURCE_SCRIPT_POLICY_INDEX.md`
- 优先复用或参考已有开源仓库实现，避免重复造轮子
