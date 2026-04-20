# Codex Workspace

这个 `.codex/` 是 `script_policy` 仓库的本地工作区，用来沉淀：
- 项目记忆
- 当前主线计划
- 仓库结构速记
- 面向后续反复施工的本地 skills
- 默认提示词契约与任务模板

当前仓库的核心目标：
- 以 `script_runtime/` 为执行层主干
- 以 `arm_control_sdk/` 为真机执行边界
- 以 `RoboTwin / robotwin2.0` 为主要仿真验证面
- 逐步建立从仿真到真机的 script policy 开发链

优先阅读顺序：
1. `PROMPT_RULES.md`
2. `WORKSPACE.md`
3. `MEMORY.md`
4. `ROBOTWIN_PLAN.md`
5. `FM_FIRST_GRASP_STACK_PLAN.md`
6. `OPEN_SOURCE_SCRIPT_POLICY_INDEX.md`
7. `TASK_PROMPT_TEMPLATE.md`
8. `../docs/HANDOFF_FROM_RL_VLA_2026-04-15.md`

本地 skills：
- `skills/robotwin-runtime/`
  - 用于 RoboTwin 环境、桥接、验证链施工
- `skills/grasp-debug-loop/`
  - 用于复杂任务的抓取语义排障与“先看图再动代码”的闭环

当前新增准则：
- 做新功能前，先查 `OPEN_SOURCE_SCRIPT_POLICY_INDEX.md`
- 优先复用或参考已有开源仓库实现，避免重复造轮子
- 复杂任务默认先过“抓取语义校验”，再继续优化放置末端
- 每轮真实任务默认要求真实视角可视化审阅，不再只看标量 trace
- 当前上游抓取链默认按 `FM-first grasp stack + runtime execution` 路线推进
- grounding / pose / grasp proposal 默认同时保留多个开源候选做横向比较，不提前只押一个方案
