# Script Policy Task Prompt Template

下面这份模板用于后续在本仓库里发起新任务时，尽量把默认约束一并带上。

## 模板

你现在在 `script_policy` 仓库中施工。

开工前请先阅读：
- `./.codex/PROMPT_RULES.md`
- `./.codex/WORKSPACE.md`
- `./.codex/MEMORY.md`
- `./.codex/ROBOTWIN_PLAN.md`
- `./.codex/OPEN_SOURCE_SCRIPT_POLICY_INDEX.md`

默认约束：
- 优先查开源索引，优先复用或参考已有实现
- 不要把复杂逻辑直接堆进单个 task patch
- 先检查上游语义链路，再优化下游表现
- `CheckGrasp` 不能只等于“抓住了”
- 每轮真实任务或 RoboTwin 复杂任务必须导出并审阅真实视角可视化
- 如果图片和标量冲突，以图片暴露的问题优先
- 每轮阶段性进展要更新回 `./.codex`

本次任务：
- 在这里写具体目标

本次任务的验收：
- 在这里写需要跑的测试
- 在这里写需要产出的 artifact
- 在这里写需要更新的 `.codex` 文档

如果你发现当前方向的核心前提不成立，请先指出“上游哪一层坏了”，再决定是否继续优化当前模块。
