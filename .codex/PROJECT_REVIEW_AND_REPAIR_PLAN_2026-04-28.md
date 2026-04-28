# Project Review and Repair Plan

Date: 2026-04-28

## 一句话结论

`script_policy` 已经从实验脚本长成了一个机器人任务执行平台雏形，但当前协作界面过度暴露机器字段，导致“系统可观测性变强，人类可读性变弱”。

这轮修复的目标不是推进新能力，而是先把项目重新变得人能读懂。

## 项目初衷

这个项目的核心初衷是：

- 把旧 `rl-vla` 中可运行的 script policy 执行逻辑独立出来。
- 以 `script_runtime` 作为任务执行核心，而不是继续把执行逻辑散在训练框架或临时脚本里。
- 以 RoboTwin 作为主要仿真验证面，先建立稳定的任务门禁、失败分类、trace 和可视化 artifact。
- 保留 `arm_control_sdk` 作为真机执行边界，让当前仿真验证链以后能迁移到真实机械臂。
- 把 perception / grasp / FM backend 接回同一条 runtime 链路，让“抓对了吗、为什么失败、哪个 backend 有用”能够被比较。

## 当前优点

- 架构方向清楚：`script_runtime`、RoboTwin adapter、FM grasp stack、真机 SDK 边界分工明确。
- 调试闭环认真：项目不只看 success 标量，而是记录失败阶段、抓取候选、抓取语义、视觉产物和 suite summary。
- 已有平台门禁：place-only gate、complex probe、FM backend compare 三类报告面已经成型。
- 测试基础可用：core、session、bridge、grasp semantics、multitask suite 都有单测保护。

## 当前不足

- 报告语言过度机器化。`contact:right:0:contact_0`、`fm_backend_compare`、`fresh artifact`、`support_regrasp_substage` 这类字段可以存在于 JSON/trace 中，但不应作为计划和报告的主要叙述。
- `.codex/MEMORY.md` 和 `.codex/ROBOTWIN_PLAN.md` 更像现场流水账，缺少清晰的当前结论层。
- 部分核心文件已经偏大，例如 multitask suite runner、RoboTwin bridge、FM grasp stack；这轮不拆，但后续需要关注。
- 项目很会解释某次 run 为什么失败，但对人类读者来说，第一眼还不够清楚：当前是否通过、主要卡在哪里、下一步该看什么。

## 本轮修复边界

本轮只修可读性和协作界面：

- 新增这份项目审视记录。
- 固化 human-first reporting 规则。
- 改进 suite markdown 报告，让开头先给中文判断。
- 为 JSON summary 增加非破坏性 `human_summary` 字段。

本轮不做：

- 不推进新任务或新 backend。
- 不重构大文件。
- 不改变 runtime 执行逻辑。
- 不删除或重命名现有 summary / trace / artifact 字段。

## 后续表达规则

默认写给人看的计划或报告时，必须遵守：

1. 先说中文结论，再给机器字段。
2. 先回答“是否通过、卡在哪里、下一步看什么”。
3. 机器字段只能作为证据附后，不能裸奔成主要叙述。
4. 如果必须写内部候选 id，应配一句解释。

推荐写法：

```text
place_can_basket 已经能完成支撑侧抓取和抬起，但最后 RoboTwin 仍不认为任务成功。当前要查的是“动作完成”和“环境成功条件”之间差在哪。

机器字段：support_regrasp / success_mismatch / CheckTaskSuccess，候选 contact:right:0:contact_0。
```

不推荐写法：

```text
place_can_basket = support_regrasp / success_mismatch / CheckTaskSuccess, attempt_candidate_identity = contact:right:0:contact_0
```

## GPT-5.5 协作要求

切到 GPT-5.5 后，后续协作应比之前更克制、更会翻译：

- 不把内部字段当成人类汇报语言。
- 不用新黑话解释旧黑话。
- 把“当前结论”和“历史过程”分开。
- 对长期计划给清楚判断，对机器细节保留可追溯证据。
