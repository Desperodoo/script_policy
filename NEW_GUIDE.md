# script_policy 最小操作手册

这份文档是给未来的你或新 agent 的第一入口。它只回答四件事：

1. 该跑哪个命令
2. 看哪三个指标
3. 失败后第一眼看哪张图
4. 下一步怎么判断

当前项目状态请先读：

- `.codex/CURRENT_STATE.md`

## 1. 该跑哪个命令

### 快速确认报告和 summary 代码没有坏

```bash
conda run -n script_policy pytest \
  script_runtime/tests/test_robotwin_multitask_suite.py \
  script_runtime/tests/test_robotwin_suite_report.py
```

### 正式 place-only 门禁

用途：确认基础放置任务有没有退化。

```bash
python -m script_runtime.runners.evaluate_robotwin_multitask_suite \
  --suite script_runtime/configs/robotwin_multitask_place_suite.yaml
```

报告位置：

```text
script_runtime/artifacts/robotwin_multitask/robotwin_multitask_place_baseline/robotwin_multitask_place_baseline_summary.md
```

### complex probe

用途：暴露复杂任务合同问题，不以通过率为第一目标。

```bash
python -m script_runtime.runners.evaluate_robotwin_multitask_suite \
  --suite script_runtime/configs/robotwin_multitask_complex_probe_suite.yaml
```

报告位置：

```text
script_runtime/artifacts/robotwin_multitask/robotwin_multitask_complex_probe/robotwin_multitask_complex_probe_summary.md
```

### FM backend compare

用途：比较不同抓取/感知后端在健康 place 任务上的表现。

```bash
python -m script_runtime.runners.evaluate_robotwin_multitask_suite \
  --suite script_runtime/configs/robotwin_multitask_place_fm_backend_compare_suite.yaml
```

报告位置：

```text
script_runtime/artifacts/robotwin_multitask/robotwin_multitask_place_fm_backend_compare/robotwin_multitask_place_fm_backend_compare_summary.md
```

## 2. 看哪三个指标

每份 suite summary 先看 `Human Summary`，再看机器表格。

三个核心指标：

| 指标 | 人话解释 | 机器字段 |
| --- | --- | --- |
| 环境成功数 | RoboTwin 最终认为任务完成了几次 | `Env success count` |
| 主要失败阶段 | 当前最值得看的能力缺口 | `failure_stage` |
| 实际执行候选 | 真正被拿去执行的抓取方式 | `Executed Candidate` |

如果是正式门禁，先看环境成功数。如果是 complex probe，先看主要失败阶段。如果是 FM compare，先看后端是否真的被选中并执行。

## 3. 失败后第一眼看哪里

优先级：

1. `Human Summary`：先确认报告对失败的中文解释。
2. run 目录里的真实视角图或动图：确认动作在画面里是否真的发生。
3. trace JSONL：只在图片和 summary 不够解释时再看机器字段。

常见视觉产物字段：

| 产物 | 用途 |
| --- | --- |
| `realview_contact_sheet_png` | 快速看关键动作是否符合预期 |
| `rollout_gif` | 看整轮动作过程 |
| `grounding_json` | 看目标定位和候选选择细节 |

## 4. 下一步怎么判断

先按人话判断，不要直接追机器字段：

| 现象 | 下一步 |
| --- | --- |
| 没找到目标或位姿明显错 | 查感知和目标定位 |
| 有候选但抓不上 | 查抓取候选、抓取语义和夹爪闭合 |
| 抓住后抬起丢了 | 查抬起后的抓取保持 |
| 动作看起来完成但环境不判成功 | 查 RoboTwin 成功条件和最终物体/目标几何 |
| FM 后端都能跑但没提升任务 | 查后端候选是否真的进入执行链 |

当前最重要的复杂任务判断：

> `place_can_basket` 的下一步不是继续追候选命名，而是解释“支撑侧动作完成”和“RoboTwin 判成功”之间差在哪里。

对应机器字段可以附在报告后面，但不要让字段替代这个判断。
