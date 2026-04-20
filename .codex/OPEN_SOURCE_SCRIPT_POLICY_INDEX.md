# Open Source Script Policy Index

这个索引的目标不是简单列仓库，而是建立：

`功能 -> 优先参考仓库 -> 适合复用什么`

默认准则：
- 每次要实现新功能时，先查这个索引
- 优先去看对应功能的优先仓库
- 优先复用代码，其次复用接口和架构
- 若最终决定不复用，也应先说明为什么

## 1. 执行层 / 行为树 / 任务编排

### 1.1 `py_trees`

- 仓库：`https://github.com/splintered-reality/py_trees`
- 适合参考：
  - Python 风格行为树节点组织
  - blackboard 设计
  - retry / fallback / decorator 组织方式
- 对当前仓库价值：
  - 最适合参考 `script_runtime/executors/` 的演进方向
  - 适合继续完善 Python runtime，而不是强行切 C++

### 1.2 `py_trees_ros`

- 仓库：`https://github.com/splintered-reality/py_trees_ros`
- 适合参考：
  - 行为树与 ROS topic/service/action 的桥接方式
  - blackboard 与机器人中间件交互方式
- 对当前仓库价值：
  - 后续如果 `script_runtime` 需要重新与 ROS 链路融合，这个仓库优先参考

### 1.3 `BehaviorTree.CPP`

- 仓库：`https://github.com/BehaviorTree/BehaviorTree.CPP`
- 适合参考：
  - 工业级行为树执行器
  - 异步 action node
  - 插件化 action / condition / decorator
- 对当前仓库价值：
  - 当前不必直接迁，但如果以后 runtime 规模变大，这是最值得参考的 C++ 行为树主干

## 2. 几何规划 / 任务级 manipulation planning

### 2.1 `MoveIt Task Constructor`

- 仓库：`https://github.com/moveit/moveit_task_constructor`
- 适合参考：
  - 多阶段 manipulation task 的 stage 化组织
  - pick / place / grasp pipeline 的规划结构
  - 任务分解与阶段依赖表达
- 对当前仓库价值：
  - 对 `pick-place`、`drawer`、`cabinet` 这类多阶段任务非常有参考价值
  - 即使不直接引入 MoveIt，也值得参考其 task decomposition 方式

### 2.2 `PlanSys2`

- 仓库：`https://github.com/PlanSys2/ros2_planning_system`
- 适合参考：
  - symbolic task planning
  - 任务重排与高层计划执行
- 对当前仓库价值：
  - 暂时不是第一阶段重点
  - 当长任务开始出现显式 reordering 需求时再重点参考

### 2.3 `PDDLStream`

- 仓库：`https://github.com/caelan/pddlstream`
- 适合参考：
  - task and motion planning
  - symbolic planning 与几何可行性检查的结合
- 对当前仓库价值：
  - 很适合未来做“符号计划 + grasp/IK/碰撞检查”耦合
  - 当前先作为中长期参考

### 2.4 `MPlib`

- 仓库：`https://github.com/haosulab/MPlib`
- 适合参考：
  - 轻量 motion planning
  - IK / collision / path planning
- 对当前仓库价值：
  - RoboTwin 本身已经在用
  - 后续如果 `robotwin_bridge` 里要对接规划语义，这个仓库应优先看

### 2.5 `cuRobo`

- 仓库：`https://github.com/NVlabs/curobo`
- 适合参考：
  - GPU motion generation
  - trajectory optimization
  - 高速碰撞约束规划
- 对当前仓库价值：
  - 适合后续在复杂场景下做更快的 motion generation
  - 当前优先作为高级规划参考，不必过早深耦合

## 3. 抓取 proposal / 抓取检测 / clutter grasping

### 3.1 `GPD`

- 仓库：`https://github.com/atenpas/gpd`
- 适合参考：
  - 传统点云抓取检测
  - clutter scene 抓取候选生成
- 对当前仓库价值：
  - 如果我们后续做传统方案的 grasp candidate generation，这个仓库优先参考
  - 适合作为“不依赖大模型”的稳健基线

### 3.2 `Contact-GraspNet`

- 仓库：`https://github.com/NVlabs/contact_graspnet`
- 适合参考：
  - 单视角 / partial point cloud 的 grasp proposal
  - clutter 场景抓取
- 对当前仓库价值：
  - 对杯子、容器、部分不规则物体的 grasp proposal 很有参考价值
  - 适合作为 learning-based proposal generator

### 3.3 `GraspNet Baseline`

- 仓库：`https://github.com/graspnet/graspnet-baseline`
- 适合参考：
  - 6-DoF grasp detection
  - grasp candidate ranking
- 对当前仓库价值：
  - 适合作为 grasp scoring / ranking 参考
  - 很适合挂到 `GenerateGraspCandidates` 或 `RankGraspCandidates` 这一层

### 3.4 `VGN`

- 仓库：`https://github.com/ethz-asl/vgn`
- 适合参考：
  - volumetric grasping
  - TSDF / voxel-based grasp prediction
- 对当前仓库价值：
  - 如果后面我们用深度 / TSDF 重建抓取候选，这个仓库很有参考价值

### 3.5 `GraspGen`

- 仓库：`https://github.com/NVlabs/GraspGen`
- 适合参考：
  - foundation-model 风格的 grasp generation 路线
  - 作为 `Contact-GraspNet / GraspNet` 之外的第三条 grasp proposal 候选
- 对当前仓库价值：
  - 很适合放进 `FM-first grasp stack` 做横向对比
  - 即使暂时不直接复用，也值得优先参考其输入输出组织

## 4. 目标物体定位 / 开放世界目标 grounding / 6D pose

### 4.1 `FoundationPose`

- 仓库：`https://github.com/NVlabs/FoundationPose`
- 适合参考：
  - 类别级 / 实例级 pose tracking
  - foundation-model 风格的目标 pose 获取
- 对当前仓库价值：
  - 非常适合未来做复杂物体位姿估计
  - 尤其适合杯子、容器、工具类物体的位姿跟踪方向

### 4.2 `GroundingDINO`

- 仓库：`https://github.com/IDEA-Research/GroundingDINO`
- 适合参考：
  - 文本驱动目标检测
  - 目标实例 grounding
- 对当前仓库价值：
  - 适合先解决“抓哪个物体”而不是“怎么抓”
  - 可作为 `ResolveTargetObject` 的候选底座

### 4.3 `SAM 2`

- 仓库：`https://github.com/facebookresearch/sam2`
- 适合参考：
  - 高质量分割
  - mask 级目标提取
- 对当前仓库价值：
  - 适合配合 GroundingDINO 或其它 detector，形成实例分割输入

### 4.4 `Grounded-SAM-2`

- 仓库：`https://github.com/IDEA-Research/Grounded-SAM-2`
- 适合参考：
  - 文本驱动目标 grounding + segmentation + tracking 的整合流程
- 对当前仓库价值：
  - 很适合作为 `TargetGrounder` 的首批后端
  - 可以把“抓哪个物体”从 task-specific 逻辑里正式拆出来

### 4.5 `Manipulate-Anything`

- 仓库：`https://github.com/Robot-MA/manipulate-anything`
- 适合参考：
  - foundation-model 参与真实操作任务的整体范式
  - 目标 grounding 与 manipulation 之间的接口设计
- 对当前仓库价值：
  - 更适合参考架构和模块边界
  - 不应直接替代当前 runtime，但非常适合指导上游模块化设计

## 5. 仿真平台 / 任务资源

### 5.1 `RoboTwin`

- 仓库：`https://github.com/RoboTwin-Platform/RoboTwin`
- 适合参考：
  - 更接近真实世界的双臂任务
  - 丰富对象与任务种类
  - 真实风格 task config / embodiment / data collection pipeline
- 对当前仓库价值：
  - 这是当前主仿真平台，不只是参考仓库

### 5.2 `LIBERO`

- 仓库：`https://github.com/Lifelong-Robot-Learning/LIBERO`
- 适合参考：
  - 长时序 household manipulation benchmark
  - goal-conditioned 和 task-conditioned manipulation 任务设计
- 对当前仓库价值：
  - 很适合作为长期任务设计和评测维度的参考

## 6. 当前推荐的功能到仓库映射

### 行为树增强

- 第一参考：`py_trees`
- 第二参考：`BehaviorTree.CPP`

### ROS 回接

- 第一参考：`py_trees_ros`

### 多阶段 pick-place / articulated 任务规划

- 第一参考：`MoveIt Task Constructor`
- 第二参考：`PDDLStream`

### motion planning / IK / collision

- 第一参考：`MPlib`
- 第二参考：`cuRobo`

### 传统 grasp candidate generation

- 第一参考：`GPD`

### learning-based grasp proposal

- 第一参考：`Contact-GraspNet`
- 第二参考：`GraspNet Baseline`
- 第三参考：`GraspGen`
- 第四参考：`VGN`

### 文本到目标物体 grounding

- 第一参考：`Grounded-SAM-2`
- 第二参考：`GroundingDINO`
- 第三参考：`SAM 2`

### 复杂物体 6D pose / tracking

- 第一参考：`FoundationPose`

### FM-first grasp stack

- Grounding：
  - `Grounded-SAM-2`
  - `GroundingDINO + SAM 2`
- Pose：
  - `FoundationPose`
- Grasp proposal：
  - `Contact-GraspNet`
  - `GraspNet Baseline`
  - `GraspGen`
- Task-aware rerank：
  - `GraspGPT / FoundationGrasp`

默认准则：
- 当前复杂任务默认先按这条栈做多后端横向试验
- 不再只押单一 heuristic provider

## 7. 实施准则

当准备实现某个功能时，默认执行顺序：

1. 明确功能属于哪一类
2. 在本索引中找到对应仓库
3. 先看：
   - 接口设计
   - 输入输出
   - 关键数据结构
   - rollout / eval / visualization 方式
4. 判断是：
   - 直接复用代码
   - 包装成 adapter
   - 仅参考架构
5. 再开始仓库内实现

如果没有先走这一步，不应默认直接从零实现。
