# Behavior Path Planner Slot 机制详解

## 1. Slot 概述

### 1.1 什么是 Slot？

**Slot（插槽）** 是 Behavior Path Planner 中的核心调度单元，用于将不同的场景模块（Scene Module）按功能依赖关系分组，并控制它们的执行顺序和状态传播。

每个 Slot 包含一个或多个场景模块管理器（SceneModuleManager），这些模块在同一个 Slot 内可以**串行执行**（approved 模块）或**并行评估**（candidate 模块），但不同 Slot 之间是**严格串行**的，确保路径规划的稳定性和可预测性。

### 1.2 Slot 的设计目的

1. **模块依赖管理**：将功能相关的模块组织在一起，避免执行顺序冲突
2. **状态隔离**：不同 Slot 之间的状态变化不会相互干扰
3. **故障隔离**：上游 Slot 的失败可以安全地传播到下游，避免级联错误
4. **性能优化**：通过分组减少不必要的模块评估次数

## 2. Slot 配置结构

### 2.1 默认 Slot 配置

根据 `scene_module_manager.param.yaml`，系统默认配置了 **4 个 Slot**：

```yaml
slots:
  - slot1
  - slot2
  - slot3
  - slot4

slot1:
  - "start_planner"              # 起步规划器

slot2:
  - "side_shift"                 # 侧向偏移
  - "avoidance_by_lane_change"   # 通过变道避障
  - "static_obstacle_avoidance"  # 静态障碍物避障
  - "lane_change_left"           # 左变道
  - "lane_change_right"          # 右变道
  - "external_request_lane_change_left"   # 外部请求左变道
  - "external_request_lane_change_right"  # 外部请求右变道

slot3:
  - "goal_planner"               # 目标规划器（停车）

slot4:
  - "dynamic_obstacle_avoidance" # 动态障碍物避障
```

### 2.2 各 Slot 的职责划分

#### **Slot1: 起步阶段（Start Planner）**
- **功能**：处理车辆从静止状态（如停车位、路肩）进入行驶车道的场景
- **特点**：通常是路径规划的起点，优先级最高
- **模块**：`start_planner`
- **执行特性**：
  - `enable_simultaneous_execution_as_approved_module: true` - 可与已批准模块并行
  - `enable_simultaneous_execution_as_candidate_module: false` - 候选时不可并行

#### **Slot2: 路径调整与避障（Path Adjustment & Avoidance）**
- **功能**：处理正常行驶过程中的路径调整、避障和变道
- **特点**：包含最多模块，是核心规划逻辑所在
- **模块**：
  - `side_shift` - 侧向微调（如避让路边障碍）
  - `avoidance_by_lane_change` - 通过变道避障
  - `static_obstacle_avoidance` - 静态障碍物避障
  - `lane_change_left/right` - 主动变道
  - `external_request_lane_change_*` - 外部请求的变道
- **执行特性**：
  - 各模块的并行能力不同：
    - `static_obstacle_avoidance`: 可与已批准模块并行，但候选时不可并行
    - `lane_change_*`: 已批准和候选时都可并行
    - `side_shift`: 已批准和候选时都不可并行（独占执行）

#### **Slot3: 目标规划（Goal Planner）**
- **功能**：处理车辆到达目标点并停车的场景（如路边停车、停车场）
- **特点**：通常是路径规划的终点
- **模块**：`goal_planner`
- **执行特性**：
  - `enable_simultaneous_execution_as_approved_module: true`
  - `enable_simultaneous_execution_as_candidate_module: false`

#### **Slot4: 动态避障（Dynamic Obstacle Avoidance）**
- **功能**：处理动态障碍物（如移动车辆、行人）的实时避障
- **特点**：需要快速响应，通常在其他规划完成后进行微调
- **模块**：`dynamic_obstacle_avoidance`
- **执行特性**：
  - `enable_simultaneous_execution_as_approved_module: true`
  - `enable_simultaneous_execution_as_candidate_module: true` - 完全可并行

## 3. Slot 工作机制详解

### 3.1 Slot 执行流程

```
┌─────────────────────────────────────────────────────────────┐
│ PlannerManager::run() - 主调度入口                           │
└─────────────────────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│ 1. 初始化：获取参考路径，构造初始 SlotOutput                 │
│    SlotOutput {                                             │
│      valid_output: 参考路径,                                 │
│      is_upstream_failed_approved: false,                     │
│      is_upstream_waiting_approved: false,                   │
│      is_upstream_candidate_exclusive: false                  │
│    }                                                         │
└─────────────────────────────────────────────────────────────┘
                        │
                        ▼
        ┌───────────────┴───────────────┐
        │   遍历所有 Slot（串行）        │
        └───────────────┬───────────────┘
                        │
        ┌───────────────▼───────────────┐
        │   Slot1: Start Planner        │
        │   ┌─────────────────────┐    │
        │   │ propagateFull()     │    │
        │   │ 或特殊传播模式       │    │
        │   └─────────────────────┘    │
        └───────────────┬───────────────┘
                        │
                        ▼ (输出传递给下一个 Slot)
        ┌───────────────▼───────────────┐
        │   Slot2: Path Adjustment      │
        │   ┌─────────────────────┐    │
        │   │ propagateFull()     │    │
        │   │ 或特殊传播模式       │    │
        │   └─────────────────────┘    │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────▼───────────────┐
        │   Slot3: Goal Planner         │
        │   ┌─────────────────────┐    │
        │   │ propagateFull()     │    │
        │   └─────────────────────┘    │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────▼───────────────┐
        │   Slot4: Dynamic Avoidance    │
        │   ┌─────────────────────┐    │
        │   │ propagateFull()     │    │
        │   └─────────────────────┘    │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │ 生成最终可行驶区域并返回       │
        └───────────────────────────────┘
```

### 3.2 Slot 内部执行模式

每个 Slot 根据上游状态，采用以下四种执行模式之一：

#### **模式1: propagateFull() - 完整执行模式**

**触发条件**：上游 Slot 正常，无特殊状态标记

**执行流程**：
```
┌─────────────────────────────────────────────┐
│ propagateFull() 循环（最多 N*(N+1)/2 次）   │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ Step 1: runApprovedModules()                │
│ - 串行执行所有已批准模块                      │
│ - 处理模块状态（成功/失败/等待审批）          │
│ - 返回 approved 模块的输出路径               │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ Step 2: getRequestModules()                 │
│ - 遍历所有模块管理器                         │
│ - 检查并行执行约束                           │
│ - 询问是否需要启动新模块                      │
│ - 返回请求执行的模块列表                     │
└─────────────────────────────────────────────┘
        │
        ▼
    ┌───┴───┐
    │ 为空? │ ──是──> 返回当前输出
    └───┬───┘
        │否
        ▼
┌─────────────────────────────────────────────┐
│ Step 3: runRequestModules()                 │
│ - 按优先级排序请求模块                       │
│ - 根据并行能力筛选可执行模块                  │
│ - 并行运行所有可执行候选模块                  │
│ - 返回最高优先级模块及其输出                  │
└─────────────────────────────────────────────┘
        │
        ▼
    ┌───┴──────────────┐
    │ 等待审批?        │ ──是──> 返回候选输出
    └───┬──────────────┘
        │否
        ▼
┌─────────────────────────────────────────────┐
│ Step 4: addApprovedModule()                 │
│ - 将最高优先级模块加入 approved 栈           │
│ - 清空候选模块                               │
│ - 继续下一轮循环                              │
└─────────────────────────────────────────────┘
```

**关键特性**：
- **迭代执行**：直到没有新模块请求执行为止
- **串行融合**：approved 模块按顺序执行，每个模块的输出作为下一个模块的输入
- **并行评估**：candidate 模块可以并行运行，但只有最高优先级的会被批准

#### **模式2: propagateWithExclusiveCandidate() - 候选独占模式**

**触发条件**：上游 Slot 存在互斥型候选模块（`is_upstream_candidate_exclusive = true`）

**执行流程**：
```
┌─────────────────────────────────────────────┐
│ propagateWithExclusiveCandidate()            │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ runApprovedModules()                        │
│ - 仅执行已批准模块                           │
│ - 不启动新的候选模块                         │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ 返回输出，保持独占标志                        │
│ is_upstream_candidate_exclusive = true       │
└─────────────────────────────────────────────┘
```

**设计目的**：
- 当上游有互斥型模块（如 `side_shift`）运行时，下游 Slot 不应启动新的候选模块，避免路径冲突
- 只允许已批准的模块继续执行，保证路径连续性

#### **模式3: propagateWithWaitingApproved() - 等待审批模式**

**触发条件**：上游 Slot 有模块返回等待审批状态（`is_upstream_waiting_approved = true`）

**执行流程**：
```
┌─────────────────────────────────────────────┐
│ propagateWithWaitingApproved()               │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ clearCandidateModules()                     │
│ - 清空当前 Slot 的所有候选模块                │
└─────────────────────────────────────────────┘
        │
        ▼
    ┌───┴──────────────────────┐
    │ 上游是否候选独占?         │
    └───┬──────────────────────┘
        │
    ┌───┴───┐
    │ 是    │ ──> propagateWithExclusiveCandidate()
    │ 否    │ ──> propagateFull()
    └───────┘
```

**设计目的**：
- 当上游模块需要重新审批时，下游需要刷新状态
- 如果上游是独占模式，下游也保持独占；否则重新走完整流程

#### **模式4: propagateWithFailedApproved() - 失败传播模式**

**触发条件**：上游 Slot 有已批准模块失败（`is_upstream_failed_approved = true`）

**执行流程**：
```
┌─────────────────────────────────────────────┐
│ propagateWithFailedApproved()                │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ clearCandidateModules()                     │
│ clearApprovedModules()                      │
│ - 清空所有模块，不执行任何规划                │
└─────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│ 直接传递上游输出，不修改                     │
└─────────────────────────────────────────────┘
```

**设计目的**：
- 上游失败时，下游 Slot 不应继续执行，避免基于错误输入产生更严重的错误
- 保持上游的输出路径作为兜底方案

### 3.3 Approved 模块与 Candidate 模块

#### **Approved 模块（已批准模块）**

**定义**：已经通过 RTC（Request To Cooperate）审批，可以实际修改车辆路径的模块

**特点**：
- **串行执行**：在同一个 Slot 内，approved 模块按顺序执行
- **路径融合**：每个模块的输出作为下一个模块的输入
- **状态稳定**：一旦批准，模块会持续运行直到完成或失败
- **路径锁定**：只有最后一个 approved 模块可以修改输出路径

**状态转换**：
```
IDLE → WAITING_APPROVAL → RUNNING → SUCCESS/FAILURE
                              ↓
                        (需要重新审批)
                        WAITING_APPROVAL
```

#### **Candidate 模块（候选模块）**

**定义**：请求执行但尚未获得审批的模块

**特点**：
- **并行评估**：多个 candidate 模块可以同时运行，评估各自的路径方案
- **优先级竞争**：系统会选择最高优先级的 candidate 模块进行审批
- **路径预览**：candidate 模块的输出用于可视化，但不影响实际车辆路径
- **并行约束**：根据 `enable_simultaneous_execution_as_candidate_module` 决定是否可以并行

**并行规则示例**：
- `lane_change_left` 和 `lane_change_right` 可以同时作为 candidate 运行
- `side_shift` 作为 candidate 时，其他模块不能同时作为 candidate

### 3.4 Slot 状态传播机制

Slot 之间通过 `SlotOutput` 结构传递状态：

```cpp
struct SlotOutput {
  BehaviorModuleOutput valid_output;              // 有效路径输出
  bool is_upstream_candidate_exclusive;            // 上游候选独占标志
  bool is_upstream_failed_approved;                // 上游已批准模块失败
  bool is_upstream_waiting_approved;               // 上游等待审批
};
```

**状态传播规则**：

1. **正常状态（NORMAL）**：
   - 所有标志为 `false`
   - 下游 Slot 正常执行 `propagateFull()`

2. **候选独占（UPSTREAM_EXCLUSIVE_CANDIDATE）**：
   - `is_upstream_candidate_exclusive = true`
   - 下游只执行已批准模块，不启动新候选

3. **等待审批（UPSTREAM_WAITING_APPROVED）**：
   - `is_upstream_waiting_approved = true`
   - 下游清空候选，根据是否独占选择执行模式

4. **失败传播（UPSTREAM_APPROVED_FAILED）**：
   - `is_upstream_failed_approved = true`
   - 下游清空所有模块，直接传递输出

## 4. Slot 执行示例

### 示例1：正常变道场景

```
初始状态：
- Slot1: 无模块运行
- Slot2: 无模块运行
- Slot3: 无模块运行
- Slot4: 无模块运行

执行流程：
1. Slot1: 无请求 → 输出参考路径
2. Slot2: 
   - lane_change_left 请求执行 → 作为 candidate 运行
   - 获得审批 → 转为 approved
   - 执行变道路径规划
   - 输出：变道后的路径
3. Slot3: 无请求 → 传递 Slot2 的输出
4. Slot4: 
   - dynamic_obstacle_avoidance 检测到动态障碍
   - 在变道路径基础上微调避障
   - 输出：最终路径

最终输出：变道 + 动态避障的融合路径
```

### 示例2：上游失败场景

```
执行流程：
1. Slot1: start_planner 运行中
2. Slot2: 
   - static_obstacle_avoidance 作为 approved 运行
   - 检测到无法避障 → 返回 FAILURE
   - 设置 is_upstream_failed_approved = true
3. Slot3: 
   - 检测到失败标志
   - 执行 propagateWithFailedApproved()
   - 清空所有模块
   - 直接传递 Slot2 的输出（失败前的路径）
4. Slot4: 
   - 同样检测到失败标志
   - 清空所有模块
   - 传递最终输出

最终输出：失败前的最后一个有效路径（作为安全兜底）
```

### 示例3：等待审批场景

```
执行流程：
1. Slot1: 正常执行
2. Slot2: 
   - lane_change_left 作为 approved 运行
   - 需要修改路径 → 返回 WAITING_APPROVAL
   - 从 approved 栈移除，加入 candidate 栈
   - 设置 is_upstream_waiting_approved = true
3. Slot3: 
   - 检测到等待审批标志
   - 执行 propagateWithWaitingApproved()
   - 清空候选模块
   - 重新评估是否需要启动新模块
4. Slot4: 正常执行

最终输出：等待审批模块的候选路径（用于可视化，等待外部审批）
```

## 5. Slot 配置的最佳实践

### 5.1 模块分组原则

1. **功能相关性**：将功能相关的模块放在同一个 Slot
2. **执行顺序**：考虑模块之间的依赖关系
3. **性能考虑**：避免在同一个 Slot 中放置过多模块

### 5.2 并行能力配置

- **可并行模块**：功能互补，不会产生路径冲突（如 `lane_change` 和 `dynamic_avoidance`）
- **互斥模块**：功能冲突，必须独占执行（如 `side_shift`）

### 5.3 优先级设置

- 优先级数字越小，优先级越高
- 在同一 Slot 内，高优先级模块会优先被批准执行

## 6. 总结

Slot 机制是 Behavior Path Planner 的核心架构，它通过以下方式确保了路径规划的稳定性和可扩展性：

1. **模块隔离**：不同 Slot 之间的状态互不干扰
2. **故障隔离**：上游失败不会导致下游产生更严重的错误
3. **灵活扩展**：可以轻松添加新的 Slot 和模块
4. **性能优化**：通过分组和并行评估提高效率
5. **状态管理**：清晰的状态传播机制确保系统行为可预测

理解 Slot 机制对于：
- **调试问题**：快速定位问题所在的 Slot 和模块
- **性能优化**：合理配置模块分组和并行能力
- **功能扩展**：正确添加新的场景模块
- **系统维护**：理解整个规划系统的执行流程

都具有重要意义。

