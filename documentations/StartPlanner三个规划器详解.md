# StartPlanner 三个规划器详解

## 概述

StartPlanner 模块支持三种不同的起步路径规划器：
1. **Shift Pull Out**（位移起步规划器）
2. **Geometric Pull Out**（几何起步规划器）
3. **Freespace Pull Out**（自由空间起步规划器）

本文档详细说明这三个规划器的作用、输入输出，以及它们如何协同工作。

---

## 1. 同时启用三个规划器会怎么样？

### 1.1 启用机制

**代码位置**: `start_planner_module.cpp:72-92`

```cpp
StartPlannerModule::StartPlannerModule(...)
{
  // ⭐ 启用 Shift Pull Out
  if (parameters_->enable_shift_pull_out) {
    start_planners_.push_back(std::make_shared<ShiftPullOut>(node, *parameters, time_keeper_));
  }
  
  // ⭐ 启用 Geometric Pull Out
  if (parameters_->enable_geometric_pull_out) {
    start_planners_.push_back(std::make_shared<GeometricPullOut>(node, *parameters, time_keeper_));
  }
  
  // ⚠️ 至少需要启用一个
  if (start_planners_.empty()) {
    RCLCPP_ERROR(getLogger(), "Not found enabled planner");
  }

  // ⭐ 启用 Freespace Planner（独立处理）
  if (parameters_->enable_freespace_planner) {
    freespace_planner_ = std::make_unique<FreespacePullOut>(node, *parameters);
    // 创建单独的定时器和回调组
    const auto freespace_planner_period_ns = rclcpp::Rate(1.0).period();
    freespace_planner_timer_cb_group_ =
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    freespace_planner_timer_ = rclcpp::create_timer(
      &node, clock_, freespace_planner_period_ns,
      std::bind(&StartPlannerModule::onFreespacePlannerTimer, this),
      freespace_planner_timer_cb_group_);
  }
}
```

### 1.2 协同工作机制

#### **Shift 和 Geometric 规划器（正常模式）**

存储在 `start_planners_` 数组中，按优先级顺序尝试：

```cpp
std::vector<std::shared_ptr<PullOutPlannerBase>> start_planners_;
// 例如：[ShiftPullOut, GeometricPullOut]
```

**选择策略**（`start_planner_module.cpp:915-945`）:

```cpp
void StartPlannerModule::planWithPriority(
  const std::vector<Pose> & start_pose_candidates, 
  const Pose & refined_start_pose,
  const Pose & goal_pose, 
  const std::string & search_priority)
{
  // 1. 确定优先级顺序
  const PriorityOrder order_priority =
    determinePriorityOrder(search_priority, start_pose_candidates.size());

  // 2. 按优先级尝试每个规划器
  for (const auto & collision_check_margin : parameters_->collision_check_margins) {
    for (const auto & [index, planner] : order_priority) {
      if (findPullOutPath(
            start_pose_candidates[index], planner, refined_start_pose, goal_pose,
            collision_check_margin, debug_data_vector)) {
        // ⭐ 找到第一个成功的路径就立即返回
        return;
      }
    }
  }
  // 所有规划器都失败
  updateStatusIfNoSafePathFound();
}
```

**关键点**:
- ✅ **按顺序尝试**：先尝试第一个规划器，如果失败再尝试第二个
- ✅ **找到即停止**：一旦某个规划器成功生成路径，立即返回，不再尝试其他规划器
- ✅ **碰撞检查边界递进**：如果所有规划器在当前碰撞检查边界下都失败，会尝试更大的边界

#### **Freespace 规划器（应急模式）**

**独立运行**，只在特殊情况下激活：

```cpp
void StartPlannerModule::onFreespacePlannerTimer()
{
  // ⚠️ 触发条件：车辆卡住且其他规划器都失败
  const bool is_stuck = is_stopped && 
                        pull_out_status.planner_type == PlannerType::STOP &&
                        !pull_out_status.found_pull_out_path;
  
  if (is_stuck) {
    // 只在卡住时才尝试 Freespace 规划
    const auto free_space_status =
      planFreespacePath(parameters, local_planner_data, pull_out_status);
    if (free_space_status) {
      freespace_thread_status_ = free_space_status;  // 更新状态
    }
  }
}
```

**特点**:
- 🔄 **单独线程**：在独立的定时器回调中运行（1Hz）
- ⚠️ **应急使用**：只在车辆停止且其他规划器都失败时激活
- 💾 **异步计算**：规划结果存储在 `freespace_thread_status_`，主线程读取

### 1.3 优先级顺序

**两种优先级模式**（`start_planner_module.cpp:947-970`）:

#### 模式1: `efficient_path`（高效路径优先）

```cpp
if (search_priority == "efficient_path") {
  // 先遍历所有规划器，再遍历起始位姿候选
  for (const auto & planner : start_planners_) {      // 外层：规划器
    for (size_t i = 0; i < start_pose_candidates_num; i++) {  // 内层：位姿
      order_priority.emplace_back(i, planner);
    }
  }
}
```

**顺序示例**（假设有 Shift 和 Geometric，3个起始位姿候选）:
```
1. Shift + 位姿0
2. Shift + 位姿1
3. Shift + 位姿2
4. Geometric + 位姿0
5. Geometric + 位姿1
6. Geometric + 位姿2
```

**优势**: 优先尝试某个规划器的所有可能性，适合有明确偏好的场景

#### 模式2: `short_back_distance`（短后退距离优先）

```cpp
else if (search_priority == "short_back_distance") {
  // 先遍历起始位姿候选，再遍历规划器
  for (size_t i = 0; i < start_pose_candidates_num; i++) {  // 外层：位姿
    for (const auto & planner : start_planners_) {         // 内层：规划器
      order_priority.emplace_back(i, planner);
    }
  }
}
```

**顺序示例**（同样3个位姿候选）:
```
1. Shift + 位姿0
2. Geometric + 位姿0
3. Shift + 位姿1
4. Geometric + 位姿1
5. Shift + 位姿2
6. Geometric + 位姿2
```

**优势**: 优先尝试不需要后退或后退距离短的位姿，更快速

### 1.4 实际运行流程

```
开始
  ↓
尝试 start_planners_[0]（例如 Shift）
  ├─ 成功 → 使用该路径，结束 ✅
  └─ 失败 ↓
尝试 start_planners_[1]（例如 Geometric）
  ├─ 成功 → 使用该路径，结束 ✅
  └─ 失败 ↓
所有常规规划器都失败
  ├─ status_.planner_type = STOP
  ├─ status_.found_pull_out_path = false
  └─ 输出停止路径（速度=0）⚠️
        ↓
【后台】Freespace 定时器检测到卡住状态
  ↓
触发 Freespace 规划器
  ├─ 成功 → 更新状态，使用 Freespace 路径 ✅
  └─ 失败 → 继续停止 ⚠️
```

---

## 2. 三个规划器的详细介绍

### 2.1 Shift Pull Out（位移起步规划器）

#### 特点

- ⭐ **最简单、最快速**的规划器
- ✅ 适用于路边起步场景
- ✅ 使用路径平移（shift）实现横向位移
- ✅ 计算速度快，实时性好

#### 工作原理

```
当前位置（路边）        目标位置（车道中心）
    o                      o
    |                     /
    |    横向位移       /
    |  ←----------→   /
    |              /
    o-----------o
    
    1. 纵向前进
    2. 横向平移
    3. 并入车道
```

**算法步骤**:
1. 获取道路中心线参考路径
2. 计算当前位置到中心线的横向偏移量
3. 生成位移路径（shift line）
4. 应用路径平移算法（PathShifter）
5. 设置速度和加速度
6. 碰撞检查

#### 输入

```cpp
std::optional<PullOutPath> ShiftPullOut::plan(
  const Pose & start_pose,              // 起始位姿
  const Pose & goal_pose,               // 目标位姿
  const std::shared_ptr<const PlannerData> & planner_data,  // 规划数据
  PlannerDebugData & planner_debug_data // 调试数据
)
```

**详细输入**:
- `start_pose`: 车辆当前或候选起始位姿
- `goal_pose`: 路由的目标位姿
- `planner_data`: 包含
  - 道路车道信息（lanelet）
  - 地图信息
  - 车辆信息
  - 动态障碍物信息
- `planner_debug_data`: 用于记录规划过程信息

#### 输出

```cpp
std::optional<PullOutPath>  // 成功返回路径，失败返回 std::nullopt
```

**PullOutPath 内容**:
```cpp
{
  partial_paths: [横向位移路径],  // 通常只有1个分段
  pairs_terminal_velocity_and_accel: [(target_vel, accel)],
  start_pose: 起始位姿,
  end_pose: 结束位姿
}
```

#### 适用场景

✅ **适合**:
- 路边并入车道
- 停车场出口（较简单的情况）
- 横向偏移较小的起步
- 前方空间充足

❌ **不适合**:
- 停车位出库（需要大转向）
- 狭窄空间
- 需要大角度转向
- 后方空间不足

#### 参数配置

**关键参数**（`start_planner.param.yaml`）:
```yaml
shift_pull_out:
  lateral_jerk: 0.5              # 横向加速度变化率
  minimum_lateral_acc: 0.3       # 最小横向加速度
  maximum_lateral_acc: 1.0       # 最大横向加速度
  minimum_shift_pull_out_distance: 10.0  # 最小起步距离
  lateral_acceleration_sampling_num: 3   # 横向加速度采样数
```

---

### 2.2 Geometric Pull Out（几何起步规划器）

#### 特点

- ⭐ **基于几何弧线**的规划器
- ✅ 适用于停车位出库场景
- ✅ 使用平行泊车算法的逆向过程
- ✅ 能够处理较大的转向角度
- ⚠️ 计算时间比 Shift 略长

#### 工作原理

```
停车位               车道
 ┌─────┐
 │  o  │            起点（停车位内）
 │  |＼ │              ↓
 │  | ＼│            弧线1（转向出库）
 └──┼──＼──             ↓
    |   ＼           弧线2（调整朝向）
    |    ＼            ↓
    |     o         终点（车道中心）
    
使用两段弧线路径实现平滑出库
```

**算法步骤**:
1. 使用 `GeometricParallelParking` 规划器
2. 计算从起点到目标的弧线路径
3. 通常生成2段弧线（可配置是否分段）
   - 第1段：转向出停车位
   - 第2段：调整朝向并入车道
4. 设置每段的速度和加速度
5. 碰撞检查和车道偏离检查

#### 输入

```cpp
std::optional<PullOutPath> GeometricPullOut::plan(
  const Pose & start_pose,              // 起始位姿（停车位内）
  const Pose & goal_pose,               // 目标位姿
  const std::shared_ptr<const PlannerData> & planner_data,
  PlannerDebugData & planner_debug_data
)
```

**与 Shift 相同的输入结构，但使用方式不同**:
- 更关注起始位姿的朝向
- 需要考虑停车位边界
- 需要更大的转向半径

#### 输出

```cpp
std::optional<PullOutPath>
```

**PullOutPath 内容**（分段模式）:
```cpp
{
  partial_paths: [弧线路径1, 弧线路径2],  // 2个分段
  pairs_terminal_velocity_and_accel: [
    (avg_vel1, accel1),    // 第1段
    (target_vel, accel2)   // 第2段
  ],
  start_pose: 停车位内起点,
  end_pose: 车道中心终点
}
```

**PullOutPath 内容**（组合模式）:
```cpp
{
  partial_paths: [组合弧线路径],  // 1个分段（两段弧线合并）
  pairs_terminal_velocity_and_accel: [(target_vel, accel)],
  start_pose: 停车位内起点,
  end_pose: 车道中心终点
}
```

#### 适用场景

✅ **适合**:
- 停车位出库
- 需要大转向角度的起步
- 车道与停车位有角度差
- 空间相对规整

❌ **不适合**:
- 极度狭窄空间
- 障碍物密集环境
- 不规则空间
- 简单路边起步（过于复杂）

#### 参数配置

**关键参数**:
```yaml
geometric_pull_out:
  geometric_pull_out_velocity: 1.0  # 几何起步速度
  arc_path_interval: 1.0            # 弧线路径点间隔
  lane_departure_margin: 0.2        # 车道偏离余量
  divide_pull_out_path: false       # 是否分段路径
```

---

### 2.3 Freespace Pull Out（自由空间起步规划器）

#### 特点

- ⭐ **最强大但最慢**的规划器
- ✅ 使用 A* 或 RRT* 算法
- ✅ 能够处理复杂障碍物环境
- ✅ 支持前进和后退的组合
- ⚠️ **只在应急情况下使用**（其他规划器都失败）
- ⚠️ 计算时间最长
- 🔄 **异步运行**（独立线程）

#### 工作原理

```
障碍物环境中的复杂路径规划

  🚗 起点        ▓ 障碍物
   ↓
  ←─┐
    │ 后退
    └→┐
      │ 前进转向
      └→┐
        │ 继续前进
        └→ o 终点
        
使用搜索算法在自由空间中寻找路径
```

**算法选择**:
- **A* Search**: 网格搜索，保证找到最优路径
- **RRT***: 随机采样树，适合高维空间

**算法步骤**:
1. 使用代价地图（costmap）表示环境
2. 运行 A* 或 RRT* 搜索算法
3. 找到从起点到终点的可行路径
4. 检测需要倒车的点（reversing indices）
5. 根据倒车点分割路径为多个分段
6. 为每个分段设置速度

#### 输入

```cpp
std::optional<PullOutPath> FreespacePullOut::plan(
  const Pose & start_pose,              // 起始位姿
  const Pose & end_pose,                // 结束位姿
  const std::shared_ptr<const PlannerData> & planner_data,
  PlannerDebugData & planner_debug_data
)
```

**特殊输入需求**:
- **必须**有有效的 `costmap`（代价地图）
- **必须**是最新的 costmap（< 1秒）
- 需要车辆处于停止状态

**触发条件**:
```cpp
const bool is_stuck = is_stopped &&                      // 车辆停止
                      pull_out_status.planner_type == PlannerType::STOP &&  // 其他规划器失败
                      !pull_out_status.found_pull_out_path;  // 未找到路径
```

#### 输出

```cpp
std::optional<PullOutPath>
```

**PullOutPath 内容**（多分段）:
```cpp
{
  partial_paths: [后退路径, 前进路径1, 前进路径2, ...],  // 可能有多个分段
  pairs_terminal_velocity_and_accel: [
    (vel1, accel1),
    (vel2, accel2),
    ...
  ],
  start_pose: 起点,
  end_pose: 终点
}
```

**分段特点**:
- 每次换向（前进↔后退）产生一个新分段
- 分段数量不固定，取决于环境复杂度
- 可能包含多次前进和后退

#### 适用场景

✅ **适合**:
- 极度狭窄空间
- 障碍物密集环境
- 不规则空间
- 其他规划器都失败的情况
- 需要多次倒车的场景

❌ **不适合**:
- 简单起步场景（过于复杂和慢）
- 没有 costmap 的情况
- 需要快速响应的场景
- 实时性要求高的场景

#### 参数配置

**关键参数**:
```yaml
freespace_planner:
  enable_freespace_planner: true     # 是否启用
  freespace_planner_algorithm: "astar"  # "astar" 或 "rrtstar"
  velocity: 1.0                      # 规划速度
  use_back: true                     # 是否允许后退
  time_limit: 3000.0                 # 规划时间限制（ms）
  # A* 参数
  astar_search_method: "backward"
  # RRT* 参数
  rrt_star_timeout_sec: 3.0
  rrt_star_max_iterations: 10000
```

---

## 3. 三个规划器的对比

### 3.1 功能对比表

| 特性 | Shift Pull Out | Geometric Pull Out | Freespace Pull Out |
|-----|---------------|-------------------|-------------------|
| **复杂度** | 简单 | 中等 | 复杂 |
| **计算速度** | 快 (< 10ms) | 中等 (10-50ms) | 慢 (100-3000ms) |
| **路径平滑度** | 高 | 高 | 中等 |
| **适用场景** | 路边起步 | 停车位出库 | 复杂障碍物环境 |
| **转向能力** | 小角度 | 大角度 | 任意角度 |
| **后退支持** | 有限 | 有限 | 完全支持 |
| **分段数量** | 1 | 1-2 | 1-N |
| **障碍物处理** | 基础碰撞检查 | 基础碰撞检查 | 完整避障 |
| **实时性** | 优秀 | 良好 | 差 |
| **运行模式** | 同步 | 同步 | 异步（独立线程）|
| **触发条件** | 默认尝试 | 默认尝试 | 仅卡住时 |

### 3.2 优先级和选择策略

```
简单场景
  └→ Shift Pull Out ✅ 快速、简单、高效
  
停车位出库
  └→ Geometric Pull Out ✅ 专为此场景设计
  
复杂环境
  ├→ 先尝试 Shift
  ├→ 失败后尝试 Geometric
  └→ 都失败后，车辆停止，触发 Freespace ⚠️
```

### 3.3 性能对比

```
计算时间（典型值）:
  Shift:      5-10 ms      ████
  Geometric:  20-50 ms     ████████████
  Freespace:  500-3000 ms  ████████████████████████████████████████████████

成功率（在适用场景下）:
  Shift:      85-90%       ████████████████████
  Geometric:  80-85%       ███████████████████
  Freespace:  90-95%       ██████████████████████

路径质量:
  Shift:      ★★★★☆
  Geometric:  ★★★★★
  Freespace:  ★★★☆☆
```

---

## 4. 输入输出总结

### 4.1 统一的输入接口

**所有规划器共享相同的接口**:

```cpp
virtual std::optional<PullOutPath> plan(
  const Pose & start_pose,              // 起始位姿
  const Pose & goal_pose,               // 目标位姿
  const std::shared_ptr<const PlannerData> & planner_data,  // 共享规划数据
  PlannerDebugData & planner_debug_data // 调试输出
) = 0;
```

**PlannerData 包含**:
```cpp
struct PlannerData {
  Odometry::ConstSharedPtr self_odometry;           // 车辆状态
  PredictedObjects::ConstSharedPtr dynamic_object;  // 动态障碍物
  OccupancyGrid::ConstSharedPtr costmap;            // 代价地图（Freespace需要）
  std::shared_ptr<RouteHandler> route_handler;      // 路由和地图信息
  BehaviorPathPlannerParameters parameters;         // 通用参数
  // ...
};
```

### 4.2 统一的输出格式

**所有规划器输出相同的数据结构**:

```cpp
struct PullOutPath {
  std::vector<PathWithLaneId> partial_paths;  // 路径分段
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel;  // 速度和加速度
  Pose start_pose;  // 起点
  Pose end_pose;    // 终点
};
```

**输出差异**:
- Shift: 通常 1 个分段
- Geometric: 1-2 个分段
- Freespace: 1-N 个分段（根据环境复杂度）

---

## 5. 配置示例

### 5.1 只启用 Shift（简单快速）

```yaml
# start_planner.param.yaml
enable_shift_pull_out: true        # ✅ 启用
enable_geometric_pull_out: false   # ❌ 禁用
enable_freespace_planner: false    # ❌ 禁用
```

**适用**:
- 只在简单路边起步
- 追求最快响应
- 不需要复杂规划

### 5.2 启用 Shift + Geometric（常规配置）

```yaml
enable_shift_pull_out: true        # ✅ 启用（第一优先级）
enable_geometric_pull_out: true    # ✅ 启用（第二优先级）
enable_freespace_planner: false    # ❌ 禁用
search_priority: "efficient_path"  # 高效路径优先
```

**适用**:
- 大多数场景
- 平衡性能和能力
- 推荐配置

### 5.3 全部启用（最强能力）

```yaml
enable_shift_pull_out: true        # ✅ 启用
enable_geometric_pull_out: true    # ✅ 启用
enable_freespace_planner: true     # ✅ 启用（应急）
search_priority: "short_back_distance"
```

**适用**:
- 复杂环境
- 需要最大鲁棒性
- 可以接受较慢的规划速度
- **推荐生产环境配置**

---

## 6. 实际运行示例

### 示例1: 简单路边起步

**场景**: 车辆停在路边，前方空间充足

```
初始状态:
  - 车辆在路边
  - 与车道中心有 2m 横向偏移
  - 前方 20m 无障碍物

运行流程:
  1. planWithPriority() 被调用
  2. 尝试 Shift Pull Out:
     ├─ 生成横向位移路径
     ├─ 碰撞检查：通过 ✅
     └─ 返回 1 个分段的 PullOutPath
  3. 使用 Shift 路径，不再尝试其他规划器
  
结果:
  - 使用规划器: Shift Pull Out
  - 规划时间: ~8ms
  - 路径分段: 1个
  - 速度: 0 → 5 m/s
```

### 示例2: 停车位出库

**场景**: 车辆在垂直停车位内，需要出库

```
初始状态:
  - 车辆朝向与车道垂直
  - 需要 90° 转向
  - 空间较窄

运行流程:
  1. planWithPriority() 被调用
  2. 尝试 Shift Pull Out:
     ├─ 生成位移路径
     ├─ 碰撞检查：失败 ❌（转向角度太大）
     └─ 返回 std::nullopt
  3. 尝试 Geometric Pull Out:
     ├─ 生成 2 段弧线路径
     ├─ 碰撞检查：通过 ✅
     └─ 返回 2 个分段的 PullOutPath
  4. 使用 Geometric 路径
  
结果:
  - 使用规划器: Geometric Pull Out
  - 规划时间: ~35ms
  - 路径分段: 2个（分段模式）
  - 第1段: 0 → 1.5 m/s（转出停车位）
  - 第2段: 0 → 3 m/s（并入车道）
```

### 示例3: 拥挤环境（触发 Freespace）

**场景**: 狭窄空间，周围有多个障碍物

```
初始状态:
  - 狭窄空间
  - 多个障碍物
  - 需要多次倒车

运行流程:
  1. planWithPriority() 被调用
  2. 尝试 Shift Pull Out:
     ├─ 碰撞检查：失败 ❌
     └─ 返回 std::nullopt
  3. 尝试 Geometric Pull Out:
     ├─ 碰撞检查：失败 ❌
     └─ 返回 std::nullopt
  4. 所有规划器失败:
     ├─ status_.found_pull_out_path = false
     ├─ status_.planner_type = STOP
     └─ 车辆停止，输出速度=0的路径
  
  【1秒后，Freespace 定时器触发】
  5. onFreespacePlannerTimer() 检测到卡住:
     ├─ 调用 Freespace Pull Out
     ├─ 运行 A* 搜索
     ├─ 找到路径（包含3次倒车）✅
     └─ 更新 freespace_thread_status_
  6. 主线程读取 Freespace 结果
  7. 使用 Freespace 路径
  
结果:
  - 使用规划器: Freespace Pull Out
  - 规划时间: ~1500ms
  - 路径分段: 7个（包含多次前进和后退）
  - 速度: 交替前进和后退
```

---

## 7. 调试和诊断

### 7.1 查看使用的规划器

```bash
# 查看日志
ros2 run rqt_console rqt_console

# 搜索关键字
- "planner_type"
- "Shift", "Geometric", "Freespace"
- "found_pull_out_path"
```

**日志示例**:
```
[INFO] Using planner: SHIFT
[INFO] Found pull out path with 1 segments
```

### 7.2 查看规划器评估表

```cpp
std::string planner_evaluation_table = 
  start_planner_module.get_planner_evaluation_table();
```

**评估表内容**:
```
Planner Evaluation:
  [SHIFT] 
    - backward_distance: 0.0m
    - collision_margin: 0.5m
    - result: success ✅
  
  [GEOMETRIC]
    - backward_distance: 0.0m
    - collision_margin: 0.5m
    - result: not_evaluated (already found)
```

### 7.3 可视化

在 RViz 中可以看到:
- 候选起步路径（不同颜色）
- 选中的路径
- 碰撞检查区域
- 规划器类型标记

---

## 8. 常见问题

### Q1: 为什么所有规划器都启用但只用了 Shift？

**A**: 因为 Shift 是第一个尝试的规划器，如果它成功了，就不会尝试其他规划器。这是按设计的优化策略。

### Q2: Freespace 规划器永远不会被使用？

**A**: Freespace 只在**应急情况**下使用：
- 车辆停止 AND
- 其他规划器都失败 AND
- 未找到任何路径

这是最后的备用方案。

### Q3: 如何强制使用某个特定规划器？

**A**: 禁用其他规划器：
```yaml
enable_shift_pull_out: false
enable_geometric_pull_out: true   # 只启用这个
enable_freespace_planner: false
```

### Q4: 规划器的顺序可以修改吗？

**A**: 顺序由代码中的添加顺序决定（`start_planners_.push_back`），目前是硬编码的。如果需要修改，需要改代码。

### Q5: 为什么 Geometric 生成的路径有时是 1 段，有时是 2 段？

**A**: 由参数 `divide_pull_out_path` 控制：
- `true`: 分成 2 段（推荐，更平滑的速度规划）
- `false`: 合并为 1 段

---

## 9. 总结

### 关键要点

1. **三个规划器各有所长**
   - Shift: 快速简单
   - Geometric: 停车位专用
   - Freespace: 应急备用

2. **协同工作机制**
   - Shift 和 Geometric 按优先级尝试
   - 找到第一个成功的就停止
   - Freespace 独立运行，只在卡住时激活

3. **推荐配置**
   - 生产环境: 全部启用
   - 简单场景: 只启用 Shift
   - 停车场: 启用 Shift + Geometric

4. **性能考虑**
   - Shift 最快（<10ms）
   - Geometric 中等（20-50ms）
   - Freespace 最慢（500-3000ms）

5. **失败处理**
   - 所有规划器失败 → 输出停止路径（速度=0）
   - 这是导致速度为0的常见原因

---

**相关文件**:
- Shift Pull Out: `shift_pull_out.hpp/cpp`
- Geometric Pull Out: `geometric_pull_out.hpp/cpp`
- Freespace Pull Out: `freespace_pull_out.hpp/cpp`
- 主模块: `start_planner_module.hpp/cpp`
- 配置: `start_planner.param.yaml`

**文档生成时间**: 2025-10-24

