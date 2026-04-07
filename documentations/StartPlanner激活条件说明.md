# StartPlanner 激活条件详解

## 核心问题

**如果车辆在车道中间，start_planner 会输出吗？起什么作用？**

## 简短回答

**❌ 不会输出！**

如果车辆已经在车道中间（中心线附近），start_planner **不会被激活**，也不会输出任何路径规划。这是按设计的行为，因为车辆已经处于正常行驶位置，不需要起步规划。

---

## 1. 激活条件详解

### 1.1 核心判断函数

**代码位置**: `start_planner_module.cpp:338-363`

```cpp
bool StartPlannerModule::isExecutionRequested() const
{
  // 如果模块已经在运行，继续执行
  if (isModuleRunning()) {
    return true;
  }

  // ⚠️ 如果满足以下任一条件，不请求执行（返回 false）：
  // - 车辆位姿在中心线上（车道中间）
  // - 车辆已经到达起始位置附近
  // - 车辆已经到达目标位置
  // - 车辆正在移动中
  if (
    isCurrentPoseOnEgoCenterline() ||      // ⭐ 在车道中间
    isCloseToOriginalStartPose() || 
    hasArrivedAtGoal() ||
    isMoving()) {
    return false;  // ❌ 不激活 start_planner
  }

  // 检查目标是否在车辆后方（同一路段内）
  if (isGoalBehindOfEgoInSameRouteSegment()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, 
      "Start plan for a backward goal is not supported now");
    return false;
  }

  return true;  // ✅ 可以激活 start_planner
}
```

### 1.2 判断是否在车道中间

**代码位置**: `start_planner_module.cpp:370-380`

```cpp
bool StartPlannerModule::isCurrentPoseOnEgoCenterline() const
{
  const auto & lanelet_map_ptr = planner_data_->route_handler->getLaneletMapPtr();
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const lanelet::ConstLanelets current_lanes = utils::getCurrentLanes(planner_data_);
  
  // 计算车辆到车道中心线的横向距离
  const double lateral_distance_to_center_lane =
    lanelet::utils::getArcCoordinatesOnEgoCenterline(
      current_lanes, current_pose, lanelet_map_ptr).distance;

  // ⭐ 如果横向距离小于阈值，认为在中心线上
  return std::abs(lateral_distance_to_center_lane) < 
         parameters_->th_distance_to_middle_of_the_road;
}
```

**判断逻辑**:
1. 获取当前车道的中心线
2. 计算车辆到中心线的横向距离（perpendicular distance）
3. 如果距离 < `th_distance_to_middle_of_the_road`，认为在车道中间
4. 返回 true → 不激活 start_planner

---

## 2. 关键参数配置

### 2.1 th_distance_to_middle_of_the_road

**参数位置**: `start_planner.param.yaml:22`

```yaml
th_distance_to_middle_of_the_road: 0.5  # 单位：米
```

**含义**: 
- 判断车辆是否在"车道中间"的横向距离阈值
- 如果车辆距离中心线 < 0.5m，认为在车道中间

**默认值**: 0.5 米

**影响**:
```
横向距离 < 0.5m  → 认为在车道中间 → ❌ 不激活 start_planner
横向距离 ≥ 0.5m  → 认为不在车道中间 → ✅ 可能激活 start_planner
```

**调整建议**:
- **增大参数值**（例如 1.0m）：更严格，只有距离中心线更远才激活
- **减小参数值**（例如 0.2m）：更宽松，即使靠近中心线也可能激活
- 默认 0.5m 对大多数场景合理

### 2.2 其他相关参数

```yaml
th_stopped_velocity: 0.01   # 停止速度阈值 [m/s]
th_stopped_time: 1.0        # 停止时间阈值 [s]
th_arrived_distance: 1.0    # 到达距离阈值 [m]
```

---

## 3. 完整的激活条件

### 3.1 必须同时满足的条件

start_planner **被激活**需要满足：

✅ **位置条件**:
- 车辆**不在**车道中心线上（横向偏移 ≥ 0.5m）
- 车辆**不在**原始起始位置附近
- 车辆**未到达**目标位置

✅ **运动状态条件**:
- 车辆处于**停止状态**（速度 < 0.01 m/s）

✅ **路由条件**:
- 目标点**不在**车辆后方（同一路段内）

❌ **任一条件不满足** → start_planner 不激活

### 3.2 各条件判断函数

#### a) `isCurrentPoseOnEgoCenterline()` - 是否在中心线上

```cpp
bool isCurrentPoseOnEgoCenterline() const
{
  const double lateral_distance_to_center_lane = 
    calcLateralDistanceToCenterline();
  
  return std::abs(lateral_distance_to_center_lane) < 
         parameters_->th_distance_to_middle_of_the_road;
}
```

**返回 true** → 车辆在车道中间 → **不激活**

#### b) `isCloseToOriginalStartPose()` - 是否靠近原始起点

```cpp
bool isCloseToOriginalStartPose() const
{
  const Pose start_pose = 
    planner_data_->route_handler->getOriginalStartPose();
  
  const double distance = autoware_utils::calc_distance2d(
    start_pose.position, 
    planner_data_->self_odometry->pose.pose.position);
  
  return distance > parameters_->th_arrived_distance;
}
```

**返回 true** → 还在起点附近 → **不激活**

#### c) `hasArrivedAtGoal()` - 是否已到达目标

```cpp
bool hasArrivedAtGoal() const
{
  const Pose goal_pose = planner_data_->route_handler->getGoalPose();
  
  const double distance = autoware_utils::calc_distance2d(
    goal_pose.position, 
    planner_data_->self_odometry->pose.pose.position);
  
  return distance < parameters_->th_arrived_distance;
}
```

**返回 true** → 已到达目标 → **不激活**

#### d) `isMoving()` - 是否正在移动

```cpp
bool isMoving() const
{
  return utils::l2Norm(
    planner_data_->self_odometry->twist.twist.linear) >= 
    parameters_->th_stopped_velocity;
}
```

**返回 true** → 车辆正在移动 → **不激活**

---

## 4. 典型场景分析

### 场景1: 车辆在车道中间停止

```
状态:
  - 位置: 车道中心线上（横向偏移 0.1m）
  - 速度: 0 m/s（停止）
  - 目标: 前方 100m

判断:
  ✅ isCurrentPoseOnEgoCenterline() = true  (0.1m < 0.5m)
  ❌ isCloseToOriginalStartPose() = false
  ❌ hasArrivedAtGoal() = false
  ❌ isMoving() = false

结果:
  ❌ isExecutionRequested() = false
  → start_planner 不激活
  → 使用其他规划器（如 lane_following）
```

**为什么不激活？**
- 车辆已经在正常行驶位置（车道中间）
- 不需要起步规划，直接正常行驶即可
- 避免不必要的规划计算

### 场景2: 车辆在路边停止

```
状态:
  - 位置: 路边（横向偏移 1.5m）
  - 速度: 0 m/s（停止）
  - 目标: 前方 100m

判断:
  ❌ isCurrentPoseOnEgoCenterline() = false  (1.5m > 0.5m)
  ❌ isCloseToOriginalStartPose() = false
  ❌ hasArrivedAtGoal() = false
  ❌ isMoving() = false

结果:
  ✅ isExecutionRequested() = true
  → start_planner 激活
  → 规划从路边并入车道的起步路径
```

**为什么激活？**
- 车辆不在正常行驶位置（路边）
- 需要起步规划来并入车道
- 使用 Shift/Geometric/Freespace 规划器

### 场景3: 车辆在车道中间行驶

```
状态:
  - 位置: 车道中心线上（横向偏移 0.2m）
  - 速度: 5 m/s（行驶中）
  - 目标: 前方 100m

判断:
  ✅ isCurrentPoseOnEgoCenterline() = true
  ✅ isMoving() = true

结果:
  ❌ isExecutionRequested() = false
  → start_planner 不激活
  → 使用 lane_following 规划器
```

**为什么不激活？**
- 车辆正在正常行驶
- 不需要起步规划
- 两个条件都满足不激活

### 场景4: 停车位内停止

```
状态:
  - 位置: 停车位内（横向偏移 3.0m）
  - 速度: 0 m/s（停止）
  - 朝向: 与车道垂直

判断:
  ❌ isCurrentPoseOnEgoCenterline() = false  (3.0m > 0.5m)
  ❌ isCloseToOriginalStartPose() = false
  ❌ hasArrivedAtGoal() = false
  ❌ isMoving() = false

结果:
  ✅ isExecutionRequested() = true
  → start_planner 激活
  → 可能使用 Geometric 或 Freespace 规划器
```

---

## 5. 模块切换机制

### 5.1 模块优先级

在 behavior_path_planner 中，多个模块可能同时存在，按优先级选择：

```
高优先级
  ↓
1. start_planner         (起步场景)
2. goal_planner          (接近目标场景)
3. lane_change           (换道场景)
4. avoidance             (避障场景)
5. lane_following        (正常跟车，默认)
  ↓
低优先级
```

### 5.2 车辆在车道中间时的规划器选择

```
车辆在车道中间
  ↓
start_planner.isExecutionRequested() = false  (不激活)
  ↓
检查其他模块的激活条件
  ↓
  ├─ 需要换道? → lane_change 模块
  ├─ 接近目标? → goal_planner 模块
  ├─ 需要避障? → avoidance 模块
  └─ 否则 → lane_following 模块（默认）
```

**结果**: 
- start_planner 不输出
- 由 **lane_following** 或其他合适的模块接管

### 5.3 从 start_planner 切换到其他模块

**切换时机**:

```cpp
// 当 start_planner 完成起步任务后
bool StartPlannerModule::canTransitSuccessState()
{
  // 条件1: 已经出发（开始前进）
  if (!status_.has_departed) return false;
  
  // 条件2: 已经到达起步路径的终点
  if (!hasReachedPullOutEnd()) return false;
  
  // ✅ 切换到成功状态
  return true;
}
```

**切换后**:
- start_planner 状态变为 SUCCESS
- 其他模块（如 lane_following）接管
- 车辆继续正常行驶

---

## 6. 实际运行流程

### 流程图

```
系统启动 / 新路由
    ↓
behavior_path_planner 检查各模块
    ↓
start_planner.isExecutionRequested()
    ↓
    ├─ 车辆在车道中间？
    │   ├─ YES → 返回 false ❌
    │   │          ↓
    │   │       不激活 start_planner
    │   │          ↓
    │   │       使用 lane_following
    │   │          ↓
    │   └────→  正常行驶
    │
    └─ 车辆在路边/停车位？
        ├─ YES → 返回 true ✅
        │          ↓
        │       激活 start_planner
        │          ↓
        │       规划起步路径
        │          ↓
        │       执行起步
        │          ↓
        │       到达车道中间
        │          ↓
        │       start_planner 完成
        │          ↓
        └────→  切换到 lane_following
```

---

## 7. 调试和诊断

### 7.1 如何查看 start_planner 是否激活

**方法1: 查看日志**

```bash
ros2 run rqt_console rqt_console

# 搜索关键字
- "isExecutionRequested"
- "start_planner"
- "ModuleStatus"
```

**日志示例**:
```
[INFO] [start_planner]: isExecutionRequested = false (on centerline)
[INFO] [start_planner]: Current pose is on ego centerline, skip start planning
```

**方法2: 查看话题**

```bash
# 查看激活的模块
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path

# 查看模块状态
ros2 topic echo /planning/scenario_planning/module_status
```

### 7.2 查看横向偏移距离

**添加调试日志**（修改代码）:

```cpp
bool StartPlannerModule::isCurrentPoseOnEgoCenterline() const
{
  // ... 计算距离 ...
  
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 1000,
    "[DEBUG] Lateral distance to centerline: %.2f m (threshold: %.2f m)",
    std::abs(lateral_distance_to_center_lane),
    parameters_->th_distance_to_middle_of_the_road
  );
  
  return std::abs(lateral_distance_to_center_lane) < 
         parameters_->th_distance_to_middle_of_the_road;
}
```

**输出示例**:
```
[INFO] Lateral distance to centerline: 0.15 m (threshold: 0.50 m)
→ 在车道中间，不激活 start_planner

[INFO] Lateral distance to centerline: 1.20 m (threshold: 0.50 m)
→ 不在车道中间，可能激活 start_planner
```

### 7.3 可视化检查

在 RViz 中查看:
- **车辆位置**：相对于车道中心线
- **当前车道**：中心线可视化
- **激活的模块**：显示当前使用的规划器

---

## 8. 参数调优建议

### 8.1 调整横向距离阈值

**场景A: 希望更严格判断（只在完全偏离时才激活）**

```yaml
th_distance_to_middle_of_the_road: 1.0  # 增大到 1.0m
```

**效果**:
- 横向偏移 < 1.0m → 认为在车道中间 → 不激活
- 横向偏移 ≥ 1.0m → 认为不在车道中间 → 激活

**适用**:
- 希望减少 start_planner 的激活次数
- 车道较宽，允许更大的横向偏差

**场景B: 希望更敏感（稍微偏离就激活）**

```yaml
th_distance_to_middle_of_the_road: 0.2  # 减小到 0.2m
```

**效果**:
- 横向偏移 < 0.2m → 认为在车道中间 → 不激活
- 横向偏移 ≥ 0.2m → 认为不在车道中间 → 激活

**适用**:
- 希望更频繁地使用 start_planner
- 对中心线对齐要求很高

### 8.2 推荐配置

**标准配置**（默认）:
```yaml
th_distance_to_middle_of_the_road: 0.5  # 平衡性能和鲁棒性
```

**宽松配置**（减少激活）:
```yaml
th_distance_to_middle_of_the_road: 1.0
```

**严格配置**（增加激活）:
```yaml
th_distance_to_middle_of_the_road: 0.3
```

---

## 9. 常见问题

### Q1: 车辆在车道边缘（不在中间），为什么 start_planner 还是不激活？

**A**: 检查其他条件：
- 车辆是否正在移动？（`isMoving()` = true）
- 车辆是否已到达目标？（`hasArrivedAtGoal()` = true）
- 任一条件为 true 都会阻止激活

### Q2: 我想强制激活 start_planner，怎么办？

**A**: 
1. **不建议强制激活**，这违反了设计意图
2. 如果真的需要，可以修改 `th_distance_to_middle_of_the_road` 为很小的值（如 0.01）
3. 或者修改代码注释掉 `isCurrentPoseOnEgoCenterline()` 检查

### Q3: start_planner 和 lane_following 如何协作？

**A**: 
- **start_planner**: 负责从**非标准位置**（路边、停车位）起步到车道中间
- **lane_following**: 负责在**车道中间**正常跟随车道行驶
- **协作**: start_planner 完成起步后自动切换到 lane_following

### Q4: 车辆在车道中间突然停止，会发生什么？

**A**:
```
车辆在车道中间停止
  ↓
isCurrentPoseOnEgoCenterline() = true
  ↓
start_planner 不激活
  ↓
lane_following 继续工作
  ↓
生成停止路径或等待障碍物清除
  ↓
继续正常行驶
```

- **不会**激活 start_planner
- 由 lane_following 或其他模块处理

---

## 10. 总结

### 核心要点

1. **车辆在车道中间时，start_planner 不会激活**
   - 判断标准：横向距离 < 0.5m（默认）
   - 目的：避免不必要的起步规划

2. **start_planner 的职责**
   - 从**非正常位置**起步到**车道中间**
   - 不负责正常行驶中的路径规划

3. **激活条件严格**
   - 必须满足：不在中间 + 停止 + 未到目标 + 目标在前方
   - 任一条件不满足都不激活

4. **模块协作机制**
   - start_planner → 起步阶段
   - lane_following → 正常行驶阶段
   - 自动切换，无缝衔接

5. **参数可调**
   - `th_distance_to_middle_of_the_road`: 控制"车道中间"的定义
   - 默认 0.5m 对大多数场景合理

---

## 11. 相关代码位置

| 功能 | 文件 | 函数/行号 |
|-----|------|----------|
| 执行请求判断 | `start_planner_module.cpp` | `isExecutionRequested()` (338-363) |
| 中心线位置判断 | `start_planner_module.cpp` | `isCurrentPoseOnEgoCenterline()` (370-380) |
| 参数定义 | `data_structs.hpp` | `th_distance_to_middle_of_the_road` (109) |
| 参数加载 | `data_structs.cpp` | 第 39-40 行 |
| 默认配置 | `start_planner.param.yaml` | 第 22 行 |

---

**文档生成时间**: 2025-10-24  
**关键参数**: `th_distance_to_middle_of_the_road = 0.5m`

