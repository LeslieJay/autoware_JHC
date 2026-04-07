# Behavior Path Planner 场景模块工作原理详解

## 重点总结

1. 各场景模块运行逻辑

- slot之间串行执行
- slot内部各组件已批准的模块按顺序修改路径，候选模块并行生成候选路径

## 📋 目录

1. [整体架构概览](#整体架构概览)
2. [核心组件说明](#核心组件说明)
3. [Slot机制详解](#slot机制详解)
4. [场景模块分类](#场景模块分类)
5. [路径生成完整流程](#路径生成完整流程)
6. [模块运行机制](#模块运行机制)
7. [配置文件说明](#配置文件说明)

---

## 整体架构概览

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                    BehaviorPathPlannerNode                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌───────────────────────────────────────────────────────────┐    │
│  │              PlannerManager (核心管理器)                  │    │
│  │                                                             │    │
│  │  ┌────────────────────────────────────────────────┐       │    │
│  │  │             Slot 1 (SubPlannerManager)         │       │    │
│  │  │  ┌──────────────────────────────────────────┐ │       │    │
│  │  │  │  [Start Planner]                         │ │       │    │
│  │  │  └──────────────────────────────────────────┘ │       │    │
│  │  └────────────────────────────────────────────────┘       │    │
│  │            ↓ (输出路径)                                   │    │
│  │  ┌────────────────────────────────────────────────┐       │    │
│  │  │             Slot 2 (SubPlannerManager)         │       │    │
│  │  │  ┌──────────────────────────────────────────┐ │       │    │
│  │  │  │  [Side Shift]                            │ │       │    │
│  │  │  │  [Avoidance by Lane Change]              │ │       │    │
│  │  │  │  [Static Obstacle Avoidance]             │ │       │    │
│  │  │  │  [Lane Change Left/Right]                │ │       │    │
│  │  │  │  [External Request Lane Change]          │ │       │    │
│  │  │  └──────────────────────────────────────────┘ │       │    │
│  │  └────────────────────────────────────────────────┘       │    │
│  │            ↓ (输出路径)                                   │    │
│  │  ┌────────────────────────────────────────────────┐       │    │
│  │  │             Slot 3 (SubPlannerManager)         │       │    │
│  │  │  ┌──────────────────────────────────────────┐ │       │    │
│  │  │  │  [Goal Planner]                          │ │       │    │
│  │  │  └──────────────────────────────────────────┘ │       │    │
│  │  └────────────────────────────────────────────────┘       │    │
│  │            ↓ (输出路径)                                   │    │
│  │  ┌────────────────────────────────────────────────┐       │    │
│  │  │             Slot 4 (SubPlannerManager)         │       │    │
│  │  │  ┌──────────────────────────────────────────┐ │       │    │
│  │  │  │  [Dynamic Obstacle Avoidance]            │ │       │    │
│  │  │  └──────────────────────────────────────────┘ │       │    │
│  │  └────────────────────────────────────────────────┘       │    │
│  │                                                             │    │
│  └───────────────────────────────────────────────────────────┘    │
│                              ↓                                     │
│           最终输出: /planning/behavior_planning/path_with_lane_id  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 核心组件说明

### 1. **PlannerManager（规划管理器）**

**位置：** `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp`

**核心职责：**
- 管理多个Slot（槽位）
- 生成root参考路径（从lanelet地图centerline）
- 协调各个场景模块的执行
- 输出最终路径

**关键代码：**
```cpp
BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  // 1. 生成参考路径（从地图读取速度限制）
  SlotOutput result_output = SlotOutput{
    getReferencePath(data),  // 这里读取地图的speed_limit
    false, false, false
  };
  
  // 2. 按顺序运行每个Slot
  for (auto & planner_manager_slot : planner_manager_slots_) {
    // 根据上游状态选择不同的传播模式
    if (result_output.is_upstream_failed_approved) {
      planner_manager_slot.propagateWithFailedApproved();
    } else if (result_output.is_upstream_waiting_approved) {
      result_output = planner_manager_slot.propagateWithWaitingApproved(data, result_output);
    } else if (result_output.is_upstream_candidate_exclusive) {
      result_output = planner_manager_slot.propagateWithExclusiveCandidate(data, result_output);
    } else {
      result_output = planner_manager_slot.propagateFull(data, result_output);
    }
  }
  
  return result_output.valid_output;  // 最终输出
}
```

### 2. **Slot（槽位）= SubPlannerManager**

**概念：**
- Slot是场景模块的容器，包含一组相关的场景模块
- 每个Slot按顺序处理输入路径，输出给下一个Slot
- 类似管道（Pipeline）的概念

**配置示例：**
```yaml
slots:
  - slot1   # 启动规划器
  - slot2   # 避障和换道模块
  - slot3   # 目标规划器
  - slot4   # 动态避障
```

**Slot内部的模块栈：**
```
┌─────────────────────────────────────┐
│          Slot (SubPlannerManager)   │
│                                     │
│  ┌───────────────────────────────┐ │
│  │   Approved Modules Stack      │ │  已批准的模块（串行运行）
│  │   - Module A (RUNNING)        │ │  
│  │   - Module B (RUNNING)        │ │  每个模块处理上一个模块的输出
│  └───────────────────────────────┘ │  
│            ↓                        │
│  ┌───────────────────────────────┐ │
│  │   Candidate Modules Stack     │ │  候选模块（并行运行）
│  │   - Module C (WAITING)        │ │  
│  │   - Module D (WAITING)        │ │  所有模块都接收approved stack的输出
│  └───────────────────────────────┘ │  一旦批准，移到approved stack
│                                     │
└─────────────────────────────────────┘
```

### 3. **Scene Module（场景模块）**

**功能：**
- 接收上一个模块的输出路径
- 根据场景需求修改路径
- 输出修改后的路径

**输入：**
- `PlannerData`: 车辆状态、地图、障碍物等
- `BehaviorModuleOutput`: 上一个模块的输出路径

**输出：**
- `BehaviorModuleOutput`: 修改后的路径
  - `path`: PathWithLaneId（包含速度信息）
  - `drivable_area_info`: 可行驶区域
  - `turn_signal_info`: 转向灯信息

---

## Slot机制详解

### Slot配置（default_preset.yaml + scene_module_manager.param.yaml）

**完整配置结构：**

```yaml
# scene_module_manager.param.yaml
slots:
  - slot1
  - slot2
  - slot3
  - slot4

slot1:
  - "start_planner"

slot2:
  - "side_shift"
  - "avoidance_by_lane_change"
  - "static_obstacle_avoidance"
  - "lane_change_left"
  - "lane_change_right"
  - "external_request_lane_change_left"
  - "external_request_lane_change_right"

slot3:
  - "goal_planner"

slot4:
  - "dynamic_obstacle_avoidance"
```

### Slot处理流程

```
Input: Reference Path (from lanelet centerline)
  ↓
┌─────────────────────────────────────────────┐
│ Slot 1: Start Planner                       │
│ - 检查是否需要启动规划                       │
│ - 如果车辆静止且需要启动，生成启动路径         │
│ - 否则，直接传递输入路径                     │
└─────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────┐
│ Slot 2: 避障和换道模块                       │
│ - 静态避障 (Static Obstacle Avoidance)       │
│ - 换道 (Lane Change Left/Right)             │
│ - 侧移 (Side Shift)                         │
│ - 通过换道避障 (Avoidance by Lane Change)   │
│                                             │
│ 工作方式：                                   │
│ 1. 所有模块检查是否需要执行                   │
│ 2. 需要执行的模块作为candidate并行运行        │
│ 3. 批准后的模块串行修改路径                   │
└─────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────┐
│ Slot 3: Goal Planner                        │
│ - 检查是否接近目标点                         │
│ - 生成停车路径                               │
│ - 设置目标点附近的速度（通常为0）             │
└─────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────┐
│ Slot 4: Dynamic Obstacle Avoidance          │
│ - 动态避障（实验性功能，默认false）           │
└─────────────────────────────────────────────┘
  ↓
Output: Final Path with Lane ID
  → /planning/behavior_planning/path_with_lane_id
```

---

## 场景模块分类

### 按功能分类

| 类别 | 模块名称 | 默认启用 | 作用 | Slot |
|-----|---------|---------|-----|------|
| **启动规划** | Start Planner | true | 车辆从静止启动时的路径规划 | Slot 1 |
| **避障** | Static Obstacle Avoidance | true | 避开静态障碍物（路边停车等） | Slot 2 |
| **避障** | Dynamic Obstacle Avoidance | false | 避开动态障碍物 | Slot 4 |
| **避障** | Avoidance by Lane Change | true | 通过换道方式避障 | Slot 2 |
| **换道** | Lane Change Left | true | 左换道 | Slot 2 |
| **换道** | Lane Change Right | true | 右换道 | Slot 2 |
| **换道** | External Request Lane Change | false | 外部请求换道 | Slot 2 |
| **目标规划** | Goal Planner | true | 接近目标点时的停车规划 | Slot 3 |
| **路径调整** | Side Shift | true | 横向偏移调整 | Slot 2 |

### 模块执行模式配置

每个模块都有以下配置参数：

```yaml
module_name:
  enable_rtc: false/true                                    # 是否需要RTC批准
  enable_simultaneous_execution_as_approved_module: true    # 作为approved时是否允许与其他模块同时运行
  enable_simultaneous_execution_as_candidate_module: true   # 作为candidate时是否允许与其他模块同时运行
```

**示例（Lane Change）：**
```yaml
lane_change_left:
  enable_rtc: false                                          # 不需要外部批准
  enable_simultaneous_execution_as_approved_module: true     # approved时可以与其他模块同时运行
  enable_simultaneous_execution_as_candidate_module: true    # candidate时可以与其他模块同时运行
```

---

## 路径生成完整流程

### 步骤1：生成参考路径（Root Reference Path）

**代码位置：** `planner_manager.cpp::getReferencePath()`

```cpp
BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  // 从当前车道获取中心线路径
  const auto reference_path = utils::getReferencePath(
    current_route_lanelet_->value(), data
  );
  
  return reference_path;
}
```

**调用链：**
```
getReferencePath()
  ↓
utils::getReferencePath() 
  ↓
getCenterLinePath()
  ↓
route_handler.getCenterLinePath()
  ↓
【关键】从lanelet读取speed_limit并设置到路径点
  → traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value()
  → p.point.longitudinal_velocity_mps = speed_limit
```

**参考路径特点：**
- 沿着lanelet centerline生成
- **速度来自地图的speed_limit标签**（km/h → m/s）
- 如果地图没有speed_limit，速度为0 ⚠️

### 步骤2：Slot 1 处理（Start Planner）

```cpp
// 检查是否需要启动规划
if (vehicle_is_stopped && need_start_planning) {
  // 生成从静止到行驶的过渡路径
  output.path = generatePullOutPath();
} else {
  // 直接传递输入路径
  output.path = input.path;
}
```

### 步骤3：Slot 2 处理（避障和换道）

**工作流程：**

1. **检查请求阶段**
   ```cpp
   for (auto & module : registered_modules) {
     if (module->isExecutionRequested()) {
       request_modules.push_back(module);
     }
   }
   ```

2. **运行候选模块（并行）**
   ```cpp
   for (auto & module : candidate_modules) {
     results[module->name()] = module->run(approved_output);
   }
   ```

3. **批准后串行运行**
   ```cpp
   BehaviorModuleOutput output = upstream_output;
   for (auto & module : approved_modules) {
     output = module->run(output);  // 串行处理
   }
   ```

**示例场景：静态避障**
```cpp
// Static Obstacle Avoidance Module
BehaviorModuleOutput run(BehaviorModuleOutput input) {
  // 1. 检测路径上的静态障碍物
  auto obstacles = detectStaticObstacles(input.path);
  
  // 2. 如果有障碍物，生成避障路径
  if (!obstacles.empty()) {
    auto avoid_path = generateAvoidancePath(input.path, obstacles);
    
    // 3. 修改速度（减速接近障碍物）
    for (auto & point : avoid_path.points) {
      point.point.longitudinal_velocity_mps = 
        calculateSafeVelocity(point);
    }
    
    output.path = avoid_path;
  } else {
    output.path = input.path;  // 无障碍物，直接传递
  }
  
  return output;
}
```

### 步骤4：Slot 3 处理（Goal Planner）

```cpp
// Goal Planner
if (isApproachingGoal()) {
  // 生成停车路径
  auto parking_path = generateParkingPath();
  
  // 设置停车速度（减速到0）
  for (auto & point : parking_path.points) {
    double distance_to_goal = calcDistanceToGoal(point);
    if (distance_to_goal < stop_distance) {
      point.point.longitudinal_velocity_mps = 0.0;  // 停车
    } else {
      // 减速曲线
      point.point.longitudinal_velocity_mps = 
        calcDecelerationVelocity(distance_to_goal);
    }
  }
  
  output.path = parking_path;
}
```

**这就是为什么接近目标点时速度为0！**

### 步骤5：Slot 4 处理（动态避障）

默认禁用，如果启用：
```cpp
// Dynamic Obstacle Avoidance
if (hasDynamicObstacles()) {
  auto avoid_path = generateDynamicAvoidancePath(input.path);
  output.path = avoid_path;
}
```

### 步骤6：路径后处理和发布

**代码位置：** `behavior_path_planner_node.cpp::run()`

```cpp
void BehaviorPathPlannerNode::run() {
  // 1. 运行planner manager
  const auto output = planner_manager_->run(planner_data_);
  
  // 2. 获取最终路径
  const auto path = getPath(output, planner_data_, planner_manager_);
  
  // 3. 路径重采样（保持速度信息）
  const auto resampled_path = utils::resamplePathWithSpline(
    *path, 
    planner_data->parameters.output_path_interval,
    keepInputPoints(module_status_ptr_vec)
  );
  
  // 4. 发布路径
  path_publisher_->publish(*resampled_path);
  // 话题: /planning/behavior_planning/path_with_lane_id
}
```

---

## 模块运行机制

### Approved Modules（已批准模块）- 串行运行

```
Input Path
  ↓
Module A (RUNNING) → Modified Path A
  ↓
Module B (RUNNING) → Modified Path B
  ↓
Module C (RUNNING) → Modified Path C
  ↓
Output Path (最终输出)
```

**特点：**
- 每个模块处理上一个模块的输出
- 按顺序执行，保证路径连续性
- 已经得到批准，可以修改路径

### Candidate Modules（候选模块）- 并行运行

```
Approved Modules Output
  ↓
  ├─→ Module X (WAITING) → Candidate Path X
  ├─→ Module Y (WAITING) → Candidate Path Y
  └─→ Module Z (WAITING) → Candidate Path Z
  
所有候选路径可视化显示，等待批准
一旦批准，移到Approved Stack
```

**特点：**
- 所有候选模块接收相同的输入（approved modules的输出）
- 并行运行，各自生成候选路径
- 仅用于可视化，不影响实际行驶路径
- 需要批准（RTC或自动批准）后才能生效

### 模块状态转换

```
     ┌────────────┐
     │   IDLE     │  模块空闲
     └──────┬─────┘
            │ 检测到需要执行
            ↓
     ┌────────────┐
     │  RUNNING   │  模块运行中（Candidate）
     └──────┬─────┘
            │ 收到批准
            ↓
     ┌────────────┐
     │  RUNNING   │  模块运行中（Approved）
     └──────┬─────┘
            │ 任务完成
            ↓
  ┌─────────┴──────────┐
  ↓                    ↓
┌──────────┐    ┌──────────┐
│ SUCCESS  │    │ FAILURE  │
└──────────┘    └──────────┘
  模块删除         模块删除
```

---

## 配置文件说明

### 1. default_preset.yaml

**位置：** `src/launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml`

**作用：** 控制哪些模块被启动

```yaml
# behavior path modules
- arg:
    name: launch_static_obstacle_avoidance
    default: "true"   # 启用静态避障

- arg:
    name: launch_lane_change_left_module
    default: "true"   # 启用左换道

- arg:
    name: launch_goal_planner_module
    default: "true"   # 启用目标规划器

- arg:
    name: launch_dynamic_obstacle_avoidance
    default: "false"  # 禁用动态避障（实验性）
```

### 2. scene_module_manager.param.yaml

**位置：** `src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/scene_module_manager.param.yaml`

**作用：** 配置Slot结构和模块行为

```yaml
slots:
  - slot1  # 包含哪些模块
  - slot2
  - slot3
  - slot4

slot2:
  - "static_obstacle_avoidance"
  - "lane_change_left"
  - "lane_change_right"

# 每个模块的执行配置
lane_change_left:
  enable_rtc: false                                    # RTC批准
  enable_simultaneous_execution_as_approved_module: true   # 串行/并行
  enable_simultaneous_execution_as_candidate_module: true
```

### 3. behavior_path_planner.param.yaml

**位置：** `src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml`

**作用：** 基础参数配置

```yaml
planning_hz: 10.0                    # 规划频率 10Hz
backward_path_length: 5.0            # 向后路径长度
forward_path_length: 300.0           # 向前路径长度
input_path_interval: 2.0             # 输入路径采样间隔
output_path_interval: 2.0            # 输出路径采样间隔
```

---

## 速度设置的关键位置

### 1. 初始速度（来自地图）⭐⭐⭐⭐⭐

**位置：** `route_handler.cpp::getCenterLinePath()`

```cpp
// 行 1588-1596
const float speed_limit =
  static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value());

p.point.longitudinal_velocity_mps = speed_limit;
```

**这是速度的源头！如果地图没有speed_limit标签，这里就是0。**

### 2. 场景模块修改速度

各个场景模块可能会修改速度：

**Goal Planner（目标规划器）：**
```cpp
// 接近目标点，设置速度为0
for (auto & point : reference_path.points) {
  point.point.longitudinal_velocity_mps = 0.0;
}
```

**Static Obstacle Avoidance（静态避障）：**
```cpp
// 遇到障碍物，降低速度
point.point.longitudinal_velocity_mps = calculateSafeVelocity(obstacle_distance);
```

**Lane Change（换道）：**
```cpp
// 换道时可能调整速度
point.point.longitudinal_velocity_mps = adjustVelocityForLaneChange(curvature);
```

### 3. 路径重采样保持速度

**位置：** `behavior_path_planner_node.cpp::getPath()`

```cpp
// 行 724-726
const auto resampled_path = utils::resamplePathWithSpline(
  *path, 
  planner_data->parameters.output_path_interval,
  keepInputPoints(module_status_ptr_vec)
);
```

重采样过程会插值速度信息，保持路径的速度连续性。

---

## 调试和监控

### 查看当前激活的模块

```bash
# 查看场景模块状态
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status

# 输出示例：
# module_name: 'lane_change_left'
# status: 'RUNNING'
# is_waiting_approval: false
```

### 查看各Slot的输出

```bash
# Approved modules路径
ros2 topic list | grep approved

# Candidate modules路径
ros2 topic list | grep candidate

# 每个模块的候选路径
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_candidate/static_obstacle_avoidance
```

### 查看停止原因

```bash
# 如果路径速度为0，查看原因
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons
```

### 完整调试脚本

使用之前创建的脚本：
```bash
./debug_bpp_velocity.sh
```

---

## 典型场景示例

### 场景1：正常行驶（无障碍物）

```
1. Reference Path (从地图) → 速度 = 8.33 m/s (30 km/h)
   ↓
2. Slot 1 (Start Planner) → 检查：车辆已运动 → 直接传递
   ↓
3. Slot 2 (避障/换道) → 检查：无障碍物 → 直接传递
   ↓
4. Slot 3 (Goal Planner) → 检查：距离目标点>20m → 直接传递
   ↓
5. 最终输出 → 速度保持 8.33 m/s
```

### 场景2：遇到路边停车车辆

```
1. Reference Path → 速度 = 8.33 m/s
   ↓
2. Slot 1 → 直接传递
   ↓
3. Slot 2:
   - Static Obstacle Avoidance 检测到障碍物
   - 作为candidate运行，生成避障路径
   - 批准后移到approved stack
   - 修改路径：横向偏移避开障碍物
   - 修改速度：接近障碍物时减速到 4.0 m/s
   ↓
4. Slot 3 → 检查：距离目标点>20m → 直接传递
   ↓
5. 最终输出 → 路径绕过障碍物，速度4.0 m/s (避障区域)
```

### 场景3：接近目标点

```
1. Reference Path → 速度 = 8.33 m/s
   ↓
2. Slot 1 → 直接传递
   ↓
3. Slot 2 → 直接传递
   ↓
4. Slot 3 (Goal Planner):
   - 检测：距离目标点 < 10m
   - 生成停车路径
   - 设置速度：
     * 5-10m: 逐渐减速
     * <5m: 速度 = 0.0 m/s
   ↓
5. 最终输出 → 停车路径，目标点附近速度为0
```

### 场景4：换道避障组合

```
1. Reference Path → 速度 = 13.89 m/s (50 km/h)
   ↓
2. Slot 1 → 直接传递
   ↓
3. Slot 2:
   a) Static Obstacle Avoidance 检测到障碍物
      - 尝试生成避障路径（横向偏移）
      - 发现空间不足，请求换道
   
   b) Lane Change Left 检测到需要换道
      - 作为candidate运行
      - 生成换道路径
      - 批准后移到approved stack
      - 执行换道，修改速度适应换道曲率
   
   c) Static Obstacle Avoidance (approved)
      - 在新车道上继续避障
   ↓
4. Slot 3 → 直接传递
   ↓
5. 最终输出 → 换道+避障组合路径
```

---

## 总结

### 关键要点

1. **路径初始速度来自地图**
   - `route_handler.getCenterLinePath()` 从lanelet读取 `speed_limit`
   - 如果地图没有设置，速度为0 ⚠️

2. **Slot是串行处理**
   - Slot 1 → Slot 2 → Slot 3 → Slot 4
   - 每个Slot的输出是下一个Slot的输入

3. **Slot内部：Approved串行，Candidate并行**
   - Approved modules: 串行修改路径
   - Candidate modules: 并行生成候选路径

4. **速度可能被修改的地方**
   - Goal Planner: 接近目标点设为0
   - Obstacle Avoidance: 遇障碍物减速
   - Lane Change: 换道时调整速度

5. **配置灵活性**
   - `default_preset.yaml`: 启用/禁用模块
   - `scene_module_manager.param.yaml`: 配置Slot和模块行为
   - `behavior_path_planner.param.yaml`: 基础参数

### 问题排查流程

如果路径速度为0：

```bash
1. 检查地图speed_limit → ./debug_bpp_velocity.sh
2. 检查Goal Planner → 是否接近目标点？
3. 检查场景模块 → 哪个模块修改了速度？
4. 查看日志 → 是否有"out of route"？
```

---

## 参考文档

- [behavior_path_planner速度为零的原因分析.md](./behavior_path_planner速度为零的原因分析.md)
- [BPP速度为零_快速诊断.md](./BPP速度为零_快速诊断.md)
- [lanelet地图速度限制参数说明.md](./lanelet地图速度限制参数说明.md)
- [官方设计文档](src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/docs/behavior_path_planner_manager_design.md)

## 代码位置索引

| 功能 | 文件 | 关键函数/行号 |
|-----|------|-------------|
| 主流程 | `planner_manager.cpp` | `run()` 103行 |
| 参考路径生成 | `planner_manager.cpp` | `getReferencePath()` 272行 |
| 速度从地图读取 | `route_handler.cpp` | `getCenterLinePath()` 1588行 |
| Slot配置 | `planner_manager.cpp` | `configureModuleSlot()` 75行 |
| 路径发布 | `behavior_path_planner_node.cpp` | `run()` 372行 |
| 路径重采样 | `path_utils.cpp` | `resamplePathWithSpline()` 62行 |

