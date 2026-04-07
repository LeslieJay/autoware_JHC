# StartPlanner 模块输入输出说明

## 概述

`StartPlanner` 模块是 `behavior_path_planner` 的一个子组件，负责生成车辆从静止状态（停车位、路边等）起步的路径规划。该模块继承自 `SceneModuleInterface`，实现了标准的场景模块接口。

---

## 1. 模块定义

### 类继承关系
```cpp
StartPlannerModule : public SceneModuleInterface
```

### 主要接口方法
```cpp
BehaviorModuleOutput plan() override;                  // 主规划函数
BehaviorModuleOutput planWaitingApproval() override;   // 等待批准时的规划
CandidateOutput planCandidate() const override;        // 候选路径规划
void updateData() override;                            // 更新数据
bool isExecutionRequested() const override;            // 是否请求执行
bool isExecutionReady() const override;                // 是否准备好执行
```

---

## 2. 输入数据 (Inputs)

### 2.1 核心输入：PlannerData

`PlannerData` 是从主节点传入的共享数据结构，包含以下关键信息：

#### 2.1.1 车辆状态信息
```cpp
Odometry::ConstSharedPtr self_odometry;              // 车辆位姿和速度
AccelWithCovarianceStamped::ConstSharedPtr self_acceleration;  // 车辆加速度
```
**作用**:
- 获取当前车辆位置、朝向
- 判断车辆是否静止
- 计算相对距离和速度

#### 2.1.2 环境感知信息
```cpp
PredictedObjects::ConstSharedPtr dynamic_object;     // 动态障碍物
OccupancyGrid::ConstSharedPtr occupancy_grid;        // 占用栅格地图
OccupancyGrid::ConstSharedPtr costmap;               // 代价地图
```
**作用**:
- 动态障碍物检测和碰撞避免
- 自由空间规划（freespace planning）
- 安全性检查

#### 2.1.3 路由和地图信息
```cpp
std::shared_ptr<RouteHandler> route_handler;         // 路由处理器
```
**作用**:
- 获取目标点信息
- 获取车道（lanelet）信息
- 获取参考路径
- 检测新路由

**关键方法**:
```cpp
// 获取中心线路径
route_handler->getCenterLinePath(lanes, start_s, end_s, use_exact);

// 获取路由头信息
route_handler->getRouteHeader();

// 获取目标位姿
route_handler->getGoalPose();
```

#### 2.1.4 其他上下文信息
```cpp
OperationModeState::ConstSharedPtr operation_mode;   // 操作模式（手动/自动）
PathWithLaneId::SharedPtr prev_output_path;          // 前一次输出路径
BehaviorPathPlannerParameters parameters;            // 通用规划参数
```

### 2.2 模块参数：StartPlannerParameters

通过构造函数传入的模块特定参数：

```cpp
std::shared_ptr<StartPlannerParameters> parameters_;
```

**主要参数包括**:
- `enable_shift_pull_out`: 是否启用位移起步规划器
- `enable_geometric_pull_out`: 是否启用几何起步规划器
- `enable_freespace_planner`: 是否启用自由空间规划器
- `maximum_deceleration_for_stop`: 最大减速度
- `maximum_jerk_for_stop`: 最大加速度变化率
- `pull_out_velocity`: 起步速度
- `maximum_longitudinal_deviation`: 最大纵向偏差
- `maximum_lateral_deviation`: 最大横向偏差
- 安全检查参数（`safety_check_params`）
- 碰撞检查参数
- 等待和超时参数

### 2.3 上一个模块的输出

```cpp
// 从基类方法获取
getPreviousModuleOutput().reference_path;  // 上一个模块的参考路径
getPreviousModuleOutput().path;            // 上一个模块的路径
```

**作用**:
- 作为起步路径的基准
- 用于路径拼接和连续性保证

---

## 3. 输出数据 (Outputs)

### 3.1 主要输出：BehaviorModuleOutput

`plan()` 方法返回 `BehaviorModuleOutput` 结构体：

```cpp
struct BehaviorModuleOutput
{
  PathWithLaneId path;                      // 规划的路径
  PathWithLaneId reference_path;            // 参考路径
  TurnSignalInfo turn_signal_info;          // 转向信号信息
  std::optional<PoseWithUuidStamped> modified_goal;  // 修改后的目标点
  DrivableAreaInfo drivable_area_info;      // 可行驶区域信息
};
```

#### 3.1.1 路径 (path)

**类型**: `PathWithLaneId`

**内容**:
```cpp
struct PathWithLaneId
{
  Header header;
  std::vector<PathPointWithLaneId> points;  // 路径点数组
  PathFootprint left_bound;                 // 左边界
  PathFootprint right_bound;                // 右边界
};
```

**路径点结构**:
```cpp
struct PathPointWithLaneId
{
  PathPoint point;              // 位置、速度、加速度等
    ├── Pose pose;              // 位姿（位置+朝向）
    ├── float longitudinal_velocity_mps;  // 纵向速度 [m/s]
    ├── float lateral_velocity_mps;       // 横向速度 [m/s]
    ├── float heading_rate_rps;           // 航向角速率 [rad/s]
    └── bool is_final;                    // 是否为最终点
  std::vector<int64_t> lane_ids;  // 车道ID
};
```

**路径生成过程**:

1. **向后路径阶段** (如果需要后退)
   - 返回 `status_.backward_path`
   - 速度为负值或0

2. **起步路径阶段**
   - 根据规划器类型生成：
     - Shift Pull Out（位移起步）
     - Geometric Pull Out（几何起步）
     - Freespace Pull Out（自由空间起步）
   - 路径包含多个分段（`partial_paths`）
   - 每个点包含位姿和速度信息

3. **停止路径**（安全检查失败时）
   - 所有点的速度设为 0.0
   - 保持当前位置或减速停止

**关键代码位置**:
```cpp
// start_planner_module.cpp:726
output.path = path;
```

#### 3.1.2 参考路径 (reference_path)

**来源**: 继承自上一个模块的输出
```cpp
output.reference_path = getPreviousModuleOutput().reference_path;
```

**作用**:
- 提供道路中心线参考
- 用于后续模块的路径规划基准

#### 3.1.3 转向信号信息 (turn_signal_info)

**类型**: `TurnSignalInfo`

**计算方法**:
```cpp
output.turn_signal_info = calcTurnSignalInfo();
```

**包含信息**:
- 转向灯命令（左转/右转/无）
- 转向信号的起始和结束位置
- 转向信号的持续时间

**判断逻辑**:
- 根据起步路径的横向偏移方向
- 考虑车道变化
- 考虑停车位出口方向

#### 3.1.4 可行驶区域信息 (drivable_area_info)

**设置方法**:
```cpp
setDrivableAreaInfo(output);
```

**包含信息**:
```cpp
struct DrivableAreaInfo
{
  std::vector<DrivableLanes> drivable_lanes;  // 可行驶车道
  bool enable_expanding_hatched_road_markings;  // 是否扩展斜线区域
  bool enable_expanding_intersection_areas;     // 是否扩展交叉口区域
  bool enable_expanding_freespace_areas;        // 是否扩展自由空间
  double drivable_margin;                       // 可行驶边界
};
```

**生成过程**:
1. 获取起步路径对应的道路车道
2. 生成可行驶车道列表
3. 计算左右边界偏移
4. 考虑车辆尺寸和安全余量

### 3.2 候选路径输出：CandidateOutput

`planCandidate()` 方法返回候选路径：

```cpp
struct CandidateOutput
{
  PathWithLaneId path_candidate;              // 候选路径
  double lateral_shift;                       // 横向偏移量
  double start_distance_to_path_change;       // 到路径变化起点的距离
  double finish_distance_to_path_change;      // 到路径变化终点的距离
};
```

**用途**:
- 在审批模式下显示候选路径
- 用于可视化和预览
- 辅助操作员决策

### 3.3 内部状态：PullOutStatus

模块维护内部状态 `status_`，虽然不直接输出，但影响输出：

```cpp
struct PullOutStatus
{
  PullOutPath pull_out_path;                  // 起步路径
  size_t current_path_idx;                    // 当前路径索引
  PlannerType planner_type;                   // 规划器类型
  PathWithLaneId backward_path;               // 后退路径
  bool found_pull_out_path;                   // 是否找到起步路径
  bool is_safe_dynamic_objects;               // 对动态物体是否安全
  bool driving_forward;                       // 是否向前行驶
  bool backward_driving_complete;             // 后退行驶是否完成
  Pose pull_out_start_pose;                   // 起步起点位姿
  PoseWithDetailOpt stop_pose;                // 停止位姿
  bool has_departed;                          // 是否已经起步
};
```

---

## 4. 输入输出数据流

### 4.1 数据流图

```
输入数据源
  ├── PlannerData (共享数据)
  │   ├── self_odometry          → [判断车辆状态]
  │   ├── dynamic_object         → [安全检查]
  │   ├── route_handler          → [获取目标和车道]
  │   └── prev_output_path       → [路径连续性]
  │
  ├── StartPlannerParameters (模块参数)
  │   ├── 规划器启用开关
  │   ├── 速度和加速度限制
  │   └── 安全检查参数
  │
  └── PreviousModuleOutput (上游输出)
      └── reference_path         → [参考路径基准]

                ↓

        StartPlannerModule
        ├── updateData()          # 更新内部数据
        ├── planWithPriority()    # 规划起步路径
        ├── 安全检查               # 碰撞检测
        └── plan()                # 生成输出

                ↓

输出数据
  ├── BehaviorModuleOutput
  │   ├── path                   → [规划的起步路径]
  │   │   └── points[]           → [包含位姿和速度]
  │   ├── reference_path         → [继承的参考路径]
  │   ├── turn_signal_info       → [转向信号]
  │   └── drivable_area_info     → [可行驶区域]
  │
  └── 调试和可视化数据
      ├── debug_marker_          → [RViz标记]
      └── info_marker_           → [信息标记]
```

### 4.2 关键处理流程

#### 输入处理
```cpp
void StartPlannerModule::updateData()
{
  // 1. 更新路由信息
  if (receivedNewRoute()) {
    // 重置状态
    resetStatus();
  }

  // 2. 检查执行条件
  if (!isExecutionRequested()) return;

  // 3. 更新起步状态
  updatePullOutStatus();

  // 4. 规划起步路径
  planWithPriority(start_pose_candidates, refined_start_pose, 
                   goal_pose, search_priority);

  // 5. 安全检查
  if (requiresDynamicObjectsCollisionDetection()) {
    // 碰撞检测
  }
}
```

#### 输出生成
```cpp
BehaviorModuleOutput StartPlannerModule::plan()
{
  // 1. 选择路径
  if (!status_.found_pull_out_path) {
    return generateStopOutput();  // 停止路径
  }

  // 2. 确定当前阶段路径
  const auto path = [&]() {
    if (!status_.driving_forward) {
      return status_.backward_path;      // 后退阶段
    }
    if (!status_.is_safe_dynamic_objects) {
      return generateFeasibleStopPath(); // 动态停止
    }
    return getCurrentPath();              // 正常起步路径
  }();

  // 3. 构建输出
  BehaviorModuleOutput output;
  output.path = path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  
  // 4. 设置可行驶区域
  setDrivableAreaInfo(output);

  return output;
}
```

---

## 5. 关键代码位置

### 5.1 输入数据使用

| 输入数据 | 使用位置 | 用途 |
|---------|---------|------|
| `planner_data_->self_odometry` | `updateData()` | 获取当前车辆位姿 |
| `planner_data_->dynamic_object` | `hasCollisionWithDynamicObjects()` | 动态障碍物碰撞检测 |
| `planner_data_->route_handler` | `updateData()` | 获取目标点和车道信息 |
| `parameters_` | 各规划器 | 规划参数配置 |
| `getPreviousModuleOutput()` | `plan()` | 获取参考路径 |

### 5.2 输出数据生成

| 输出数据 | 生成位置 | 代码行 |
|---------|---------|--------|
| `output.path` | `plan()` | line 726 |
| `output.reference_path` | `plan()` | line 727 |
| `output.turn_signal_info` | `calcTurnSignalInfo()` | line 728 |
| `output.drivable_area_info` | `setDrivableAreaInfo()` | line 732 |

---

## 6. 速度处理说明

### 6.1 输入速度来源

1. **参考路径速度**:
   - 从 `route_handler` 获取的中心线路径
   - 速度值来自 lanelet 地图的 `speed_limit` 属性

2. **参数配置速度**:
   - `pull_out_velocity`: 起步目标速度
   - `prepare_time_before_start`: 起步前准备时间

### 6.2 输出速度设置

#### a. 正常起步路径
```cpp
// shift_pull_out.cpp:450
point.point.longitudinal_velocity_mps =
  std::min(point.point.longitudinal_velocity_mps, 
           static_cast<float>(terminal_velocity));
```
- 限制速度不超过终端速度
- 保留参考路径的速度限制

#### b. 停止路径
```cpp
// start_planner_module.cpp:831
for (auto & p : stop_path.points) {
  p.point.longitudinal_velocity_mps = 0.0;  // ⭐ 强制设为0
}
```
- **这是导致 start_planner 输出速度为0的关键代码**
- 在以下情况触发：
  - 未找到安全起步路径
  - 系统未就绪
  - 等待批准状态

#### c. 后退路径
- 速度可能为负值或0
- 根据后退距离和时间计算

---

## 7. 典型使用场景

### 场景1: 路边起步
**输入**:
- 车辆静止在路边
- 目标点在前方道路上
- 无障碍物阻挡

**处理**:
1. 使用 Shift Pull Out 规划器
2. 生成横向位移+前进的路径
3. 计算转向信号（左转或右转）

**输出**:
- 平滑的起步路径
- 速度从0逐渐增加到目标速度
- 正确的转向信号

### 场景2: 停车位出库（需要后退）
**输入**:
- 车辆在停车位内
- 目标点需要先后退
- 有停车位边界约束

**处理**:
1. 使用 Geometric Pull Out 规划器
2. 生成后退+前进的组合路径
3. 进行碰撞检查

**输出**:
- 分段路径：先后退，再前进
- 后退段速度为负或0
- 前进段速度正常

### 场景3: 拥挤环境起步
**输入**:
- 周围有动态障碍物
- 空间受限

**处理**:
1. 使用 Freespace Planner
2. 持续监控动态障碍物
3. 动态调整路径和速度

**输出**:
- 自由空间规划路径
- 可能包含停止点
- 速度根据障碍物距离动态调整

---

## 8. 调试建议

### 8.1 检查输入数据

```bash
# 检查车辆位姿
ros2 topic echo /localization/kinematic_state

# 检查动态障碍物
ros2 topic echo /perception/object_recognition/objects

# 检查路由
ros2 topic echo /planning/mission_planning/route
```

### 8.2 检查输出数据

```bash
# 检查输出路径
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id

# 查看速度值
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id | grep longitudinal_velocity_mps
```

### 8.3 查看模块状态

```bash
# 查看日志
ros2 run rqt_console rqt_console

# 过滤 start_planner 日志
# 关键字: [VEL_DEBUG], [start_planner], PullOutStatus
```

### 8.4 可视化

在 RViz 中查看：
- `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` - 输出路径
- `/planning/scenario_planning/lane_driving/behavior_planning/path_candidate` - 候选路径
- `debug markers` - 调试标记

---

## 9. 总结

### 关键要点

1. **输入是共享的规划数据**
   - 包含车辆状态、环境感知、路由信息
   - 通过 `PlannerData` 结构体传递
   - 使用上一个模块的参考路径

2. **输出是标准的行为模块输出**
   - 主要是规划的路径（`PathWithLaneId`）
   - 包含速度、位姿、车道ID等信息
   - 附带转向信号和可行驶区域

3. **速度为0的常见原因**
   - 未找到安全起步路径 (`!found_pull_out_path`)
   - 系统未就绪 (`!is_ready`)
   - 生成停止路径 (`generateStopPath()`)

4. **模块职责**
   - 从静止状态规划起步路径
   - 处理后退场景
   - 进行安全性检查
   - 生成转向信号

---

**相关文件**:
- 头文件: `autoware_behavior_path_start_planner_module/include/autoware/behavior_path_start_planner_module/start_planner_module.hpp`
- 实现: `autoware_behavior_path_start_planner_module/src/start_planner_module.cpp`
- 数据结构: `autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp`

**文档生成时间**: 2025-10-24

