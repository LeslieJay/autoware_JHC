# autoware_operation_mode_transition_manager 详解

## 0.调试总结

### 查看操作模式的状态
- 话题：/system/operation_mode/state

uint8 UNKNOWN = 0
uint8 STOP = 1
uint8 AUTONOMOUS = 2
uint8 LOCAL = 3
uint8 REMOTE = 4

### 当前所选择的控制指令模式
- 话题：/control/current_gate_mode

type: 
    tier4_control_msgs/msg/GateMode

interface: 
    uint8 AUTO = 0
    uint8 EXTERNAL = 1
    uint8 data

### engage启动

- 设定目标点，规划出路径后，有3种方法让小车进入 *自动驾驶* 状态：

1. 在rviz界面点击auto按钮

2. 通过服务 `ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode` 将小车转换到 auto，效果与rviz界面点击左上角的auto相同，小车开始自动行驶到目标点，停止后自动转换到stop模式。

3. 通过话题 `ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage "{engage: true}" -1`，通知自动驾驶系统会开始控制车辆的动作。

4. 通过服务 `ros2 service call /api/autoware/set/engage tier4_external_api_msgs/srv/Engage "{engage: true}"` 启动engage。

通过服务切换模式时，也会发布 `/autoware/engage` 话题，所以服务其实是间接调用话题。

- 跑rosbag条件下，发布话题不起作用，需要使用服务。

- 没有启动 engage 时，trajectory_follower 发布的速度也是 0。


[/adapi/node/operation_mode]:

- Service Servers:
    /api/operation_mode/change_to_autonomous: autoware_adapi_v1_msgs/srv/ChangeOperationMode
    /api/operation_mode/change_to_local: autoware_adapi_v1_msgs/srv/ChangeOperationMode
    /api/operation_mode/change_to_remote: autoware_adapi_v1_msgs/srv/ChangeOperationMode
    /api/operation_mode/change_to_stop: autoware_adapi_v1_msgs/srv/ChangeOperationMode
    /api/operation_mode/disable_autoware_control: autoware_adapi_v1_msgs/srv/ChangeOperationMode
    /api/operation_mode/enable_autoware_control: autoware_adapi_v1_msgs/srv/ChangeOperationMode
- Service Clients:
    /system/operation_mode/change_autoware_control: tier4_system_msgs/srv/ChangeAutowareControl
    /system/operation_mode/change_operation_mode: tier4_system_msgs/srv/ChangeOperationMode

- [api]也是要通过 `/system/operation_mode/change_operation_mode` 服务向[system]请求改变 operation_mode ，[api]是autoware暴露给外界的交互接口
实际的操作还是在[system]完成

## 1. 模块概述

### 1.1 目的与功能

`autoware_operation_mode_transition_manager` 是 Autoware 系统中负责管理操作模式（Operation Mode）转换的核心模块。它确保车辆在不同控制模式之间安全、平滑地切换。

### 1.2 支持的操作模式

系统支持以下四种操作模式：

- **`Autonomous`（自主模式）**：车辆完全由 Autoware 自动驾驶系统控制
- **`Local`（本地模式）**：车辆由物理连接的控制系统控制（如操纵杆）
- **`Remote`（远程模式）**：车辆由远程控制器控制
- **`Stop`（停止模式）**：车辆停止，没有活动的控制系统

### 1.3 转换状态

在模式转换过程中，系统会进入 **`In Transition`（转换中）** 状态：

- 转换到新操作者尚未完成
- 前一个操作者仍负责控制系统，直到转换完成
- 某些动作可能受限（如突然制动或转向），由 `vehicle_cmd_gate` 限制

---

### 快速参考表

| 操作 | 命令 |
|------|------|
| 查看当前状态 | `ros2 topic echo /api/operation_mode/state` |
| 切换到自主模式 | `ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| 切换到停止模式 | `ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| 切换到本地模式 | `ros2 service call /api/operation_mode/change_to_local autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| 切换到远程模式 | `ros2 service call /api/operation_mode/change_to_remote autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| 启用控制 | `ros2 service call /api/operation_mode/enable_autoware_control autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| 禁用控制 | `ros2 service call /api/operation_mode/disable_autoware_control autoware_adapi_v1_msgs/srv/ChangeOperationMode` |

### 与旧版接口的兼容性

Autoware 还提供了旧版的 engagement 接口用于向后兼容：

```bash
# 查看 engage 状态
ros2 topic echo /api/autoware/get/engage

# 设置 engage
ros2 topic pub /api/autoware/get/engage autoware_vehicle_msgs/msg/Engage "stamp: {sec: 0, nanosec: 0}, engage: true"
```

## 2. 系统架构

### 2.1 状态机结构

模块管理三个层次的状态转换：

#### 层次1：Autoware 控制启用/禁用
- **ENABLED**：车辆由 Autoware 控制
- **DISABLED**：车辆脱离 Autoware 控制，期望手动驾驶

#### 层次2：操作模式（AUTO/LOCAL/REMOTE/NONE）
- **AUTO**：车辆由 Autoware 控制，使用规划/控制组件计算的自主控制指令
- **LOCAL**：车辆由 Autoware 控制，使用本地连接的操作者（如操纵杆控制器）
- **REMOTE**：车辆由 Autoware 控制，使用远程连接的操作者
- **NONE**：车辆不受任何操作者控制

#### 层次3：转换状态（IN TRANSITION / COMPLETED）
- **IN TRANSITION**：模式转换过程中，前一个操作者负责确认转换完成
- **COMPLETED**：模式转换已完成

### 2.2 与其他节点的关系

```
┌─────────────────────────────────────────────────────────┐
│  autoware_operation_mode_transition_manager             │
│                                                          │
│  ┌──────────────────────────────────────────────────┐   │
│  │  输入：                                          │   │
│  │  - 模式转换请求（服务）                          │   │
│  │  - 车辆状态（话题）                              │   │
│  │  - 控制指令（话题）                              │   │
│  │  - 轨迹（话题）                                  │   │
│  └──────────────────────────────────────────────────┘   │
│                                                          │
│  ┌──────────────────────────────────────────────────┐   │
│  │  输出：                                          │   │
│  │  - 操作模式状态（话题）                          │   │
│  │  - 控制模式请求（服务客户端）                    │   │
│  │  - 调试信息（话题）                              │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
    vehicle_cmd_gate    control_mode    trajectory_follower
```

---

## 3. 输入/输出接口

### 3.1 输入接口

#### 服务（模式转换请求）
- `/system/operation_mode/change_autoware_control` [`tier4_system_msgs/srv/ChangeAutowareControl`]
  - 启用/禁用 Autoware 控制
  
- `/system/operation_mode/change_operation_mode` [`tier4_system_msgs/srv/ChangeOperationMode`]
  - 改变操作模式（AUTONOMOUS/LOCAL/REMOTE/STOP）

#### 话题（状态监控）
- `/control/command/control_cmd` [`autoware_control_msgs/msg/Control`]
  - 车辆控制信号（用于检查转换条件）
  
- `/localization/kinematic_state` [`nav_msgs/msg/Odometry`]
  - 自车状态（位置、速度、姿态）
  
- `/planning/scenario_planning/trajectory` [`autoware_planning_msgs/msg/Trajectory`]
  - 规划轨迹（用于检查车辆是否在轨迹上）
  
- `/vehicle/status/control_mode` [`autoware_vehicle_msgs/msg/ControlModeReport`]
  - 车辆控制模式（自主/手动）
  
- `/control/trajectory_follower/control_cmd` [`autoware_control_msgs/msg/Control`]
  - 轨迹跟踪器控制指令（用于检查速度一致性）

### 3.2 输出接口

#### 话题
- `/system/operation_mode/state` [`autoware_adapi_v1_msgs/msg/OperationModeState`]
  - 当前操作模式状态（模式、是否在转换中、各模式是否可用）
  
- `/control/autoware_operation_mode_transition_manager/debug_info` [`OperationModeTransitionManagerDebug`]
  - 详细的模式转换调试信息

#### 服务客户端
- `/control/control_mode_request` [`autoware_vehicle_msgs/srv/ControlModeCommand`]
  - 请求改变车辆控制模式（自主/手动）

---

## 4. 核心工作流程

### 4.1 主循环（onTimer）

模块以固定频率（默认 10 Hz）运行主循环：

```cpp
void OperationModeTransitionManager::onTimer()
{
  // 1. 获取最新数据
  control_mode_report_ = *sub_control_mode_report_.take_data();
  gate_operation_mode_ = *sub_gate_operation_mode_.take_data();
  
  // 2. 更新各模式状态
  for (const auto & [type, mode] : modes_) {
    mode->update(current_mode_ == type && transition_);
  }
  
  // 3. 检查各模式转换是否可用
  for (const auto & [type, mode] : modes_) {
    available_mode_change_[type] = mode->isModeChangeAvailable();
  }
  
  // 4. 处理兼容性接口同步
  // ...
  
  // 5. 处理模式转换
  if (transition_) {
    processTransition();
  }
  
  // 6. 发布数据
  publishData();
}
```

### 4.2 模式转换请求处理

#### 4.2.1 启用/禁用 Autoware 控制

```cpp
void onChangeAutowareControl(request, response)
{
  if (request->autoware_control) {
    // 启用 Autoware 控制：转换到当前操作模式
    changeOperationMode(std::nullopt);
  } else {
    // 禁用 Autoware 控制：取消转换，切换到手动模式
    compatibility_transition_ = std::nullopt;
    transition_.reset();
    changeControlMode(ControlModeCommand::Request::MANUAL);
  }
}
```

#### 4.2.2 改变操作模式

```cpp
void onChangeOperationMode(request, response)
{
  const auto mode = toEnum(*request);
  changeOperationMode(mode.value());
}
```

### 4.3 模式转换逻辑（changeOperationMode）

```cpp
void changeOperationMode(std::optional<OperationMode> request_mode)
{
  // 1. 检查是否与当前模式相同
  if (current_mode_ == request_mode) {
    throw NoEffectWarning("The mode is the same as the current.");
  }
  
  // 2. 检查是否已在转换中
  if (transition_ && request_mode != OperationMode::STOP) {
    throw ServiceException(ERROR_IN_TRANSITION, "The mode transition is in progress.");
  }
  
  // 3. 检查转换条件是否满足
  if (current_control || request_control) {
    if (!available_mode_change_[request_mode.value_or(current_mode_)]) {
      throw ServiceException(ERROR_NOT_AVAILABLE, "The mode change condition is not satisfied.");
    }
    
    // 4. 创建转换对象
    if (request_control) {
      transition_ = std::make_unique<Transition>(now(), request_control, std::nullopt);
    } else {
      transition_ = std::make_unique<Transition>(now(), request_control, current_mode_);
    }
  }
  
  // 5. 更新当前模式
  current_mode_ = request_mode.value_or(current_mode_);
}
```

### 4.4 转换处理（processTransition）

转换处理是模块的核心逻辑，确保安全完成模式切换：

```cpp
void processTransition()
{
  // 1. 检查超时
  if (transition_timeout_ < (now() - transition_->time).seconds()) {
    return cancelTransition();
  }
  
  // 2. 检查 engage 失败
  if (transition_->is_engage_failed) {
    return cancelTransition();
  }
  
  // 3. 检查是否已 engage（车辆是否已切换到自主模式）
  if (current_control) {
    transition_->is_engage_completed = true;
  } else {
    if (transition_->is_engage_completed) {
      return cancelTransition();  // 已 engage 但又被取消
    }
  }
  
  // 4. 检查兼容性接口是否同步
  if (current_mode_ != compatibility_.get_mode()) {
    return;  // 等待同步
  }
  
  // 5. 如果已 engage，检查转换是否完成
  if (current_control) {
    if (modes_.at(current_mode_)->isModeChangeCompleted()) {
      return transition_.reset();  // 转换完成
    }
  } else {
    // 6. 如果未 engage，请求 engage
    if (transition_->is_engage_requested && gate_operation_mode_.is_in_transition) {
      transition_->is_engage_requested = false;
      return changeControlMode(ControlModeCommand::Request::AUTONOMOUS);
    }
  }
}
```

---

## 5. 模式类详解

### 5.1 基类：ModeChangeBase

所有模式类都继承自 `ModeChangeBase`，提供统一的接口：

```cpp
class ModeChangeBase {
  virtual void update(bool transition) {}  // 更新状态
  virtual bool isModeChangeCompleted() = 0;  // 检查转换是否完成
  virtual bool isModeChangeAvailable() = 0;  // 检查转换是否可用
  virtual DebugInfo getDebugInfo() { return DebugInfo{}; }  // 获取调试信息
};
```

### 5.2 StopMode（停止模式）

最简单的模式，转换始终可用且立即完成：

```cpp
class StopMode : public ModeChangeBase {
  bool isModeChangeCompleted() override { return true; }
  bool isModeChangeAvailable() override { return true; }
};
```

### 5.3 AutonomousMode（自主模式）

最复杂的模式，包含详细的安全检查：

#### 5.3.1 转换可用性检查（isModeChangeAvailable）

检查车辆是否满足进入自主模式的条件：

```cpp
bool AutonomousMode::isModeChangeAvailable()
{
  // 1. 如果禁用检查，始终可用
  if (!check_engage_condition_) {
    return true;
  }
  
  // 2. 检查是否允许在行驶中 engage
  if (!enable_engage_on_driving_ && std::fabs(current_speed) > 0.01) {
    return false;  // 车辆在行驶且不允许在行驶中 engage
  }
  
  // 3. 检查轨迹是否可用
  if (trajectory_.points.size() < 2) {
    return false;
  }
  
  // 4. 找到最近的轨迹点
  const auto closest_idx = findNearestIndex(...);
  if (!closest_idx) {
    return false;
  }
  
  // 5. 检查横向偏差
  const auto lateral_deviation = calc_distance2d(closest_point.pose, kinematics_.pose.pose);
  const bool lateral_deviation_ok = lateral_deviation < param.dist_threshold;
  
  // 6. 检查航向偏差
  const auto yaw_deviation = calc_yaw_deviation(closest_point.pose, kinematics_.pose.pose);
  const bool yaw_deviation_ok = yaw_deviation < param.yaw_threshold;
  
  // 7. 检查速度偏差
  const auto speed_deviation = current_speed - target_planning_speed;
  const bool speed_upper_deviation_ok = speed_deviation <= param.speed_upper_threshold;
  const bool speed_lower_deviation_ok = speed_deviation >= param.speed_lower_threshold;
  
  // 8. 检查停止命令
  const bool is_stop_cmd_indicated = std::abs(target_control_speed) < 0.01;
  const bool stop_ok = !(std::abs(current_speed) > 0.1 && is_stop_cmd_indicated);
  
  // 9. 检查加速度
  const bool large_acceleration_ok = !hasDangerAcceleration();
  
  // 10. 检查横向加速度
  const auto [has_large_lat_acc, has_large_lat_acc_diff] = hasDangerLateralAcceleration();
  const bool large_lateral_acceleration_ok = !has_large_lat_acc;
  const bool large_lateral_acceleration_diff_ok = !has_large_lat_acc_diff;
  
  // 11. 如果车辆已停止，允许转换
  if (param.allow_autonomous_in_stopped && std::abs(current_speed) < 0.01) {
    return true;
  }
  
  // 12. 所有条件都满足
  return lateral_deviation_ok && yaw_deviation_ok && 
         speed_upper_deviation_ok && speed_lower_deviation_ok && 
         stop_ok && large_acceleration_ok && 
         large_lateral_acceleration_ok && large_lateral_acceleration_diff_ok;
}
```

#### 5.3.2 转换完成检查（isModeChangeCompleted）

检查车辆是否稳定在轨迹上，满足完成转换的条件：

```cpp
bool AutonomousMode::isModeChangeCompleted()
{
  // 1. 如果禁用检查，立即完成
  if (!check_engage_condition_) {
    return true;
  }
  
  // 2. 如果车辆已停止，立即完成
  if (param.allow_autonomous_in_stopped && std::abs(current_speed) < 0.01) {
    return true;
  }
  
  // 3. 检查轨迹
  if (trajectory_.points.size() < 2) {
    return false;
  }
  
  // 4. 找到最近的轨迹点
  const auto closest_idx = findNearestIndex(...);
  if (!closest_idx) {
    return false;
  }
  
  // 5. 检查横向偏差（使用更严格的阈值）
  const auto dist_deviation = calcLateralOffset(trajectory_.points, kinematics_.pose.pose.position);
  if (dist_deviation > stable_check_param_.dist_threshold) {
    stable_start_time_.reset();  // 重置稳定时间
    return false;
  }
  
  // 6. 检查航向偏差
  const auto yaw_deviation = calcYawDeviation(trajectory_.points, kinematics_.pose.pose);
  if (yaw_deviation > stable_check_param_.yaw_threshold) {
    stable_start_time_.reset();
    return false;
  }
  
  // 7. 检查速度偏差
  const auto speed_deviation = kinematics_.twist.twist.linear.x - closest_point.longitudinal_velocity_mps;
  if (speed_deviation > stable_check_param_.speed_upper_threshold ||
      speed_deviation < stable_check_param_.speed_lower_threshold) {
    stable_start_time_.reset();
    return false;
  }
  
  // 8. 开始计时稳定时间
  if (!stable_start_time_) {
    stable_start_time_ = std::make_unique<rclcpp::Time>(clock_->now());
  }
  
  // 9. 检查是否稳定足够长时间
  const double stable_time = (clock_->now() - *stable_start_time_).seconds();
  return stable_time > stable_check_param_.duration;
}
```

#### 5.3.3 危险加速度检查

```cpp
bool AutonomousMode::hasDangerAcceleration()
{
  // 如果车辆已停止，任何加速度都可以
  if (std::abs(kinematics_.twist.twist.linear.x) < 0.01) {
    return false;
  }
  
  // 检查加速度是否超过阈值
  return std::abs(control_cmd_.longitudinal.acceleration) > engage_acceptable_param_.acc_threshold;
}

std::pair<bool, bool> AutonomousMode::hasDangerLateralAcceleration()
{
  // 计算当前横向加速度
  const auto curr_lat_acc = curr_vx * curr_wz;
  
  // 计算目标横向加速度（基于转向角）
  const auto target_wz = curr_vx * std::tan(control_cmd_.lateral.steering_tire_angle) / wheelbase;
  const auto target_lat_acc = curr_vx * target_wz;
  
  // 检查横向加速度是否过大
  const bool has_large_lat_acc = 
    std::abs(curr_lat_acc) > engage_acceptable_param_.lateral_acc_threshold;
  
  // 检查横向加速度偏差是否过大
  const bool has_large_lat_acc_diff = 
    std::abs(curr_lat_acc - target_lat_acc) > engage_acceptable_param_.lateral_acc_diff_threshold;
  
  return {has_large_lat_acc, has_large_lat_acc_diff};
}
```

### 5.4 LocalMode 和 RemoteMode

这两个模式目前实现简单，转换始终可用且立即完成：

```cpp
class LocalMode : public ModeChangeBase {
  bool isModeChangeCompleted() override { return true; }
  bool isModeChangeAvailable() override { return true; }
};

class RemoteMode : public ModeChangeBase {
  bool isModeChangeCompleted() override { return true; }
  bool isModeChangeAvailable() override { return true; }
};
```

---

## 6. 参数配置

### 6.1 基本参数

| 参数名 | 类型 | 描述 | 默认值 |
|--------|------|------|--------|
| `transition_timeout` | `double` | 转换超时时间（秒），超过此时间视为转换失败 | 10.0 |
| `frequency_hz` | `double` | 运行频率（Hz） | 10.0 |
| `enable_engage_on_driving` | `bool` | 是否允许在车辆行驶时 engage | false |
| `check_engage_condition` | `bool` | 是否检查 engage 条件（false 时始终允许） | false |
| `nearest_dist_deviation_threshold` | `double` | 查找最近轨迹点的距离阈值（米） | 3.0 |
| `nearest_yaw_deviation_threshold` | `double` | 查找最近轨迹点的航向阈值（弧度） | 1.57 |

### 6.2 engage_acceptable_limits 参数

用于检查是否可以进入自主模式：

| 参数名 | 类型 | 描述 | 默认值 |
|--------|------|------|--------|
| `allow_autonomous_in_stopped` | `bool` | 如果为 true，车辆停止时跳过所有检查 | true |
| `dist_threshold` | `double` | 轨迹与车辆的距离阈值（米） | 1.5 |
| `yaw_threshold` | `double` | 轨迹与车辆的航向角阈值（弧度） | 0.524 |
| `speed_upper_threshold` | `double` | 速度偏差上限（m/s） | 10.0 |
| `speed_lower_threshold` | `double` | 速度偏差下限（m/s） | -10.0 |
| `acc_threshold` | `double` | 加速度阈值（m/s²） | 1.5 |
| `lateral_acc_threshold` | `double` | 横向加速度阈值（m/s²） | 1.0 |
| `lateral_acc_diff_threshold` | `double` | 横向加速度偏差阈值（m/s²） | 0.5 |

### 6.3 stable_check 参数

用于检查转换是否完成：

| 参数名 | 类型 | 描述 | 默认值 |
|--------|------|------|--------|
| `duration` | `double` | 稳定条件必须满足的持续时间（秒） | 0.1 |
| `dist_threshold` | `double` | 距离偏差阈值（米） | 1.5 |
| `speed_upper_threshold` | `double` | 速度偏差上限（m/s） | 2.0 |
| `speed_lower_threshold` | `double` | 速度偏差下限（m/s） | -2.0 |
| `yaw_threshold` | `double` | 航向偏差阈值（弧度） | 0.262 |

### 6.4 参数组合行为矩阵

不同参数组合下的 engage 行为：

| `enable_engage_on_driving` | `check_engage_condition` | `allow_autonomous_in_stopped` | 允许 engage 的场景 |
|:--------------------------:|:-------------------------:|:----------------------------:|:-----------------:|
| ❌ | ❌ | ❌ | 仅当车辆静止时 |
| ❌ | ❌ | ✅ | 仅当车辆静止时 |
| ❌ | ✅ | ❌ | 车辆静止且满足所有 engage 条件 |
| ❌ | ✅ | ✅ | 仅当车辆静止时 |
| ✅ | ❌ | ❌ | 任何时候（⚠️ 不推荐） |
| ✅ | ❌ | ✅ | 任何时候（⚠️ 不推荐） |
| ✅ | ✅ | ❌ | 满足所有 engage 条件时（无论车辆状态） |
| ✅ | ✅ | ✅ | 满足所有 engage 条件时，或车辆静止时 |

---

## 7. 转换状态机

### 7.1 转换流程示例：切换到自主模式

```
1. 接收转换请求
   └─> changeOperationMode(AUTONOMOUS)
       └─> 检查转换条件
           └─> 创建 Transition 对象
               └─> 设置 current_mode_ = AUTONOMOUS

2. 进入转换状态（is_in_transition = true）
   └─> processTransition()
       └─> 检查 gate_operation_mode_.is_in_transition
           └─> 如果为 true，请求 engage
               └─> changeControlMode(AUTONOMOUS)

3. 等待车辆切换到自主控制模式
   └─> 监控 control_mode_report_.mode
       └─> 当 mode == AUTONOMOUS 时
           └─> transition_->is_engage_completed = true

4. 检查转换完成条件
   └─> AutonomousMode::isModeChangeCompleted()
       └─> 检查车辆是否稳定在轨迹上
           └─> 如果稳定时间 > stable_check.duration
               └─> 转换完成

5. 退出转换状态
   └─> transition_.reset()
       └─> is_in_transition = false
```

### 7.2 转换失败处理

转换可能因以下原因失败：

1. **超时**：转换时间超过 `transition_timeout`
   ```cpp
   if (transition_timeout_ < (now() - transition_->time).seconds()) {
     cancelTransition();
   }
   ```

2. **Engage 失败**：车辆拒绝切换到自主模式
   ```cpp
   if (transition_->is_engage_failed) {
     cancelTransition();
   }
   ```

3. **条件不满足**：转换条件检查失败
   ```cpp
   if (!available_mode_change_[request_mode]) {
     throw ServiceException(ERROR_NOT_AVAILABLE, ...);
   }
   ```

### 7.3 转换取消（cancelTransition）

```cpp
void cancelTransition()
{
  const auto & previous = transition_->previous;
  if (previous) {
    // 恢复到之前的模式
    current_mode_ = previous.value();
  } else {
    // 切换到手动模式
    changeControlMode(ControlModeCommand::Request::MANUAL);
  }
  transition_.reset();
}
```

---

## 8. 兼容性接口

模块提供了向后兼容的接口，用于与旧版本系统集成：

### 8.1 兼容性话题

- `/autoware/engage` [`autoware_vehicle_msgs/msg/Engage`]
- `/control/current_gate_mode` [`tier4_control_msgs/msg/GateMode`]
- `/control/external_cmd_selector/current_selector_mode` [`tier4_control_msgs/msg/ExternalCommandSelectorMode`]

### 8.2 兼容性同步机制

模块会同步兼容性接口的状态，确保新旧接口一致：

```cpp
// 如果正在转换，设置兼容性接口的模式
if (compatibility_transition_) {
  compatibility_.set_mode(current_mode_);
} else {
  // 否则，从兼容性接口读取模式
  current_mode_ = compatibility_.get_mode().value_or(current_mode_);
}
```

---

## 9. 调试信息

模块发布详细的调试信息，帮助诊断转换问题：

### 9.1 DebugInfo 消息字段

- **状态信息**：
  - `status`：当前状态字符串
  - `in_autoware_control`：是否在 Autoware 控制下
  - `in_transition`：是否在转换中

- **条件检查标志**：
  - `is_all_ok`：所有条件是否满足
  - `engage_allowed_for_stopped_vehicle`：是否因车辆停止而允许
  - `trajectory_available_ok`：轨迹是否可用
  - `lateral_deviation_ok`：横向偏差是否满足
  - `yaw_deviation_ok`：航向偏差是否满足
  - `speed_upper_deviation_ok`：速度上限偏差是否满足
  - `speed_lower_deviation_ok`：速度下限偏差是否满足
  - `stop_ok`：停止命令检查是否通过
  - `large_acceleration_ok`：加速度检查是否通过
  - `large_lateral_acceleration_ok`：横向加速度检查是否通过
  - `large_lateral_acceleration_diff_ok`：横向加速度偏差检查是否通过

- **数值信息**：
  - `current_speed`：当前速度
  - `target_control_speed`：目标控制速度
  - `target_planning_speed`：目标规划速度
  - `target_control_acceleration`：目标控制加速度
  - `lateral_acceleration`：横向加速度
  - `lateral_acceleration_deviation`：横向加速度偏差
  - `lateral_deviation`：横向偏差
  - `yaw_deviation`：航向偏差
  - `speed_deviation`：速度偏差

---

## 10. 典型使用场景

### 10.1 场景1：从停止状态启动自主驾驶

1. 车辆处于 `STOP` 模式，静止状态
2. 调用服务切换到 `AUTONOMOUS` 模式
3. 检查转换条件：
   - 如果 `allow_autonomous_in_stopped = true`，立即允许
   - 否则检查轨迹、位置等条件
4. 创建转换对象，进入转换状态
5. 请求车辆切换到自主控制模式
6. 等待车辆确认切换到自主模式
7. 检查车辆是否稳定在轨迹上
8. 转换完成，退出转换状态

### 10.2 场景2：在行驶中切换到自主模式

1. 车辆在手动模式下行驶
2. 调用服务切换到 `AUTONOMOUS` 模式
3. 检查 `enable_engage_on_driving` 参数
   - 如果为 `false`，拒绝转换
   - 如果为 `true`，继续检查其他条件
4. 检查车辆是否在轨迹上，速度、加速度是否满足条件
5. 如果条件满足，创建转换对象
6. 请求车辆切换到自主控制模式
7. 等待车辆稳定在轨迹上
8. 转换完成

### 10.3 场景3：从自主模式切换到停止模式

1. 车辆在 `AUTONOMOUS` 模式下运行
2. 调用服务切换到 `STOP` 模式
3. 立即允许转换（STOP 模式转换始终可用）
4. 创建转换对象
5. 请求车辆切换到手动控制模式
6. 转换完成（STOP 模式转换立即完成）

---

## 11. 注意事项与最佳实践

### 11.1 参数调优建议

1. **`transition_timeout`**：
   - 应大于 `stable_check.duration` + 安全余量（建议 0.5 秒）
   - 太短可能导致正常转换被误判为超时
   - 太长可能导致失败转换无法及时取消

2. **`enable_engage_on_driving`**：
   - 默认应设置为 `false`，确保安全
   - 如果启用，必须仔细调整 `engage_acceptable_limits` 参数
   - 必须确保 `vehicle_cmd_gate` 的转换过滤器配置正确

3. **`check_engage_condition`**：
   - 生产环境应设置为 `true`
   - 测试环境可以设置为 `false` 以简化测试

4. **`stable_check.duration`**：
   - 太短可能导致车辆未稳定就完成转换
   - 太长可能导致转换时间过长
   - 建议根据车辆动态特性调整

### 11.2 常见问题

1. **转换总是超时**：
   - 检查 `stable_check` 参数是否过于严格
   - 检查车辆是否真的稳定在轨迹上
   - 检查 `transition_timeout` 是否足够

2. **无法切换到自主模式**：
   - 检查 `isModeChangeAvailable()` 返回的调试信息
   - 检查车辆是否在轨迹上
   - 检查速度、加速度是否满足条件

3. **转换完成但车辆不稳定**：
   - 增加 `stable_check.duration`
   - 收紧 `stable_check` 的阈值参数

---

## 12. 未来改进方向

根据 README 中的说明，未来可能进行以下改进：

1. **移除向后兼容接口**：简化代码，移除旧接口
2. **合并到 vehicle_cmd_gate**：由于与 `vehicle_cmd_gate` 紧密相关，可能合并到该模块

---

## 13. 总结

`autoware_operation_mode_transition_manager` 是 Autoware 系统中确保安全模式转换的关键模块。它通过：

1. **多层次状态管理**：管理 Autoware 控制启用/禁用、操作模式、转换状态
2. **严格的安全检查**：在转换前检查车辆状态、轨迹、速度、加速度等条件
3. **稳定的转换过程**：确保车辆稳定在轨迹上后才完成转换
4. **完善的错误处理**：处理超时、失败等异常情况
5. **详细的调试信息**：提供丰富的调试数据帮助诊断问题

确保了车辆在不同控制模式之间安全、平滑地切换。

