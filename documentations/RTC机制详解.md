# RTC (Request To Cooperate) 机制详解

## 1. RTC 概述

### 1.1 什么是 RTC？

**RTC (Request To Cooperate)** 是 Autoware 中的**协作请求机制**，用于在行为规划模块与外部系统（如安全监控系统、人工操作界面）之间建立**审批流程**。

### 1.2 RTC 的设计目的

1. **安全审批**：允许外部系统（如安全监控、人工操作员）对关键路径修改进行审批
2. **状态透明**：向外部系统报告模块的执行状态和安全性
3. **灵活控制**：支持自动模式（auto mode）和手动审批模式
4. **多模块协调**：管理多个模块的并发请求，避免冲突

### 1.3 RTC 的核心概念

- **Cooperate Status（协作状态）**：模块向外部系统报告的状态信息
- **Cooperate Command（协作命令）**：外部系统对模块的审批指令（ACTIVATE/DEACTIVATE）
- **Auto Mode（自动模式）**：当启用时，模块根据安全性自动决定是否执行，无需外部审批
- **UUID**：每个模块实例的唯一标识符，用于跟踪特定的执行请求

## 2. RTC 代码位置

### 2.1 核心接口代码

**主要包**：`autoware_rtc_interface`

```
src/universe/autoware_universe/planning/autoware_rtc_interface/
├── include/autoware/rtc_interface/
│   └── rtc_interface.hpp          # RTC 接口类定义
├── src/
│   └── rtc_interface.cpp          # RTC 接口实现
├── README.md                       # RTC 使用文档
└── package.xml
```

**关键文件**：
- **头文件**：`autoware_rtc_interface/include/autoware/rtc_interface/rtc_interface.hpp`
- **实现文件**：`autoware_rtc_interface/src/rtc_interface.cpp`

### 2.2 在 Behavior Path Planner 中的使用

**场景模块接口**：
```
src/universe/autoware_universe/planning/behavior_path_planner/
└── autoware_behavior_path_planner_common/
    └── include/autoware/behavior_path_planner_common/interface/
        └── scene_module_interface.hpp  # 场景模块基类，包含 RTC 接口
```

**配置参数**：
```
src/universe/autoware_universe/planning/behavior_path_planner/
└── autoware_behavior_path_planner/
    └── config/
        └── scene_module_manager.param.yaml  # enable_rtc 配置
```

## 3. RTC 工作流程

### 3.1 基本工作流程

```
┌─────────────────────────────────────────────────────────────┐
│ 场景模块（Scene Module）                                      │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│ 1. 模块请求执行                                               │
│    - 生成 UUID（唯一标识）                                    │
│    - 计算安全性（safe）                                       │
│    - 计算距离（start_distance, finish_distance）              │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. updateCooperateStatus()                                   │
│    - 注册状态到 RTCInterface                                 │
│    - 状态：WAITING_FOR_EXECUTION                            │
│    - 发布到 Topic: /planning/cooperate_status/{module_name} │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. 外部系统（安全监控/人工操作）                              │
│    - 订阅 CooperateStatus                                    │
│    - 评估安全性                                               │
│    - 发送审批命令                                             │
│    - Service: /planning/cooperate_commands/{module_name}     │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. RTCInterface 接收命令                                     │
│    - 验证 UUID 有效性                                        │
│    - 更新 command_status (ACTIVATE/DEACTIVATE)              │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. 模块检查 isActivated()                                     │
│    - 如果返回 true → 执行路径规划                             │
│    - 如果返回 false → 等待或停止                              │
└─────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. 模块执行过程中持续更新状态                                 │
│    - 状态：RUNNING → SUCCEEDED/FAILED                        │
│    - 持续发布状态更新                                         │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 RTC 状态机

```
                    ┌─────────────────┐
                    │  未注册 (未启动) │
                    └────────┬────────┘
                             │
                             │ updateCooperateStatus()
                             │ (首次注册)
                             ▼
                    ┌─────────────────────────┐
                    │ WAITING_FOR_EXECUTION    │
                    │ (等待审批)                │
                    └────────┬─────────────────┘
                             │
                ┌────────────┴────────────┐
                │                         │
        收到 ACTIVATE              收到 DEACTIVATE
        或 Auto Mode              或 不安全
                │                         │
                ▼                         ▼
        ┌───────────────┐        ┌──────────────┐
        │   RUNNING     │        │   ABORTING   │
        │  (执行中)      │        │  (中止中)    │
        └───────┬───────┘        └──────┬───────┘
                │                        │
                │                        │
        执行完成/失败             中止完成
                │                        │
                ▼                        ▼
        ┌───────────────┐        ┌──────────────┐
        │ SUCCEEDED     │        │    FAILED    │
        │  (成功)       │        │   (失败)     │
        └───────────────┘        └──────────────┘
```

## 4. RTC 核心 API

### 4.1 RTCInterface 类

**位置**：`autoware_rtc_interface/include/autoware/rtc_interface/rtc_interface.hpp`

**主要方法**：

#### **updateCooperateStatus()** - 更新协作状态
```cpp
void updateCooperateStatus(
  const UUID & uuid,              // 模块实例的唯一标识
  const bool safe,                // 安全性评估结果
  const uint8_t state,            // 状态（WAITING_FOR_EXECUTION/RUNNING/SUCCEEDED/FAILED）
  const double start_distance,    // 到起始点的距离
  const double finish_distance,   // 到结束点的距离
  const rclcpp::Time & stamp,     // 时间戳
  const bool requested = false    // 是否已请求
);
```

**功能**：
- 注册或更新模块的协作状态
- 如果 UUID 不存在，创建新的状态记录
- 如果 UUID 已存在，更新状态（遵循状态转换规则）

#### **isActivated()** - 检查是否已激活
```cpp
bool isActivated(const UUID & uuid) const;
```

**返回值逻辑**：
- **Auto Mode 启用**：根据 `safe` 字段自动决定
  - `safe = true` → 返回 `true`（自动批准）
  - `safe = false` → 返回 `false`（自动拒绝）
- **Auto Mode 禁用**：需要外部审批
  - 收到 `ACTIVATE` 命令 → 返回 `true`
  - 收到 `DEACTIVATE` 命令 → 返回 `false`
  - 未收到命令 → 返回 `false`

#### **publishCooperateStatus()** - 发布状态
```cpp
void publishCooperateStatus(const rclcpp::Time & stamp);
```

**功能**：
- 将注册的所有状态发布到 ROS Topic
- Topic 名称：`/planning/cooperate_status/{module_name}`

#### **其他关键方法**：
- `isRegistered(uuid)` - 检查 UUID 是否已注册
- `isRTCEnabled(uuid)` - 检查是否启用了 RTC（非自动模式）
- `isTerminated(uuid)` - 检查模块是否已终止（SUCCEEDED/FAILED）
- `removeCooperateStatus(uuid)` - 移除状态记录
- `clearCooperateStatus()` - 清空所有状态

### 4.2 在 Scene Module 中的使用

**位置**：`scene_module_interface.hpp`

#### **updateRTCStatus()** - 更新 RTC 状态
```cpp
virtual void updateRTCStatus(const double start_distance, const double finish_distance)
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      const auto state = !ptr->isRegistered(uuid_map_.at(module_name)) || isWaitingApproval()
                           ? State::WAITING_FOR_EXECUTION
                           : State::RUNNING;
      ptr->updateCooperateStatus(
        uuid_map_.at(module_name), 
        isExecutionReady(),  // 安全性评估
        state, 
        start_distance, 
        finish_distance,
        clock_->now()
      );
    }
  }
}
```

**调用时机**：
- 模块请求执行时
- 模块执行过程中（周期性更新）
- 模块状态变化时

#### **isActivated()** - 检查激活状态
```cpp
bool isActivated() const
{
  if (rtc_interface_ptr_map_.empty()) {
    return true;  // 没有 RTC 接口，默认激活
  }
  
  if (!existRegisteredRequest()) {
    return false;  // 未注册请求
  }
  
  return existApprovedRequest();  // 检查是否有已批准的请求
}
```

**使用场景**：
- 模块在 `plan()` 或 `planWaitingApproval()` 中检查是否应该执行
- 如果未激活，模块返回等待状态或停止执行

## 5. RTC 消息类型

### 5.1 CooperateStatus（协作状态）

**消息类型**：`tier4_rtc_msgs::msg::CooperateStatus`

**字段**：
```cpp
builtin_interfaces::msg::Time stamp;        // 时间戳
unique_identifier_msgs::msg::UUID uuid;    // 模块实例 UUID
Module module;                              // 模块类型
bool safe;                                  // 安全性评估
bool requested;                             // 是否已请求
Command command_status;                      // 收到的命令（ACTIVATE/DEACTIVATE）
State state;                                 // 当前状态
double start_distance;                       // 到起始点的距离
double finish_distance;                      // 到结束点的距离
bool auto_mode;                              // 是否自动模式
```

### 5.2 CooperateCommand（协作命令）

**消息类型**：`tier4_rtc_msgs::msg::CooperateCommand`

**字段**：
```cpp
unique_identifier_msgs::msg::UUID uuid;    // 目标模块的 UUID
Module module;                              // 模块类型
Command command;                             // 命令类型（ACTIVATE/DEACTIVATE）
```

### 5.3 状态类型（State）

```cpp
enum State {
  WAITING_FOR_EXECUTION = 0,  // 等待执行（等待审批）
  RUNNING = 1,                // 运行中
  ABORTING = 2,                // 中止中
  SUCCEEDED = 3,               // 成功完成
  FAILED = 4                   // 失败
};
```

### 5.4 命令类型（Command）

```cpp
enum Command {
  ACTIVATE = 0,     // 激活（批准执行）
  DEACTIVATE = 1    // 停用（拒绝执行）
};
```

## 6. RTC 配置

### 6.1 enable_rtc 参数

**配置文件**：`scene_module_manager.param.yaml`

```yaml
static_obstacle_avoidance:
  enable_rtc: false  # 是否启用 RTC 审批
```

**含义**：
- `enable_rtc: true` - 模块需要外部审批才能执行
- `enable_rtc: false` - 模块自动执行，无需审批（自动模式）

### 6.2 当前配置状态

根据 `scene_module_manager.param.yaml`，**所有模块的 `enable_rtc` 都设置为 `false`**，这意味着：

- 所有模块默认运行在**自动模式**下
- 模块根据自身的安全性评估自动决定是否执行
- 不需要外部系统进行人工审批

## 7. RTC 在 Behavior Path Planner 中的实际应用

### 7.1 模块状态转换与 RTC

```cpp
// 在 scene_module_interface.hpp 中的状态更新逻辑
ModuleStatus updateState()
{
  // 检查是否可以转换到 WAITING_APPROVAL 状态
  if (canTransitWaitingApprovalState()) {
    return ModuleStatus::WAITING_APPROVAL;
  }
  
  // 检查是否可以转换到 RUNNING 状态
  if (canTransitWaitingApprovalToRunningState()) {
    return ModuleStatus::RUNNING;
  }
  
  // ... 其他状态转换
}
```

**RTC 与模块状态的关系**：
- `WAITING_APPROVAL` ↔ `WAITING_FOR_EXECUTION`（RTC 状态）
- `RUNNING` ↔ `RUNNING`（RTC 状态）
- `SUCCESS` ↔ `SUCCEEDED`（RTC 状态）
- `FAILURE` ↔ `FAILED`（RTC 状态）

### 7.2 实际使用示例

**示例：Start Planner Module**

```cpp
// 在 start_planner_module.cpp 中
BehaviorModuleOutput StartPlannerModule::plan()
{
  // 1. 计算路径和安全距离
  const double start_distance = ...;
  const double finish_distance = ...;
  
  // 2. 更新 RTC 状态
  updateRTCStatus(start_distance, finish_distance);
  
  // 3. 检查是否已激活
  if (!isActivated()) {
    // 未激活，返回等待审批的路径
    return planWaitingApproval();
  }
  
  // 4. 已激活，执行完整规划
  const auto path = planPullOutPath();
  
  // 5. 继续更新 RTC 状态
  updateRTCStatus(start_distance, finish_distance);
  
  return output;
}
```

## 8. RTC 的 ROS 接口

### 8.1 发布主题（Publisher）

**Topic**：`/planning/cooperate_status/{module_name}`
- **类型**：`tier4_rtc_msgs::msg::CooperateStatusArray`
- **频率**：10 Hz（通过定时器）
- **内容**：所有已注册模块的协作状态

**Topic**：`/planning/auto_mode_status/{module_name}`
- **类型**：`tier4_rtc_msgs::msg::AutoModeStatus`
- **内容**：自动模式启用状态

### 8.2 服务（Service）

**Service**：`/planning/cooperate_commands/{module_name}`
- **类型**：`tier4_rtc_msgs::srv::CooperateCommands`
- **功能**：接收外部系统的审批命令
- **请求**：`CooperateCommand[]` - 命令列表
- **响应**：`CooperateResponse[]` - 验证结果

**Service**：`/planning/enable_auto_mode/{module_name}`
- **类型**：`tier4_rtc_msgs::srv::AutoMode`
- **功能**：启用/禁用自动模式
- **请求**：`bool enable`
- **响应**：`bool success`

## 9. RTC 的工作模式

### 9.1 自动模式（Auto Mode）

**特点**：
- `enable_rtc: false` 或通过服务启用自动模式
- 模块根据 `safe` 字段自动决定是否执行
- 无需外部审批

**判断逻辑**：
```cpp
bool isActivated(const UUID & uuid) const
{
  if (itr->auto_mode && !itr->requested) {
    return itr->safe;  // 自动模式：根据安全性自动决定
  }
  return itr->command_status.type == Command::ACTIVATE;  // 手动模式：需要命令
}
```

### 9.2 手动审批模式

**特点**：
- `enable_rtc: true` 且自动模式禁用
- 必须等待外部系统发送 `ACTIVATE` 命令
- 外部系统可以发送 `DEACTIVATE` 命令强制停止

**工作流程**：
1. 模块注册状态（`WAITING_FOR_EXECUTION`）
2. 外部系统评估安全性
3. 外部系统发送 `ACTIVATE` 或 `DEACTIVATE` 命令
4. 模块检查 `isActivated()` 决定是否执行

## 10. RTC 与 Slot 机制的关系

### 10.1 Approved 模块与 RTC

**Approved 模块** = **已通过 RTC 审批的模块**

```cpp
// 在 scene_module_interface.hpp 中
bool existApprovedRequest() const
{
  return std::any_of(
    rtc_interface_ptr_map_.begin(), 
    rtc_interface_ptr_map_.end(), 
    [&](const auto & rtc) {
      if (!rtc.second->isRegistered(uuid_map_.at(rtc.first))) {
        return false;
      }
      if (rtc.second->isTerminated(uuid_map_.at(rtc.first))) {
        return false;
      }
      return rtc.second->isActivated(uuid_map_.at(rtc.first));  // 已激活 = 已批准
    }
  );
}
```

### 10.2 Candidate 模块与 RTC

**Candidate 模块** = **请求执行但尚未通过 RTC 审批的模块**

- 状态：`WAITING_FOR_EXECUTION`
- `isActivated()` 返回 `false`
- 等待外部审批或自动模式评估

### 10.3 状态转换流程

```
模块请求执行
    │
    ▼
注册 RTC 状态 (WAITING_FOR_EXECUTION)
    │
    ▼
检查 isActivated()
    │
    ├─ true ──> 转为 RUNNING ──> 加入 approved 栈
    │
    └─ false ──> 保持 WAITING_APPROVAL ──> 加入 candidate 栈
```

## 11. RTC 调试工具

### 11.1 RTC Replayer

**工具**：`autoware_rtc_replayer`

**功能**：
- 回放 RTC 状态历史
- 模拟外部审批命令
- 调试 RTC 交互流程

**文档**：https://autowarefoundation.github.io/autoware_tools/main/planning/autoware_rtc_replayer/

### 11.2 查看 RTC 状态

**命令行工具**：
```bash
# 查看协作状态
ros2 topic echo /planning/cooperate_status/{module_name}

# 查看自动模式状态
ros2 topic echo /planning/auto_mode_status/{module_name}

# 发送审批命令（通过服务）
ros2 service call /planning/cooperate_commands/{module_name} \
  tier4_rtc_msgs/srv/CooperateCommands "{commands: [{uuid: {...}, module: {...}, command: {type: 0}}]}"
```

## 12. 总结

### 12.1 RTC 的核心价值

1. **安全审批**：为关键路径修改提供外部审批机制
2. **状态透明**：实时报告模块执行状态和安全性
3. **灵活控制**：支持自动和手动两种模式
4. **系统集成**：便于与安全监控、人工操作等外部系统集成

### 12.2 当前使用情况

- **默认配置**：所有模块的 `enable_rtc: false`（自动模式）
- **实际应用**：主要用于状态报告和调试
- **未来扩展**：可以启用 RTC 实现更严格的安全审批流程

### 12.3 关键代码位置总结

| 组件 | 文件路径 |
|------|---------|
| **RTC 核心接口** | `autoware_rtc_interface/include/autoware/rtc_interface/rtc_interface.hpp` |
| **RTC 实现** | `autoware_rtc_interface/src/rtc_interface.cpp` |
| **场景模块集成** | `autoware_behavior_path_planner_common/include/.../scene_module_interface.hpp` |
| **配置参数** | `autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| **使用文档** | `autoware_rtc_interface/README.md` |

### 12.4 与 Slot 机制的关系

- **RTC 审批决定模块是 approved 还是 candidate**
- **Slot 机制管理 approved/candidate 模块的执行顺序**
- **两者配合实现安全可控的路径规划流程**

