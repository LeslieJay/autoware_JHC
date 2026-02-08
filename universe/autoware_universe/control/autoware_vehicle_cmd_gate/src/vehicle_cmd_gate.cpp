// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file vehicle_cmd_gate.cpp
 * @brief 车辆命令门控节点实现
 * 
 * ============================================================================
 * 模块概述
 * ============================================================================
 * 
 * 本文件实现了车辆命令门控节点（VehicleCmdGate），这是Autoware控制系统的关键安全组件。
 * 该节点作为控制命令的最后一道防线，负责：
 * 1. 接收来自多个源的控制命令（自动驾驶、外部控制、紧急控制）
 * 2. 根据当前模式（GateMode）选择合适的命令源
 * 3. 处理紧急状态和外部急停请求
 * 4. 对控制命令进行安全过滤和限制
 * 5. 发布最终的控制命令给车辆执行器
 * 
 * ============================================================================
 * 整体架构
 * ============================================================================
 * 
 * 输入源：
 * ├── 自动驾驶命令（AUTO模式）
 * │   ├── control_cmd（控制命令：速度、加速度、转向）
 * │   ├── turn_indicators_cmd（转向灯命令）
 * │   ├── hazard_lights_cmd（危险警示灯命令）
 * │   └── gear_cmd（档位命令）
 * ├── 外部控制命令（EXTERNAL模式）
 * │   └── （同上结构）
 * ├── 紧急控制命令（EMERGENCY状态）
 * │   ├── control_cmd
 * │   ├── hazard_lights_cmd
 * │   └── gear_cmd
 * └── 状态信息
 *     ├── gate_mode（门控模式：AUTO/EXTERNAL）
 *     ├── engage（是否已接管）
 *     ├── operation_mode（操作模式：AUTONOMOUS/MANUAL/TRANSITION）
 *     ├── mrm_state（最小风险机动状态）
 *     └── 车辆状态（速度、加速度、转向角等）
 * 
 * 处理流程：
 * 1. 数据接收 → 2. 安全检查 → 3. 命令选择 → 4. 命令过滤 → 5. 命令发布
 * 
 * 输出：
 * ├── control_cmd（最终控制命令）
 * ├── turn_indicators_cmd（转向灯命令）
 * ├── hazard_lights_cmd（危险警示灯命令）
 * ├── gear_cmd（档位命令）
 * └── 状态信息（gate_mode、engage、emergency等）
 * 
 * ============================================================================
 * 核心处理逻辑（onTimer函数）
 * ============================================================================
 * 
 * 定时器以固定频率（update_rate参数）执行，主要流程：
 * 
 * 1. 读取命令数据
 *    - 使用PollingSubscriber非阻塞读取最新命令（转向灯、危险警示灯、档位）
 *    - 控制命令通过回调函数实时接收
 * 
 * 2. 数据准备检查
 *    - 检查紧急状态心跳是否已接收（如果启用紧急处理）
 *    - 检查外部紧急停止心跳是否已接收（如果启用检查）
 *    - 如果数据未准备就绪，等待而不发布命令
 * 
 * 3. 心跳超时检查
 *    - 系统紧急心跳超时 → 发布紧急停止命令并返回
 *    - 外部急停心跳超时 → 设置外部急停标志
 * 
 * 4. 外部急停检查
 *    - 如果外部急停标志被设置 → 发布紧急停止命令并返回
 * 
 * 5. 命令选择
 *    - 根据gate_mode选择auto_commands_或remote_commands_
 *    - 如果处于系统紧急状态，使用emergency_commands_
 *    - 注意：当前实现中始终使用auto_commands_（可能是调试用途）
 * 
 * 6. 命令发布
 *    - 发布转向灯、危险警示灯、档位命令（带连续性检查）
 *    - 控制命令在回调函数中实时发布（onAutoCtrlCmd/onRemoteCtrlCmd）
 * 
 * ============================================================================
 * 命令发布逻辑（publishControlCommands函数）
 * ============================================================================
 * 
 * 当收到控制命令时，执行以下检查和修改（按顺序）：
 * 
 * 1. 安全检查
 *    - 系统紧急心跳超时 → 直接返回，不发布
 *    - 外部急停激活 → 直接返回，不发布
 *    - 数据未准备就绪 → 直接返回，不发布
 * 
 * 2. 命令处理
 *    - 温和停止请求 → 设置速度为0，使用温和停止加速度
 *    - 系统紧急状态 → 使用紧急命令
 *    - 未engage → 创建纵向停止命令
 * 
 * 3. 暂停检查
 *    - 如果已暂停且已engage → 只停止纵向运动
 *    - 如果已暂停且未engage → 完全停止（包括横向）
 * 
 * 4. 命令过滤
 *    - 如果启用过滤器 → 应用速度、加速度、转向角等限制
 *    - 根据操作模式选择过滤器（正常模式/过渡模式）
 * 
 * 5. 命令发布
 *    - 发布车辆紧急状态
 *    - 发布最终控制命令
 *    - 发布暂停接口状态
 *    - 发布温和停止接口状态
 * 
 * ============================================================================
 * 命令过滤器
 * ============================================================================
 * 
 * 过滤器确保所有控制命令在安全范围内，限制包括：
 * - 速度限制（vel_lim）
 * - 纵向加速度限制（lon_acc_lim，基于速度插值）
 * - 纵向急动度限制（lon_jerk_lim，基于速度插值）
 * - 横向加速度限制（lat_acc_lim，基于速度插值）
 * - 横向急动度限制（lat_jerk_lim，基于速度插值）
 * - 转向角限制（steer_lim，基于速度插值）
 * - 转向角速度限制（steer_rate_lim，基于速度插值）
 * - 实际转向角差值限制（actual_steer_diff_lim，基于速度插值）
 * 
 * 过滤器参数：
 * - nominal：正常模式（AUTONOMOUS）下的限制参数
 * - on_transition：过渡模式（从MANUAL切换到AUTO）下的限制参数（通常更严格）
 * 
 * ============================================================================
 * 紧急处理机制
 * ============================================================================
 * 
 * 1. 系统紧急状态
 *    - 通过MRM状态检测（mrm_state topic）
 *    - 如果MRM状态为EMERGENCY_STOP且正在运行 → 设置is_system_emergency_=true
 *    - 需要紧急处理器发送心跳，如果心跳超时 → 发布紧急停止命令
 * 
 * 2. 外部急停
 *    - 通过服务接口设置（~/service/external_emergency_stop）
 *    - 需要外部模块发送心跳（如果启用检查）
 *    - 如果心跳超时 → 自动设置外部急停标志
 *    - 通过服务接口清除（~/service/clear_external_emergency_stop）
 * 
 * 3. 紧急停止命令
 *    - 速度：0
 *    - 加速度：emergency_acceleration_（较大的负值）
 *    - 转向角：保持上一帧的转向角
 *    - 危险警示灯：启用
 *    - 转向灯：无命令
 * 
 * ============================================================================
 * 关键状态变量
 * ============================================================================
 * 
 * - is_engaged_：是否已接管车辆控制
 * - is_system_emergency_：系统是否处于紧急状态
 * - is_external_emergency_stop_：外部急停是否激活
 * - current_gate_mode_：当前门控模式（AUTO/EXTERNAL）
 * - current_operation_mode_：当前操作模式（AUTONOMOUS/MANUAL/TRANSITION）
 * - auto_commands_/remote_commands_/emergency_commands_：各命令源的最新命令
 * 
 * ============================================================================
 * 注意事项
 * ============================================================================
 * 
 * 1. 命令连续性：转向灯、危险警示灯、档位命令需要保证时间戳连续性，
 *    避免时间倒退导致的问题
 * 
 * 2. 心跳机制：关键模块（紧急处理器、外部急停模块）需要定期发送心跳，
 *    如果心跳超时，节点会采取安全措施
 * 
 * 3. 过滤器激活：如果过滤器频繁激活，说明控制模块可能需要调优
 * 
 * 4. 过渡模式：从手动切换到自动时，使用更严格的过滤器参数，确保平滑切换
 * 
 * 5. 当前实现：代码中某些部分被注释掉，始终使用auto_commands_，
 *    可能是调试或特殊需求，需要根据实际情况调整
 */

#include "vehicle_cmd_gate.hpp"

#include "autoware_utils/ros/update_param.hpp"
#include "marker_helper.hpp"

#include <rclcpp/logging.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::vehicle_cmd_gate
{

namespace
{
/**
 * @brief 获取门控模式名称（用于日志输出）
 * @param gate_mode 门控模式枚举值
 * @return 门控模式的字符串表示
 */
const char * getGateModeName(const GateMode::_data_type & gate_mode)
{
  if (gate_mode == GateMode::AUTO) {
    return "AUTO";  // 自动驾驶模式
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";  // 外部遥控模式
  }
  return "NOT_SUPPORTED";  // 不支持的模式
}

}  // namespace

/**
 * @brief 构造函数：初始化车辆命令门控节点
 * @param node_options ROS2节点选项
 * 
 * 构造函数完成以下初始化工作：
 * 1. 初始化发布者和订阅者
 * 2. 加载和配置参数
 * 3. 创建定时器和服务
 * 4. 初始化诊断更新器
 * 5. 设置命令过滤器参数
 */
VehicleCmdGate::VehicleCmdGate(const rclcpp::NodeOptions & node_options)
: Node("vehicle_cmd_gate", node_options), is_engaged_(false), updater_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // 初始化上一帧命令指针（用于保证命令连续性）
  prev_turn_indicator_ = nullptr;
  prev_hazard_light_ = nullptr;
  prev_gear_ = nullptr;

  // 配置QoS：使用transient_local确保新订阅者能收到最后一条消息
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  // ==================== 创建车辆停止检查器 ====================
  // 用于检查车辆是否已停止（基于停止持续时间）
  vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);

  // ==================== 创建发布者 ====================
  // 发布车辆紧急状态
  vehicle_cmd_emergency_pub_ =
    create_publisher<VehicleEmergencyStamped>("output/vehicle_cmd_emergency", durable_qos);
  // 发布控制命令（速度、加速度、转向角等）
  control_cmd_pub_ = create_publisher<Control>("output/control_cmd", durable_qos);
  // 发布档位命令
  gear_cmd_pub_ = create_publisher<GearCommand>("output/gear_cmd", durable_qos);
  // 发布转向灯命令
  turn_indicator_cmd_pub_ =
    create_publisher<TurnIndicatorsCommand>("output/turn_indicators_cmd", durable_qos);
  // 发布危险警示灯命令
  hazard_light_cmd_pub_ =
    create_publisher<HazardLightsCommand>("output/hazard_lights_cmd", durable_qos);
  // 发布当前门控模式
  gate_mode_pub_ = create_publisher<GateMode>("output/gate_mode", durable_qos);
  // 发布engage状态（是否已接管车辆控制）
  engage_pub_ = create_publisher<EngageMsg>("output/engage", durable_qos);
  // 发布外部紧急状态
  pub_external_emergency_ = create_publisher<Emergency>("output/external_emergency", durable_qos);
  // 发布操作模式状态
  operation_mode_pub_ = create_publisher<OperationModeState>("output/operation_mode", durable_qos);
  // 发布处理耗时（用于性能分析）
  processing_time_pub_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  // 发布过滤器激活状态
  is_filter_activated_pub_ =
    create_publisher<IsFilterActivated>("~/is_filter_activated", durable_qos);
  // 发布过滤器激活标记（可视化用）
  filter_activated_marker_pub_ =
    create_publisher<MarkerArray>("~/is_filter_activated/marker", durable_qos);
  // 发布原始过滤器激活标记
  filter_activated_marker_raw_pub_ =
    create_publisher<MarkerArray>("~/is_filter_activated/marker_raw", durable_qos);
  // 发布过滤器激活标志
  filter_activated_flag_pub_ =
    create_publisher<BoolStamped>("~/is_filter_activated/flag", durable_qos);

  // ==================== 创建订阅者 ====================
  // 订阅外部紧急停止心跳（用于监控外部紧急停止模块的健康状态）
  external_emergency_stop_heartbeat_sub_ = create_subscription<Heartbeat>(
    "input/external_emergency_stop_heartbeat", 1,
    std::bind(&VehicleCmdGate::onExternalEmergencyStopHeartbeat, this, _1));
  // 订阅门控模式（AUTO或EXTERNAL）
  gate_mode_sub_ = create_subscription<GateMode>(
    "input/gate_mode", 1, std::bind(&VehicleCmdGate::onGateMode, this, _1));
  // 订阅engage状态（是否已接管控制）
  engage_sub_ = create_subscription<EngageMsg>(
    "input/engage", 1, std::bind(&VehicleCmdGate::onEngage, this, _1));
  // 订阅车辆运动学状态（位置、速度、姿态等，用于过滤器）
  kinematics_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    [this](Odometry::SharedPtr msg) { current_kinematics_ = *msg; });
  // 订阅加速度信息（用于过滤器）
  acc_sub_ = create_subscription<AccelWithCovarianceStamped>(
    "input/acceleration", 1, [this](AccelWithCovarianceStamped::SharedPtr msg) {
      current_acceleration_ = msg->accel.accel.linear.x;
    });
  // 订阅转向角报告（用于过滤器）
  steer_sub_ = create_subscription<SteeringReport>(
    "input/steering", 1,
    [this](SteeringReport::SharedPtr msg) { current_steer_ = msg->steering_tire_angle; });
  // 订阅操作模式状态（AUTONOMOUS、MANUAL、TRANSITION等）
  operation_mode_sub_ = create_subscription<OperationModeState>(
    "input/operation_mode", rclcpp::QoS(1).transient_local(),
    [this](const OperationModeState::SharedPtr msg) { current_operation_mode_ = *msg; });
  // 订阅MRM状态（最小风险机动状态，用于检测系统紧急状态）
  mrm_state_sub_ = create_subscription<MrmState>(
    "input/mrm_state", 1, std::bind(&VehicleCmdGate::onMrmState, this, _1));

  // ==================== 订阅自动驾驶命令 ====================
  // 订阅自动驾驶控制命令（来自轨迹跟踪器）
  auto_control_cmd_sub_ = create_subscription<Control>(
    "input/auto/control_cmd", 1, std::bind(&VehicleCmdGate::onAutoCtrlCmd, this, _1));
  // 注意：转向灯、危险警示灯、档位命令使用PollingSubscriber在onTimer中读取

  // ==================== 订阅外部控制命令 ====================
  // 订阅外部遥控控制命令
  remote_control_cmd_sub_ = create_subscription<Control>(
    "input/external/control_cmd", 1, std::bind(&VehicleCmdGate::onRemoteCtrlCmd, this, _1));
  // 注意：转向灯、危险警示灯、档位命令使用PollingSubscriber在onTimer中读取

  // ==================== 订阅紧急控制命令 ====================
  // 订阅紧急处理器的控制命令
  emergency_control_cmd_sub_ = create_subscription<Control>(
    "input/emergency/control_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyCtrlCmd, this, _1));
  // 注意：危险警示灯、档位命令使用PollingSubscriber在onTimer中读取

  // ==================== 加载参数 ====================
  // 是否启用紧急处理（如果启用，将处理系统紧急状态）
  // 设置为false可以关闭MRM（最小风险机动）功能
  use_emergency_handling_ = declare_parameter<bool>("use_emergency_handling");
  // 是否检查外部紧急停止心跳（如果启用，需要外部紧急停止模块发送心跳）
  check_external_emergency_heartbeat_ =
    declare_parameter<bool>("check_external_emergency_heartbeat");
  
  // 输出参数信息用于调试
  RCLCPP_INFO(this->get_logger(), "use_emergency_handling_: %s", use_emergency_handling_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "check_external_emergency_heartbeat_: %s", check_external_emergency_heartbeat_ ? "true" : "false");
  
  // 系统紧急状态心跳超时时间（秒）
  system_emergency_heartbeat_timeout_ =
    declare_parameter<double>("system_emergency_heartbeat_timeout");
  // 外部紧急停止心跳超时时间（秒）
  external_emergency_stop_heartbeat_timeout_ =
    declare_parameter<double>("external_emergency_stop_heartbeat_timeout");
  // 停止保持时的加速度（m/s²，通常为负值，用于保持车辆停止）
  stop_hold_acceleration_ = declare_parameter<double>("stop_hold_acceleration");
  // 紧急停止时的加速度（m/s²，通常为较大的负值）
  emergency_acceleration_ = declare_parameter<double>("emergency_acceleration");
  // 温和停止服务请求的加速度（m/s²）
  moderate_stop_service_acceleration_ =
    declare_parameter<double>("moderate_stop_service_acceleration");
  // 停止检查持续时间（秒，用于判断车辆是否已停止）
  stop_check_duration_ = declare_parameter<double>("stop_check_duration");
  // 是否启用命令限制过滤器
  enable_cmd_limit_filter_ = declare_parameter<bool>("enable_cmd_limit_filter");
  // 过滤器激活计数阈值（连续激活多少次才触发警告）
  filter_activated_count_threshold_ = declare_parameter<int>("filter_activated_count_threshold");
  // 过滤器激活速度阈值（m/s，超过此速度时过滤器激活才被视为严重）
  filter_activated_velocity_threshold_ =
    declare_parameter<double>("filter_activated_velocity_threshold");

  // ==================== 配置车辆参数和过滤器参数 ====================
  // 获取车辆信息（轴距等）
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  {
    // 配置正常模式（AUTONOMOUS）下的过滤器参数
    VehicleCmdFilterParam p;
    p.wheel_base = vehicle_info.wheel_base_m;  // 轴距（米）
    p.vel_lim = declare_parameter<double>("nominal.vel_lim");  // 速度限制（m/s）
    p.reference_speed_points =
      declare_parameter<std::vector<double>>("nominal.reference_speed_points");  // 参考速度点（用于插值）
    p.steer_lim = declare_parameter<std::vector<double>>("nominal.steer_lim");  // 转向角限制（rad）
    p.steer_rate_lim = declare_parameter<std::vector<double>>("nominal.steer_rate_lim");  // 转向角速度限制（rad/s）
    p.lon_acc_lim = declare_parameter<std::vector<double>>("nominal.lon_acc_lim");  // 纵向加速度限制（m/s²）
    p.lon_jerk_lim = declare_parameter<std::vector<double>>("nominal.lon_jerk_lim");  // 纵向急动度限制（m/s³）
    p.lat_acc_lim = declare_parameter<std::vector<double>>("nominal.lat_acc_lim");  // 横向加速度限制（m/s²）
    p.lat_jerk_lim = declare_parameter<std::vector<double>>("nominal.lat_jerk_lim");  // 横向急动度限制（m/s³）
    p.actual_steer_diff_lim =
      declare_parameter<std::vector<double>>("nominal.actual_steer_diff_lim");  // 实际转向角差值限制（rad）
    filter_.setParam(p);  // 设置过滤器参数
  }

  {
    // 配置过渡模式（TRANSITION，从MANUAL切换到AUTO）下的过滤器参数
    // 过渡模式通常使用更严格的限制，以确保平滑切换
    VehicleCmdFilterParam p;
    p.wheel_base = vehicle_info.wheel_base_m;
    p.vel_lim = declare_parameter<double>("on_transition.vel_lim");
    p.reference_speed_points =
      declare_parameter<std::vector<double>>("on_transition.reference_speed_points");
    p.steer_lim = declare_parameter<std::vector<double>>("on_transition.steer_lim");
    p.steer_rate_lim = declare_parameter<std::vector<double>>("on_transition.steer_rate_lim");
    p.lon_acc_lim = declare_parameter<std::vector<double>>("on_transition.lon_acc_lim");
    p.lon_jerk_lim = declare_parameter<std::vector<double>>("on_transition.lon_jerk_lim");
    p.lat_acc_lim = declare_parameter<std::vector<double>>("on_transition.lat_acc_lim");
    p.lat_jerk_lim = declare_parameter<std::vector<double>>("on_transition.lat_jerk_lim");
    p.actual_steer_diff_lim =
      declare_parameter<std::vector<double>>("on_transition.actual_steer_diff_lim");
    filter_on_transition_.setParam(p);  // 设置过渡模式过滤器参数
  }

  // ==================== 设置默认值 ====================
  current_gate_mode_.data = GateMode::AUTO;  // 默认使用自动驾驶模式
  current_operation_mode_.mode = OperationModeState::STOP;  // 默认操作模式为停止

  // ==================== 创建服务 ====================
  // Engage服务（用于接管/释放车辆控制）
  srv_engage_ = create_service<EngageSrv>(
    "~/service/engage", std::bind(&VehicleCmdGate::onEngageService, this, _1, _2));
  // 外部紧急停止服务（统一接口）
  srv_external_emergency_ = create_service<SetEmergency>(
    "~/service/external_emergency",
    std::bind(&VehicleCmdGate::onExternalEmergencyStopService, this, _1, _2, _3));
  // 设置外部紧急停止服务（旧接口，已废弃）
  srv_external_emergency_stop_ = create_service<Trigger>(
    "~/service/external_emergency_stop",
    std::bind(&VehicleCmdGate::onSetExternalEmergencyStopService, this, _1, _2, _3));
  // 清除外部紧急停止服务（旧接口，已废弃）
  srv_clear_external_emergency_stop_ = create_service<Trigger>(
    "~/service/clear_external_emergency_stop",
    std::bind(&VehicleCmdGate::onClearExternalEmergencyStopService, this, _1, _2, _3));

  // ==================== 配置诊断更新器 ====================
  updater_.setHardwareID("vehicle_cmd_gate");
  // 添加心跳诊断（用于检查节点是否存活）
  updater_.add("heartbeat", [](auto & stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  });
  // 添加外部紧急停止诊断
  updater_.add("emergency_stop_operation", this, &VehicleCmdGate::checkExternalEmergencyStop);

  // ==================== 创建接口对象 ====================
  // ADAPI暂停接口（用于处理暂停请求）
  adapi_pause_ = std::make_unique<AdapiPauseInterface>(this);
  // 温和停止接口（用于处理温和停止请求）
  moderate_stop_interface_ = std::make_unique<ModerateStopInterface>(this);

  // ==================== 创建定时器 ====================
  // 计算更新周期（从update_rate参数计算）
  const auto update_period = 1.0 / declare_parameter<double>("update_rate");
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_period));
  // 主定时器：执行主要处理逻辑（命令选择、过滤、发布等）
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&VehicleCmdGate::onTimer, this));
  // 状态发布定时器：定期发布状态信息
  timer_pub_status_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&VehicleCmdGate::publishStatus, this));

  // ==================== 创建工具对象 ====================
  // 日志级别配置器（用于动态调整日志级别）
  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  // 发布时间发布器（用于发布消息的时间戳）
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // ==================== 注册参数回调 ====================
  // 当参数被动态修改时，会调用onParameter回调函数
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&VehicleCmdGate::onParameter, this, _1));
}

/**
 * @brief 参数更新回调函数
 * @param parameters 要更新的参数列表
 * @return 参数更新结果
 * 
 * 当节点参数被动态修改时，此函数会被调用。
 * 更新所有可动态修改的参数，包括：
 * - 紧急处理相关参数
 * - 心跳超时参数
 * - 加速度参数
 * - 过滤器相关参数
 * - 车辆过滤器限制参数
 */
rcl_interfaces::msg::SetParametersResult VehicleCmdGate::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  // ==================== 更新基本参数 ====================
  update_param<bool>(parameters, "use_emergency_handling", use_emergency_handling_);
  update_param<bool>(
    parameters, "check_external_emergency_heartbeat", check_external_emergency_heartbeat_);
  update_param<double>(
    parameters, "system_emergency_heartbeat_timeout", system_emergency_heartbeat_timeout_);
  update_param<double>(
    parameters, "external_emergency_stop_heartbeat_timeout",
    external_emergency_stop_heartbeat_timeout_);
  update_param<double>(parameters, "stop_hold_acceleration", stop_hold_acceleration_);
  update_param<double>(parameters, "emergency_acceleration", emergency_acceleration_);
  update_param<double>(
    parameters, "moderate_stop_service_acceleration", moderate_stop_service_acceleration_);
  update_param<double>(parameters, "stop_check_duration", stop_check_duration_);
  update_param<bool>(parameters, "enable_cmd_limit_filter", enable_cmd_limit_filter_);
  update_param<int>(
    parameters, "filter_activated_count_threshold", filter_activated_count_threshold_);
  update_param<double>(
    parameters, "filter_activated_velocity_threshold", filter_activated_velocity_threshold_);

  // Vehicle Parameter
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  {
    VehicleCmdFilterParam p = filter_.getParam();
    p.wheel_base = vehicle_info.wheel_base_m;
    update_param<double>(parameters, "nominal.vel_lim", p.vel_lim);
    update_param<std::vector<double>>(
      parameters, "nominal.reference_speed_points", p.reference_speed_points);
    update_param<std::vector<double>>(parameters, "nominal.steer_lim", p.steer_lim);
    update_param<std::vector<double>>(parameters, "nominal.steer_rate_lim", p.steer_rate_lim);
    update_param<std::vector<double>>(parameters, "nominal.lon_acc_lim", p.lon_acc_lim);
    update_param<std::vector<double>>(parameters, "nominal.lon_jerk_lim", p.lon_jerk_lim);
    update_param<std::vector<double>>(parameters, "nominal.lat_acc_lim", p.lat_acc_lim);
    update_param<std::vector<double>>(parameters, "nominal.lat_jerk_lim", p.lat_jerk_lim);
    update_param<std::vector<double>>(
      parameters, "nominal.actual_steer_diff_lim", p.actual_steer_diff_lim);
    filter_.setParam(p);
  }

  {
    VehicleCmdFilterParam p = filter_on_transition_.getParam();
    p.wheel_base = vehicle_info.wheel_base_m;
    update_param<double>(parameters, "on_transition.vel_lim", p.vel_lim);
    update_param<std::vector<double>>(
      parameters, "on_transition.reference_speed_points", p.reference_speed_points);
    update_param<std::vector<double>>(parameters, "on_transition.steer_lim", p.steer_lim);
    update_param<std::vector<double>>(parameters, "on_transition.steer_rate_lim", p.steer_rate_lim);
    update_param<std::vector<double>>(parameters, "on_transition.lon_acc_lim", p.lon_acc_lim);
    update_param<std::vector<double>>(parameters, "on_transition.lon_jerk_lim", p.lon_jerk_lim);
    update_param<std::vector<double>>(parameters, "on_transition.lat_acc_lim", p.lat_acc_lim);
    update_param<std::vector<double>>(parameters, "on_transition.lat_jerk_lim", p.lat_jerk_lim);
    update_param<std::vector<double>>(
      parameters, "on_transition.actual_steer_diff_lim", p.actual_steer_diff_lim);
    filter_on_transition_.setParam(p);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

/**
 * @brief 检查心跳是否超时
 * @param heartbeat_received_time 心跳接收时间（共享指针）
 * @param timeout 超时时间（秒）
 * @return true表示心跳超时，false表示心跳正常
 * 
 * 此函数用于检查关键模块（如紧急处理器、外部紧急停止模块）的心跳是否超时。
 * 如果心跳超时，说明该模块可能已失效，需要采取相应的安全措施。
 */
bool VehicleCmdGate::isHeartbeatTimeout(
  const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout)
{
  // 如果超时时间为0，表示不检查心跳超时
  if (timeout == 0.0) {
    return false;
  }

  // 如果从未收到心跳，视为超时
  if (!heartbeat_received_time) {
    return true;
  }

  // 计算从上次心跳到现在的时间
  const auto time_from_heartbeat = this->now() - *heartbeat_received_time;

  // 如果超过超时时间，返回true
  return time_from_heartbeat.seconds() > timeout;
}

/**
 * @brief 检查数据是否准备就绪
 * @return true表示数据准备就绪，false表示数据未准备就绪
 * 
 * 在开始处理控制命令之前，需要确保所有必要的输入数据都已接收。
 * 特别是紧急状态心跳和外部紧急停止心跳（如果启用）。
 * 如果数据未准备就绪，节点将等待，不会发布控制命令。
 */
bool VehicleCmdGate::isDataReady()
{
  // 如果启用紧急处理，需要先收到紧急状态心跳
  if (use_emergency_handling_) {
    if (!emergency_state_heartbeat_received_time_) {
      RCLCPP_WARN(get_logger(), "emergency_state_heartbeat_received_time_ is false");
      return false;
    }
  }

  // 如果启用外部紧急停止心跳检查，需要先收到心跳
  if (check_external_emergency_heartbeat_) {
    if (!external_emergency_stop_heartbeat_received_time_) {
      RCLCPP_WARN(get_logger(), "external_emergency_stop_heartbeat_received_time_ is false");
      return false;
    }
  }

  return true;
}

/**
 * @brief 自动驾驶控制命令回调函数
 * @param msg 控制命令消息
 * 
 * 当收到来自自动驾驶模块（轨迹跟踪器）的控制命令时调用。
 * 如果当前门控模式为AUTO，则立即发布该命令。
 * 注意：此函数只处理控制命令（速度、加速度、转向），
 * 转向灯、危险警示灯、档位命令在onTimer中通过PollingSubscriber读取。
 */
void VehicleCmdGate::onAutoCtrlCmd(Control::ConstSharedPtr msg)
{
  // 保存自动驾驶控制命令
  auto_commands_.control = *msg;

  // 如果当前为自动驾驶模式，立即发布控制命令
  if (current_gate_mode_.data == GateMode::AUTO) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "leslie auto ");
    publishControlCommands(auto_commands_);
  }
}

/**
 * @brief 外部遥控控制命令回调函数
 * @param msg 控制命令消息
 * 
 * 当收到来自外部遥控模块的控制命令时调用。
 * 如果当前门控模式为EXTERNAL，则立即发布该命令。
 */
void VehicleCmdGate::onRemoteCtrlCmd(Control::ConstSharedPtr msg)
{
  // 保存外部遥控控制命令
  remote_commands_.control = *msg;

  // 如果当前为外部遥控模式，立即发布控制命令
  if (current_gate_mode_.data == GateMode::EXTERNAL) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "leslie remote");
    publishControlCommands(remote_commands_);
  }
}

/**
 * @brief 紧急控制命令回调函数
 * @param msg 控制命令消息
 * 
 * 当收到来自紧急处理器的控制命令时调用。
 * 如果启用紧急处理且系统处于紧急状态，则使用紧急命令。
 * 
 * 注意：当前实现中，即使处于紧急状态，也发布auto_commands_而不是emergency_commands_。
 * 这可能是为了调试或特殊需求。
 */
void VehicleCmdGate::onEmergencyCtrlCmd(Control::ConstSharedPtr msg)
{
  // 保存紧急控制命令
  emergency_commands_.control = *msg;

  // 如果启用紧急处理且系统处于紧急状态
  if (use_emergency_handling_ && is_system_emergency_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "leslie emergency");

    // 注意：当前实现始终发布auto_commands_，而不是emergency_commands_
    // 如果需要使用紧急命令，应取消注释下面这行并注释掉auto_commands_那行
    // publishControlCommands(emergency_commands_);
    publishControlCommands(auto_commands_); // 始终发布auto_commands_
    
  }
}

// check the continuity of topics
template <typename T>
T VehicleCmdGate::getContinuousTopic(
  const std::shared_ptr<T> & prev_topic, const T & current_topic, const std::string & topic_name)
{
  if ((rclcpp::Time(current_topic.stamp) - rclcpp::Time(prev_topic->stamp)).seconds() >= 0.0) {
    return current_topic;
  } else {
    if (topic_name != "") {
      RCLCPP_INFO(
        get_logger(),
        "The operation mode is changed, but the %s is not received yet:", topic_name.c_str());
    }
    return *prev_topic;
  }
}

/**
 * @brief 定时器回调函数 - 主要的处理逻辑
 * 
 * 这是节点的核心处理函数，按固定频率（由update_rate参数控制）执行。
 * 主要完成以下工作：
 * 1. 读取各种命令（转向灯、危险警示灯、档位）
 * 2. 更新诊断状态
 * 3. 检查数据准备状态
 * 4. 检查心跳超时
 * 5. 检查紧急停止状态
 * 6. 根据模式选择命令
 * 7. 发布控制命令和辅助命令
 * 8. 发布处理耗时
 * 
 * 执行流程：
 * - 首先读取所有PollingSubscriber的数据（非阻塞）
 * - 检查数据准备状态和心跳超时
 * - 如果处于紧急状态，发布紧急停止命令并返回
 * - 否则，根据gate_mode选择命令并发布
 */
void VehicleCmdGate::onTimer()
{
  // 用于计时本周期处理，用于debug/性能分析
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  // =========================
  // 1. 读取自动驾驶命令订阅者（turn_indicator、hazard_light、gear）
  // =========================
  // 使用PollingSubscriber非阻塞地读取最新数据
  // 如果收到新数据，更新对应的命令
  const auto msg_auto_command_turn_indicator = auto_turn_indicator_cmd_sub_.take_data();
  if (msg_auto_command_turn_indicator)
    auto_commands_.turn_indicator = *msg_auto_command_turn_indicator;

  const auto msg_auto_command_hazard_light = auto_hazard_light_cmd_sub_.take_data();
  if (msg_auto_command_hazard_light) auto_commands_.hazard_light = *msg_auto_command_hazard_light;

  const auto msg_auto_command_gear = auto_gear_cmd_sub_.take_data();
  if (msg_auto_command_gear) auto_commands_.gear = *msg_auto_command_gear;

  // =========================
  // 2. 读取远程控制命令订阅者（turn_indicator、hazard_light、gear）
  // =========================
  const auto msg_remote_command_turn_indicator = remote_turn_indicator_cmd_sub_.take_data();
  if (msg_remote_command_turn_indicator)
    remote_commands_.turn_indicator = *msg_remote_command_turn_indicator;

  const auto msg_remote_command_hazard_light = remote_hazard_light_cmd_sub_.take_data();
  if (msg_remote_command_hazard_light)
    remote_commands_.hazard_light = *msg_remote_command_hazard_light;

  const auto msg_remote_command_gear = remote_gear_cmd_sub_.take_data();
  if (msg_remote_command_gear) remote_commands_.gear = *msg_remote_command_gear;

  // =========================
  // 3. 读取紧急控制命令订阅者（仅支持hazard_light、gear）
  // =========================
  // 注意：紧急命令不包含转向灯命令
  const auto msg_emergency_command_hazard_light = emergency_hazard_light_cmd_sub_.take_data();
  if (msg_emergency_command_hazard_light)
    emergency_commands_.hazard_light = *msg_emergency_command_hazard_light;

  const auto msg_emergency_command_gear = emergency_gear_cmd_sub_.take_data();
  if (msg_emergency_command_gear) emergency_commands_.gear = *msg_emergency_command_gear;

  // =========================
  // 4. 状态更新
  // =========================
  // 强制更新诊断状态（用于心跳检测等）
  updater_.force_update();

  // =========================
  // 5. 是否数据准备完毕，否则等待
  // =========================
  // 检查所有必要的输入数据是否已接收（特别是心跳）
  // 如果数据未准备就绪，等待而不发布命令
  if (!isDataReady()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting topics...");
    return;
  }

  // =========================
  // 6. 检查系统紧急信号心跳超时
  // =========================
  // 如果启用紧急处理，检查紧急处理器的心跳是否超时
  // 如果超时，说明紧急处理器可能失效，立即发布紧急停止命令
  if (use_emergency_handling_) {
    is_emergency_state_heartbeat_timeout_ = isHeartbeatTimeout(
      emergency_state_heartbeat_received_time_, system_emergency_heartbeat_timeout_);

    if (is_emergency_state_heartbeat_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000 /*ms*/, "system_emergency heartbeat is timeout.");
      publishEmergencyStopControlCommands();
      return;  // 发布紧急停止命令后直接返回，不继续处理
    }
  }

  // =========================
  // 7. 检查外部急停心跳超时
  // =========================
  // 如果启用外部紧急停止心跳检查，检查心跳是否超时
  // 如果超时，设置外部急停标志
  if (check_external_emergency_heartbeat_) {
    is_external_emergency_stop_heartbeat_timeout_ = isHeartbeatTimeout(
      external_emergency_stop_heartbeat_received_time_, external_emergency_stop_heartbeat_timeout_);

    if (is_external_emergency_stop_heartbeat_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000 /*ms*/, "external_emergency_stop heartbeat is timeout.");
      is_external_emergency_stop_ = true;  // 设置外部急停标志
    }
  }

  // =========================
  // 8. 检查外部急停状态
  // =========================
  // 如果外部急停标志被设置，发布紧急停止命令并返回
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      // 如果心跳正常但外部急停仍被请求，提示用户清除状态
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000 /*ms*/,
        "Please call `clear_external_emergency_stop` service to clear state.");
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "is_external_emergency_stop_   and  is_external_emergency_stop_heartbeat_timeout_.");
    publishEmergencyStopControlCommands();
    return;  // 发布紧急停止命令后直接返回
  }

  // =========================
  // 9. 根据当前模式选择指令
  // =========================
  // 根据gate_mode和紧急状态选择要发布的命令
  // 注意：当前实现中，无论模式如何，都使用auto_commands_
  // 下面的代码被注释掉了，原本的逻辑是：
  // - 如果处于系统紧急状态，使用emergency_commands_
  // - 否则根据gate_mode选择auto_commands_或remote_commands_
  TurnIndicatorsCommand turn_indicator;
  HazardLightsCommand hazard_light;
  GearCommand gear;
  // if (use_emergency_handling_ && is_system_emergency_) {
  //   // 如果处于系统紧急状态，用紧急命令
  //   turn_indicator = emergency_commands_.turn_indicator;
  //   hazard_light = emergency_commands_.hazard_light;
  //   gear = emergency_commands_.gear;
  // } else {
  //   if (current_gate_mode_.data == GateMode::AUTO) {
  //     // 自动驾驶模式
  //     turn_indicator = auto_commands_.turn_indicator;
  //     hazard_light = auto_commands_.hazard_light;
  //     gear = auto_commands_.gear;

  //     // Autoware 未接管/未engaged时不下发灯光指令
  //     if (!is_engaged_) {
  //       turn_indicator.command = TurnIndicatorsCommand::NO_COMMAND;
  //       hazard_light.command = HazardLightsCommand::NO_COMMAND;
  //       turn_indicator.stamp = this->now();
  //       hazard_light.stamp = this->now();
  //     }
  //   } else if (current_gate_mode_.data == GateMode::EXTERNAL) {
  //     // 外部遥控模式
  //     turn_indicator = remote_commands_.turn_indicator;
  //     hazard_light = remote_commands_.hazard_light;
  //     gear = remote_commands_.gear;
  //   } else {
  //     // 不支持的控制模式
  //     throw std::runtime_error("invalid mode");
  //   }
  // }

  // 当前实现：始终使用auto_commands_（可能是调试或特殊需求）
  turn_indicator = auto_commands_.turn_indicator;
  hazard_light = auto_commands_.hazard_light;
  gear = auto_commands_.gear;

  // 打印三个话题的内容
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000 /*ms*/,"TurnIndicator: cmd=%d, HazardLight: cmd=%d, Gear: cmd=%d", 
    turn_indicator.command, hazard_light.command, gear.command);

  // =========================
  // 10. 发布指令（转向灯、危险警示灯、档位指令），包含连续性检查
  // =========================
  // 这些命令需要保证时间戳的连续性，避免时间倒退导致的问题
  // 如果新命令的时间戳早于上一帧，则使用上一帧的命令

  // --- 转向灯指令（TurnIndicatorsCommand） ---
  if (prev_turn_indicator_ != nullptr) {
    // 保证连续性：如果新命令的时间戳早于上一帧，使用上一帧的命令
    *prev_turn_indicator_ =
      getContinuousTopic(prev_turn_indicator_, turn_indicator, "TurnIndicatorsCommand");
    turn_indicator_cmd_pub_->publish(*prev_turn_indicator_);
  } else {
    // 初始化prev_turn_indicator_（第一次发布时）
    if (msg_auto_command_turn_indicator || msg_remote_command_turn_indicator) {
      prev_turn_indicator_ = std::make_shared<TurnIndicatorsCommand>(turn_indicator);
    }
    turn_indicator_cmd_pub_->publish(turn_indicator);
  }

  // --- 危险警示灯指令（HazardLightsCommand） ---
  if (prev_hazard_light_ != nullptr) {
    // 保证连续性
    *prev_hazard_light_ =
      getContinuousTopic(prev_hazard_light_, hazard_light, "HazardLightsCommand");
    hazard_light_cmd_pub_->publish(*prev_hazard_light_);
  } else {
    // 初始化prev_hazard_light_
    if (
      msg_auto_command_hazard_light || msg_remote_command_hazard_light ||
      msg_emergency_command_hazard_light) {
      prev_hazard_light_ = std::make_shared<HazardLightsCommand>(hazard_light);
    }
    hazard_light_cmd_pub_->publish(hazard_light);
  }

  // --- 档位指令（GearCommand） ---
  if (prev_gear_ != nullptr) {
    // 保证连续性
    *prev_gear_ = getContinuousTopic(prev_gear_, gear, "GearCommand");
    gear_cmd_pub_->publish(*prev_gear_);
  } else {
    // 初始化prev_gear_
    if (msg_auto_command_gear || msg_remote_command_gear || msg_emergency_command_gear) {
      prev_gear_ = std::make_shared<GearCommand>(gear);
    }
    gear_cmd_pub_->publish(gear);
  }

  // =========================
  // 11. 发布本周期处理耗时，方便debug性能瓶颈
  // =========================
  // 用于性能分析和调试，帮助识别性能瓶颈
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();  // 获取从stop_watch创建到现在的耗时（毫秒）
  processing_time_pub_->publish(processing_time_msg);
}

/**
 * @brief 发布控制命令
 * @param commands 要发布的命令集合（包含控制命令、转向灯、危险警示灯、档位等）
 * 
 * 这是发布控制命令的核心函数，执行以下步骤：
 * 1. 检查紧急状态（系统紧急、外部急停）
 * 2. 检查数据准备状态
 * 3. 应用温和停止请求（如果请求）
 * 4. 应用紧急命令（如果处于紧急状态）
 * 5. 应用停止命令（如果未engage）
 * 6. 应用暂停命令（如果暂停）
 * 7. 应用命令过滤器（如果启用）
 * 8. 发布最终的控制命令
 * 
 * 注意：此函数只发布控制命令（Control），不发布转向灯、危险警示灯、档位命令。
 * 这些命令在onTimer中单独发布。
 */
void VehicleCmdGate::publishControlCommands(const Commands & commands)
{
  // ==================== 安全检查 ====================
  // 检查系统紧急状态心跳超时，如果超时则不发布命令
  if (use_emergency_handling_ && is_emergency_state_heartbeat_timeout_) {

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/,"use_emergency_handling_ && is_emergency_state_heartbeat_timeout_  == true");
    return;
  }

  // 检查外部紧急停止，如果激活则不发布命令
  if (is_external_emergency_stop_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/,"is_external_emergency_stop_  == true");

    return;
  }

  // 检查数据是否准备就绪，如果未准备则不发布命令
  if (!isDataReady()) {

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/,"!isDataReady()  == true");

    return;
  }

  // ==================== 命令处理 ====================
  // 从输入命令中获取控制命令
  Control filtered_control = commands.control;

  // 如果温和停止服务请求停止，设置速度为0并使用温和停止加速度
  if (moderate_stop_interface_->is_stop_requested()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/,"moderate_stop_interface_  == true");
    filtered_control.longitudinal.velocity = 0.0;
    filtered_control.longitudinal.acceleration = moderate_stop_service_acceleration_;
  }

  // 如果启用紧急处理且系统处于紧急状态，使用紧急命令
  if (use_emergency_handling_ && is_system_emergency_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "is_system_emergency_: %s", is_system_emergency_ ? "true" : "false");
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "leslie Emergency!");
    filtered_control = emergency_commands_.control;
  }

  // 如果未engage（未接管控制），创建纵向停止命令
  // 这确保在未接管时车辆保持停止
  is_engaged_ = true;
  if (!is_engaged_) {
    filtered_control.longitudinal = createLongitudinalStopControlCmd();
  } 
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "is_engaged_: %s", is_engaged_ ? "true" : "false");

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "befor adapi_pause_->is_paused() \n filtered_control: longitudinal: velocity=%f, acceleration=%f, jerk=%f, lateral: steering_angle=%f, steering_angle_velocity=%f",
  filtered_control.longitudinal.velocity, 
  filtered_control.longitudinal.acceleration, 
  filtered_control.longitudinal.jerk, 
  filtered_control.lateral.steering_tire_angle, 
  filtered_control.lateral.steering_tire_rotation_rate);

  // ==================== 暂停检查 ====================
  // 注意：暂停检查放在其他检查之后，因为它需要最终的输出
  // 更新暂停接口（传入当前控制命令）
  adapi_pause_->update(filtered_control);
  if (adapi_pause_->is_paused()) {
    // 如果已暂停，根据engage状态选择停止方式
    if (is_engaged_) {
      // 如果已engage，只停止纵向运动
      filtered_control.longitudinal = createLongitudinalStopControlCmd();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "paused and engaged");
      
    } else {
      // 如果未engage，完全停止（包括横向）
      filtered_control = createStopControlCmd();
      filtered_control.stamp = commands.control.stamp;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "paused but not engaged");
    }
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "not paused");
  }

  // ==================== 命令过滤 ====================
  // 如果启用命令限制过滤器，应用过滤器
  // 过滤器会限制速度、加速度、转向角等，确保命令在安全范围内
  if (enable_cmd_limit_filter_) {
    // 应用限制过滤（速度、加速度、转向角等限制）
    filtered_control = filterControlCommand(filtered_control);
    filtered_control.stamp = commands.control.stamp;  // 保持原始时间戳
  }
  
  // ==================== 发布命令 ====================
  // 发布车辆紧急状态
  VehicleEmergencyStamped vehicle_cmd_emergency;
  vehicle_cmd_emergency.emergency = (use_emergency_handling_ && is_system_emergency_);
  vehicle_cmd_emergency.stamp = filtered_control.stamp;

  // 发布所有命令和状态
  
  vehicle_cmd_emergency_pub_->publish(vehicle_cmd_emergency);
  control_cmd_pub_->publish(filtered_control);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Longitudinal Control command after filtered: velocity=%f, acceleration=%f, jerk=%f, lateral: steering_angle=%f, steering_angle_velocity=%f",
  filtered_control.longitudinal.velocity, 
  filtered_control.longitudinal.acceleration, 
  filtered_control.longitudinal.jerk, 
  filtered_control.lateral.steering_tire_angle, 
  filtered_control.lateral.steering_tire_rotation_rate);

  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, filtered_control.stamp);
  adapi_pause_->publish();
  moderate_stop_interface_->publish();

  // 保存当前控制命令，用于在断开连接时保持转向角
  prev_control_cmd_ = filtered_control;
}

/**
 * @brief 发布紧急停止控制命令
 * 
 * 当检测到紧急状态（心跳超时、外部急停等）时，发布紧急停止命令。
 * 紧急停止命令包括：
 * - 控制命令：速度0，紧急加速度（较大的负值）
 * - 转向灯：无命令
 * - 危险警示灯：启用
 * - 档位：默认值（0）
 * - 紧急标志：设置为true
 */
void VehicleCmdGate::publishEmergencyStopControlCommands()
{
  const auto stamp = this->now();

  // 创建紧急停止控制命令
  Control control_cmd;
  control_cmd.stamp = stamp;
  control_cmd = createEmergencyStopControlCmd();

  // 更新暂停接口（即使紧急停止，也考虑暂停状态）
  adapi_pause_->update(control_cmd);

  // 创建档位命令（默认值为0）
  GearCommand gear;
  gear.stamp = stamp;
  // default value is 0

  // 创建转向灯命令（无命令）
  TurnIndicatorsCommand turn_indicator;
  turn_indicator.stamp = stamp;
  turn_indicator.command = TurnIndicatorsCommand::NO_COMMAND;

  // 创建危险警示灯命令（启用）
  HazardLightsCommand hazard_light;
  hazard_light.stamp = stamp;
  hazard_light.command = HazardLightsCommand::ENABLE;

  // 创建车辆紧急状态消息
  VehicleEmergencyStamped vehicle_cmd_emergency;
  vehicle_cmd_emergency.stamp = stamp;
  vehicle_cmd_emergency.emergency = true;

  // 发布所有命令
  vehicle_cmd_emergency_pub_->publish(vehicle_cmd_emergency);
  control_cmd_pub_->publish(control_cmd);
  turn_indicator_cmd_pub_->publish(turn_indicator);
  hazard_light_cmd_pub_->publish(hazard_light);
  gear_cmd_pub_->publish(gear);
}

/**
 * @brief 发布状态信息
 * 
 * 定期发布节点的状态信息，包括：
 * - 门控模式（AUTO/EXTERNAL）
 * - Engage状态（是否已接管）
 * - 外部紧急状态
 * - 操作模式状态
 * - 暂停接口状态
 * - 温和停止接口状态
 */
void VehicleCmdGate::publishStatus()
{
  const auto stamp = this->now();

  // Engage状态
  EngageMsg autoware_engage;
  autoware_engage.stamp = stamp;
  autoware_engage.engage = is_engaged_;

  // 外部紧急状态
  Emergency external_emergency;
  external_emergency.stamp = stamp;
  external_emergency.emergency = is_external_emergency_stop_;

  // 发布所有状态
  gate_mode_pub_->publish(current_gate_mode_);
  engage_pub_->publish(autoware_engage);
  pub_external_emergency_->publish(external_emergency);
  operation_mode_pub_->publish(current_operation_mode_);
  adapi_pause_->publish();
  moderate_stop_interface_->publish();
}

/**
 * @brief 过滤控制命令
 * @param in 输入的控制命令
 * @return 过滤后的控制命令
 * 
 * 应用命令限制过滤器，确保控制命令在安全范围内。
 * 过滤器会根据当前速度和操作模式，应用以下限制：
 * - 速度限制
 * - 纵向加速度限制
 * - 纵向急动度限制
 * - 横向加速度限制
 * - 横向急动度限制
 * - 转向角限制
 * - 转向角速度限制
 * - 实际转向角差值限制
 * 
 * 如果处于过渡模式（从MANUAL切换到AUTO），使用更严格的过渡过滤器。
 */
Control VehicleCmdGate::filterControlCommand(const Control & in)
{
  Control out = in;
  const double dt = getDt();  // 获取时间步长
  const auto mode = current_operation_mode_;
  const auto current_status_cmd = getActualStatusAsCommand();  // 获取当前实际状态作为命令
  const auto ego_is_stopped = vehicle_stop_checker_->isVehicleStopped(stop_check_duration_);  // 检查车辆是否已停止

  // 设置过滤器的当前速度
  filter_.setCurrentSpeed(current_kinematics_.twist.twist.linear.x);
  filter_on_transition_.setCurrentSpeed(current_kinematics_.twist.twist.linear.x);

  IsFilterActivated is_filter_activated;

  // 根据操作模式选择过滤器
  // 如果处于过渡模式（从MANUAL切换到AUTO），使用过渡过滤器（更严格的限制）
  if (mode.is_in_transition) {
    filter_on_transition_.filterAll(dt, current_steer_, out, is_filter_activated);
  } else {
    filter_.filterAll(dt, current_steer_, out, is_filter_activated);
  }

  // 设置上一帧命令值，用于保持切换时的一致性
  // 如果Autoware控制已启用，使用输出命令；否则使用当前实际状态
  // 这可以防止从手动切换到自动时产生突然运动
  auto prev_values = mode.is_autoware_control_enabled ? out : current_status_cmd;

  // 如果车辆已停止，更新纵向命令为当前输出
  if (ego_is_stopped) {
    prev_values.longitudinal = out.longitudinal;
  }

  // TODO(Horibe): 为了防止从手动切换到自动时的突然加减速，
  // 过滤器应该在手动驾驶期间应用于实际速度和加速度。
  // 然而，这意味着Gate的输出命令在手动驾驶期间将始终接近驾驶状态。
  // 这里，当车辆停止时，让autoware发布停止命令，以表示autoware正在尝试保持停止。

  // 更新两个过滤器的上一帧命令
  filter_.setPrevCmd(prev_values);
  filter_on_transition_.setPrevCmd(prev_values);

  // 发布过滤器激活状态（用于调试和可视化）
  is_filter_activated.stamp = now();
  is_filter_activated_pub_->publish(is_filter_activated);
  publishMarkers(is_filter_activated);

  return out;
}

/**
 * @brief 创建完全停止控制命令
 * @return 停止控制命令（包括纵向和横向）
 * 
 * 创建一个完全停止的控制命令：
 * - 速度：0
 * - 加速度：stop_hold_acceleration_（用于保持停止）
 * - 转向角：当前转向角（保持不变）
 * - 转向角速度：0
 */
Control VehicleCmdGate::createStopControlCmd() const
{
  Control cmd;
  const auto t = this->now();
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.longitudinal.stamp = t;
  cmd.lateral.steering_tire_angle = current_steer_;  // 保持当前转向角
  cmd.lateral.steering_tire_rotation_rate = 0.0;
  cmd.longitudinal.velocity = 0.0;
  cmd.longitudinal.acceleration = stop_hold_acceleration_;  // 停止保持加速度

  return cmd;
}

/**
 * @brief 创建纵向停止控制命令
 * @return 纵向停止命令（只包括纵向，不包括横向）
 * 
 * 创建一个只停止纵向运动的控制命令：
 * - 速度：0
 * - 加速度：stop_hold_acceleration_（用于保持停止）
 * 
 * 注意：横向（转向）不受影响，可以继续转向。
 */
Longitudinal VehicleCmdGate::createLongitudinalStopControlCmd() const
{
  Longitudinal cmd;
  const auto t = this->now();
  cmd.stamp = t;
  cmd.velocity = 0.0;
  cmd.acceleration = stop_hold_acceleration_;  // 停止保持加速度

  return cmd;
}

Control VehicleCmdGate::createEmergencyStopControlCmd() const
{
  Control cmd;
  const auto t = this->now();
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.longitudinal.stamp = t;
  cmd.lateral.steering_tire_angle = prev_control_cmd_.lateral.steering_tire_angle;
  cmd.lateral.steering_tire_rotation_rate = prev_control_cmd_.lateral.steering_tire_rotation_rate;
  cmd.longitudinal.velocity = 0.0;
  cmd.longitudinal.acceleration = emergency_acceleration_;

  return cmd;
}

void VehicleCmdGate::onExternalEmergencyStopHeartbeat(Heartbeat::ConstSharedPtr msg)
{
  if (msg->ready) {
    external_emergency_stop_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
  }
}

void VehicleCmdGate::onGateMode(GateMode::ConstSharedPtr msg)
{
  const auto prev_gate_mode = current_gate_mode_;
  current_gate_mode_ = *msg;

  if (current_gate_mode_.data != prev_gate_mode.data) {
    RCLCPP_INFO(
      get_logger(), "GateMode changed: %s -> %s", getGateModeName(prev_gate_mode.data),
      getGateModeName(current_gate_mode_.data));
  }
}

void VehicleCmdGate::onEngage(EngageMsg::ConstSharedPtr msg)
{
  is_engaged_ = msg->engage;
}

void VehicleCmdGate::onEngageService(
  const EngageSrv::Request::SharedPtr request, const EngageSrv::Response::SharedPtr response)
{
  is_engaged_ = request->engage;
  response->status = tier4_api_utils::response_success();
}

void VehicleCmdGate::onMrmState(MrmState::ConstSharedPtr msg)
{
  is_system_emergency_ =
    (msg->state == MrmState::MRM_OPERATING || msg->state == MrmState::MRM_SUCCEEDED ||
     msg->state == MrmState::MRM_FAILED) &&
    (msg->behavior == MrmState::EMERGENCY_STOP);
  emergency_state_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
}

double VehicleCmdGate::getDt()
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    return 0.0;
  }

  const auto current_time = this->now();
  const auto dt = (current_time - *prev_time_).seconds();
  *prev_time_ = current_time;

  return dt;
}

Control VehicleCmdGate::getActualStatusAsCommand()
{
  Control status;
  status.stamp = status.lateral.stamp = status.longitudinal.stamp = this->now();
  status.lateral.steering_tire_angle = current_steer_;
  status.lateral.steering_tire_rotation_rate = 0.0;
  status.longitudinal.velocity = current_kinematics_.twist.twist.linear.x;
  status.longitudinal.acceleration = current_acceleration_;
  return status;
}

void VehicleCmdGate::onExternalEmergencyStopService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetEmergency::Request::SharedPtr request, const SetEmergency::Response::SharedPtr response)
{
  auto req = std::make_shared<Trigger::Request>();
  auto res = std::make_shared<Trigger::Response>();
  if (request->emergency) {
    onSetExternalEmergencyStopService(request_header, req, res);
  } else {
    onClearExternalEmergencyStopService(request_header, req, res);
  }

  if (res->success) {
    response->status = tier4_api_utils::response_success(res->message);
  } else {
    response->status = tier4_api_utils::response_error(res->message);
  }
}

bool VehicleCmdGate::onSetExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
{
  is_external_emergency_stop_ = true;
  res->success = true;
  res->message = "external_emergency_stop requested was accepted.";

  return true;
}

bool VehicleCmdGate::onClearExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
{
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      is_external_emergency_stop_ = false;
      res->success = true;
      res->message = "external_emergency_stop state was cleared.";
    } else {
      res->success = false;
      res->message = "Couldn't clear external_emergency_stop state because heartbeat is timeout.";
    }
  } else {
    res->success = false;
    res->message = "Not in external_emergency_stop state.";
  }

  return true;
}

void VehicleCmdGate::checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DiagnosticStatus status;
  if (is_external_emergency_stop_heartbeat_timeout_) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "external_emergency_stop heartbeat is timeout.";
  } else if (is_external_emergency_stop_) {
    status.level = DiagnosticStatus::ERROR;
    status.message =
      "external_emergency_stop is required. Please call `clear_external_emergency_stop` service "
      "to "
      "clear state.";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

MarkerArray VehicleCmdGate::createMarkerArray(const IsFilterActivated & filter_activated)
{
  MarkerArray msg;

  if (!filter_activated.is_activated) {
    return msg;
  }

  /* add string marker */
  bool first_msg = true;
  std::string reason = "filter activated on";

  if (filter_activated.is_activated_on_acceleration) {
    reason += " lon_acc";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_jerk) {
    reason += first_msg ? " jerk" : ", jerk";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_speed) {
    reason += first_msg ? " speed" : ", speed";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_steering) {
    reason += first_msg ? " steer" : ", steer";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_steering_rate) {
    reason += first_msg ? " steer_rate" : ", steer_rate";
  }

  msg.markers.emplace_back(createStringMarker(
    "base_link", "msg", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    createMarkerPosition(0.0, 0.0, 1.0), createMarkerScale(0.0, 0.0, 1.0),
    createMarkerColor(1.0, 0.0, 0.0, 1.0), reason));

  /* add sphere marker */
  msg.markers.emplace_back(createMarker(
    "base_link", "sphere", 0, visualization_msgs::msg::Marker::SPHERE,
    createMarkerPosition(0.0, 0.0, 0.0), createMarkerScale(3.0, 3.0, 3.0),
    createMarkerColor(1.0, 0.0, 0.0, 0.3)));

  return msg;
}

void VehicleCmdGate::publishMarkers(const IsFilterActivated & filter_activated)
{
  BoolStamped filter_activated_flag;
  if (filter_activated.is_activated) {
    filter_activated_count_++;
  } else {
    filter_activated_count_ = 0;
  }
  if (
    filter_activated_count_ >= filter_activated_count_threshold_ &&
    std::fabs(current_kinematics_.twist.twist.linear.x) >= filter_activated_velocity_threshold_ &&
    current_operation_mode_.mode == OperationModeState::AUTONOMOUS) {
    filter_activated_marker_pub_->publish(createMarkerArray(filter_activated));
    filter_activated_flag.data = true;
  } else {
    filter_activated_flag.data = false;
  }

  filter_activated_flag.stamp = now();
  filter_activated_flag_pub_->publish(filter_activated_flag);
  filter_activated_marker_raw_pub_->publish(createMarkerArray(filter_activated));
}
}  // namespace autoware::vehicle_cmd_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_cmd_gate::VehicleCmdGate)
