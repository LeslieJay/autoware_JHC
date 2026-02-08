// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/trajectory_follower_node/controller_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"
#include "autoware_utils/ros/marker_helper.hpp"

#include <autoware/trajectory_follower_base/lateral_controller_base.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/**
 * @file controller_node.cpp
 * @brief 轨迹跟踪控制节点实现文件。负责自动驾驶车辆的横向与纵向控制命令的周期性计算、同步和下发，并支持调试及实验性的控制批量输出。
 *
 * ## 总体流程与内部逻辑
 * 1. 控制器节点初始化，根据参数选择横向和纵向控制算法，创建控制/调试/marker等发布器，启动定时控制循环。
 * 2. 定时回调周期性执行控制主流程（`callbackTimerControl`）：
 *    1) 查询车辆当前状态及参考轨迹等输入数据，判断数据完整性。
 *    2) 检查横向/纵向控制器是否准备好。
 *    3) 分别调用横向和纵向控制器得到命令输出；
 *    4) 控制器之间数据同步；
 *    5) 超时检查，若命令过时则不发布新命令；
 *    6) 发布控制命令，调试信息（处理时间、可视化marker等）、实验批量命令等。
 *
 * 主要函数如下方详细注释。
 */

namespace
{
// ==========================
// 辅助函数
// ==========================
/**
 * @brief 将控制/命令的horizon按新步长做零阶保持采样
 * @tparam T 控制/命令类型
 * @param original_horizon 原始控制序列
 * @param original_time_step_ms 原始步长(ms)
 * @param new_time_step_ms 新的步长(ms)
 * @return 采样完成的新序列
 */
template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
  const std::vector<T> & original_horizon, const double original_time_step_ms,
  const double new_time_step_ms)
{
  std::vector<T> resampled_horizon{};
  const size_t step_factor = static_cast<size_t>(original_time_step_ms / new_time_step_ms);
  const size_t resampled_size = original_horizon.size() * step_factor;
  resampled_horizon.reserve(resampled_size);
  for (const auto & command : original_horizon) {
    for (size_t i = 0; i < step_factor; ++i) {
      resampled_horizon.push_back(command);
    }
  }
  return resampled_horizon;
}
}  // namespace

namespace autoware::motion::control::trajectory_follower_node
{
// ==========================
// Controller 构造函数
// ==========================
/**
 * @brief 控制节点构造函数，加载参数，选择控制器，初始化发布器和定时器
 * @param node_options ROS2节点参数
 */
Controller::Controller(const rclcpp::NodeOptions & node_options) : Node("controller", node_options)
{
  using std::placeholders::_1;

  // 控制周期等参数
  const double ctrl_period = declare_parameter<double>("ctrl_period");
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec");
  enable_control_cmd_horizon_pub_ =
    declare_parameter<bool>("enable_control_cmd_horizon_pub", false);

  diag_updater_->setHardwareID("trajectory_follower_node");

  // (1) 横向控制器选择，如MPC或Pure Pursuit
  const auto lateral_controller_mode =
    getLateralControllerMode(declare_parameter<std::string>("lateral_controller_mode", "mpc"));
  switch (lateral_controller_mode) {
    case LateralControllerMode::MPC: {
      lateral_controller_ =
        std::make_shared<mpc_lateral_controller::MpcLateralController>(*this, diag_updater_);
      break;
    }
    case LateralControllerMode::PURE_PURSUIT: {
      lateral_controller_ =
        std::make_shared<autoware::pure_pursuit::PurePursuitLateralController>(*this);
      break;
    }
    default:
      throw std::domain_error("[LateralController] invalid algorithm");
  }

  // (2) 纵向控制器选择，如PID
  const auto longitudinal_controller_mode =
    getLongitudinalControllerMode(declare_parameter<std::string>("longitudinal_controller_mode", "pid"));
  switch (longitudinal_controller_mode) {
    case LongitudinalControllerMode::PID: {
      longitudinal_controller_ =
        std::make_shared<pid_longitudinal_controller::PidLongitudinalController>(
          *this, diag_updater_);
      break;
    }
    default:
      throw std::domain_error("[LongitudinalController] invalid algorithm");
  }

  // (3) 控制命令与调试相关发布器
  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());
  pub_processing_time_lat_ms_ =
    create_publisher<Float64Stamped>("~/lateral/debug/processing_time_ms", 1);
  pub_processing_time_lon_ms_ =
    create_publisher<Float64Stamped>("~/longitudinal/debug/processing_time_ms", 1);
  debug_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker", rclcpp::QoS{1});

  // (4) 实验性：控制批量命令序列发布
  if (enable_control_cmd_horizon_pub_) {
    control_cmd_horizon_pub_ = create_publisher<autoware_control_msgs::msg::ControlHorizon>(
      "~/debug/control_cmd_horizon", 1);
  }

  // (5) 启动定时器——主控制循环及日志/时间发布等
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

// ==========================
// 获取横向控制器模式
// ==========================
/**
 * @brief 根据参数字符串返回横向控制器枚举类型
 * @param controller_mode 控制器模式字符串（"mpc", "pure_pursuit"）
 * @return 对应枚举
 */
Controller::LateralControllerMode Controller::getLateralControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "mpc") return LateralControllerMode::MPC;
  if (controller_mode == "pure_pursuit") return LateralControllerMode::PURE_PURSUIT;
  return LateralControllerMode::INVALID;
}

// ==========================
// 获取纵向控制器模式
// ==========================
/**
 * @brief 根据参数字符串返回纵向控制器枚举类型
 * @param controller_mode 控制器模式字符串（"pid"）
 * @return 对应枚举
 */
Controller::LongitudinalControllerMode Controller::getLongitudinalControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "pid") return LongitudinalControllerMode::PID;
  return LongitudinalControllerMode::INVALID;
}

// ==========================
// 输入数据准备（状态/轨迹等）
// ==========================
/**
 * @brief 检查和提取当前的加速度/转向/轨迹/里程计/模式等数据
 * @param clock 当前clock对象（用于打印节流log）
 * @return 所有数据准备好返回true，否则false
 */
bool Controller::processData(rclcpp::Clock & clock)
{
  bool is_ready = true;

  // 对指定类型数据如果未获取到则周期性打印提示信息
  const auto & logData = [&clock, this](const std::string & data_type) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval, "Waiting for %s data", data_type.c_str());
  };

  // 尝试从topic抓取最新数据，并填到dest参数，未拿到则日志提示
  const auto & getData = [&logData](auto & dest, auto & sub, const std::string & data_type = "") {
    const auto temp = sub.take_data();
    if (temp) {
      dest = temp;
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  is_ready &= getData(current_accel_ptr_, sub_accel_, "acceleration");
  is_ready &= getData(current_steering_ptr_, sub_steering_, "steering");
  is_ready &= getData(current_trajectory_ptr_, sub_ref_path_, "trajectory");
  is_ready &= getData(current_odometry_ptr_, sub_odometry_, "odometry");
  is_ready &= getData(current_operation_mode_ptr_, sub_operation_mode_, "operation mode");

  return is_ready;
}

// ==========================
// 超时判定
// ==========================
/**
 * @brief 命令超时检测。检查上一次横/纵向控制命令时间戳与当前的间隔是否超过阈值
 * @param lon_out 纵向控制输出
 * @param lat_out 横向控制输出
 * @return 只要有过时则返回true，否则false
 */
bool Controller::isTimeOut(
  const trajectory_follower::LongitudinalOutput & lon_out,
  const trajectory_follower::LateralOutput & lat_out)
{
  const auto now = this->now();
  if ((now - lat_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Lateral control command too old, control_cmd will not be published.");
    return true;
  }
  if ((now - lon_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Longitudinal control command too old, control_cmd will not be published.");
    return true;
  }
  return false;
}

// ==========================
// 输入数据包装
// ==========================
/**
 * @brief 调用processData并打包为InputData, 若无效则返回空optional
 * @param clock 当前clock对象
 * @return boost::optional<trajectory_follower::InputData>
 */
boost::optional<trajectory_follower::InputData> Controller::createInputData(rclcpp::Clock & clock)
{
  if (!processData(clock)) {
    return {};
  }

  trajectory_follower::InputData input_data;
  input_data.current_trajectory = *current_trajectory_ptr_;
  input_data.current_odometry = *current_odometry_ptr_;
  input_data.current_steering = *current_steering_ptr_;
  input_data.current_accel = *current_accel_ptr_;
  input_data.current_operation_mode = *current_operation_mode_ptr_;

  return input_data;
}

// ==========================
// 定时主控制回调函数
// ==========================
/**
 * @brief 主控制流程定时回调（周期性），执行感知输入检查，控制器运行，同步，命令/调试信息发布等
 * @details
 * 1）构造InputData感知状态
 * 2）检查横纵控制器准备状态
 * 3）分别运行横向和纵向控制，计时
 * 4）同步两个控制器
 * 5）新老命令超时检测
 * 6）发布控制命令
 * 7）发布调试可视化与处理时间
 * 8）实验性批量命令(可选)
 */
void Controller::callbackTimerControl()
{
  autoware_control_msgs::msg::Control out;
  out.stamp = this->now();

  // 1. create input data
  const auto input_data = createInputData(*get_clock());
  if (!input_data) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since input data is not ready.");
    return;
  }

  // 2. check if controllers are ready
  const bool is_lat_ready = lateral_controller_->isReady(*input_data);
  const bool is_lon_ready = longitudinal_controller_->isReady(*input_data);
  if (!is_lat_ready || !is_lon_ready) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Control is skipped since lateral and/or longitudinal controllers are not ready to run.");
    return;
  }

  // 3. run controllers
  stop_watch_.tic("lateral");
  const auto lat_out = lateral_controller_->run(*input_data);
  publishProcessingTime(stop_watch_.toc("lateral"), pub_processing_time_lat_ms_);

  stop_watch_.tic("longitudinal");
  const auto lon_out = longitudinal_controller_->run(*input_data);
  publishProcessingTime(stop_watch_.toc("longitudinal"), pub_processing_time_lon_ms_);

  // 4. sync with each other controllers
  longitudinal_controller_->sync(lat_out.sync_data);
  lateral_controller_->sync(lon_out.sync_data);

  // 5. timeout check
  if (isTimeOut(lon_out, lat_out)) return;

  // 6. publish control command
  // 打印lat_out和lon_out的详细信息

  out.lateral = lat_out.control_cmd;
  out.longitudinal = lon_out.control_cmd;

  // 打印out.longitudinal的详细信息
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Publishing longitudinal control command to vehicle_cmd_gate: "
    "velocity=%.6f, acceleration=%.6f, jerk=%.6f, is_defined_accel=%d, is_defined_jerk=%d",
    out.longitudinal.velocity,
    out.longitudinal.acceleration,
    out.longitudinal.jerk,
    out.longitudinal.is_defined_acceleration,
    out.longitudinal.is_defined_jerk);

  control_cmd_pub_->publish(out);

  // 7. publish debug
  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, out.stamp);
  publishDebugMarker(*input_data, lat_out);

  // 8. optionally publish control horizon (for experiment)
  if (enable_control_cmd_horizon_pub_) {
    const auto control_horizon =
      mergeLatLonHorizon(lat_out.control_cmd_horizon, lon_out.control_cmd_horizon, this->now());
    if (control_horizon.has_value()) {
      control_cmd_horizon_pub_->publish(control_horizon.value());
    }
  }
}

// ==========================
// marker调试信息发布
// ==========================
/**
 * @brief 发布可视化marker，用于在RViz等工具呈现方向盘收敛状态等调试信息
 * @param input_data 当前输入数据
 * @param lat_out 横向控制输出
 */
void Controller::publishDebugMarker(
  const trajectory_follower::InputData & input_data,
  const trajectory_follower::LateralOutput & lat_out) const
{
  visualization_msgs::msg::MarkerArray debug_marker_array{};

  // steer converged marker，可视化方向盘是否与目标指令收敛
  {
    auto marker = autoware_utils::create_default_marker(
      "map", this->now(), "steer_converged", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 1.0),
      autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.99));
    marker.pose = input_data.current_odometry.pose.pose;

    std::stringstream ss;
    const double current = input_data.current_steering.steering_tire_angle;
    const double cmd = lat_out.control_cmd.steering_tire_angle;
    const double diff = current - cmd;
    ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
       << (lat_out.sync_data.is_steer_converged ? " (converged)" : " (not converged)");
    marker.text = ss.str();

    debug_marker_array.markers.push_back(marker);
  }

  debug_marker_pub_->publish(debug_marker_array);
}

// ==========================
// 处理时间调试信息发布
// ==========================
/**
 * @brief 发布每次横纵控制器处理时间(ms)到调试topic
 * @param t_ms 处理耗时(ms)
 * @param pub 对应Publisher
 */
void Controller::publishProcessingTime(
  const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = t_ms;
  pub->publish(msg);
}

// ==========================
// 批量命令horizon横纵合并
// ==========================
/**
 * @brief 合并横向与纵向控制horizon，输出带齐时间戳的批量ControlHorizon
 * @details 若某一horizon只有1个指令，则重复该指令；否则同步步长，各自零阶保持补齐。若仍不同步或有空，则返回空
 * @param lateral_horizon 横向控制器输出的horizon
 * @param longitudinal_horizon 纵向控制器输出的horizon
 * @param stamp ROS时间戳
 * @return std::optional<ControlHorizon>
 */
std::optional<ControlHorizon> Controller::mergeLatLonHorizon(
  const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp)
{
  // 检查输入horizon有效性
  if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
    return std::nullopt;
  }

  autoware_control_msgs::msg::ControlHorizon control_horizon{};
  control_horizon.stamp = stamp;

  // 若某一方只有1个指令，重复该指令补齐另一方长度
  if (lateral_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
    const auto lateral = lateral_horizon.controls.front();
    for (const auto & longitudinal : longitudinal_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }
  if (longitudinal_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = lateral_horizon.time_step_ms;
    const auto longitudinal = longitudinal_horizon.controls.front();
    for (const auto & lateral : lateral_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }

  // 双方horizon均为多步，步长lcm后零阶保持采样对齐
  // 步长取最大公约数
  const auto gcd_double = [](const double a, const double b) {
    const double precision = 1e9;
    const int int_a = static_cast<int>(round(a * precision));
    const int int_b = static_cast<int>(round(b * precision));
    return static_cast<double>(std::gcd(int_a, int_b)) / precision;
  };
  const double time_step_ms =
    gcd_double(lateral_horizon.time_step_ms, longitudinal_horizon.time_step_ms);
  control_horizon.time_step_ms = time_step_ms;

  const auto lateral_controls = resampleHorizonByZeroOrderHold(
    lateral_horizon.controls, lateral_horizon.time_step_ms, time_step_ms);
  const auto longitudinal_controls = resampleHorizonByZeroOrderHold(
    longitudinal_horizon.controls, longitudinal_horizon.time_step_ms, time_step_ms);

  // 若最终长度不同步则失败
  if (lateral_controls.size() != longitudinal_controls.size()) {
    return std::nullopt;
  }

  const size_t num_steps = lateral_controls.size();
  for (size_t i = 0; i < num_steps; ++i) {
    autoware_control_msgs::msg::Control control{};
    control.stamp = stamp;
    control.lateral = lateral_controls.at(i);
    control.longitudinal = longitudinal_controls.at(i);
    control_horizon.controls.push_back(control);
  }
  return control_horizon;
}

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_node::Controller)
