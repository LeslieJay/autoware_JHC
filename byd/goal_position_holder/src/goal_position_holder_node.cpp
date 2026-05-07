// Copyright 2026 BYD
//
// 末段位置闭环节点（纵向 + 横向，目标精度 ±2 cm）
// ------------------------------------------------
// 在车辆接近 goal 末点时同时接管纵向和横向控制：
//   * 纵向：基于沿 goal_yaw 投影的剩余位移 s_long 做 PD，必要时蠕行(creep)
//     克服制动死区。
//   * 横向：基于横向偏差 s_lat 与航向偏差 yaw_err 做 P 控制，输出
//     steering_tire_angle，让车在末段的小幅蠕行/制动过程中把横向误差也
//     压到 ±2 cm。
//
// 拓扑：
//   trajectory_follower → /control/trajectory_follower/control_cmd
//        ↓ (本节点订阅)
//   goal_position_holder
//        ↓ 发布
//   /control/trajectory_follower/control_cmd_holded
//        ↓ vehicle_cmd_gate input/auto/control_cmd
//
// 模式：
//   PASSTHROUGH (默认) —— 直接转发上游 control_cmd
//   HOLD（满足以下全部条件时进入）：
//     - trajectory 末点距 goal < traj_to_goal_max
//     - |s_long| < activation_distance
//     - |ego_v| < activation_speed
//   HOLD 内部：
//     - 已收敛(|s_long|<tol_pos & |s_lat|<tol_lat & |yaw_err|<tol_yaw &
//       |ego_v|<tol_vel)：v=0, a=brake_acc, 保持当前直线方向盘(δ=0)
//     - 接近停止但未收敛：creep（小速度向能减小最大残差的方向推），
//       同时叠加横向 P；不允许倒车则向后侧改为强制刹停
//     - 否则：纵向 PD + 横向 P
//   横向 P:  δ = clamp( -k_lat · s_lat - k_yaw · yaw_err , ±max_steer )
//   符号约定：goal 系下 s_lat>0 表示 ego 在 goal 方向左侧；steering>0=左转。
//   所以 s_lat>0 → 应右打方向(δ<0) → 系数取负号即可。

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace goal_position_holder
{

class GoalPositionHolderNode : public rclcpp::Node
{
public:
  GoalPositionHolderNode() : Node("goal_position_holder")
  {
    // —— 通用参数 ——
    enable_ = declare_parameter<bool>("enable", true);
    activation_distance_ = declare_parameter<double>("activation_distance", 3.0);
    activation_speed_ = declare_parameter<double>("activation_speed", 1.0);
    traj_to_goal_max_ = declare_parameter<double>("traj_to_goal_max", 0.5);

    // —— 纵向参数 ——
    kp_ = declare_parameter<double>("kp", 0.8);
    kv_ = declare_parameter<double>("kv", 3.0);
    a_min_ = declare_parameter<double>("a_min", -2.0);
    a_max_ = declare_parameter<double>("a_max", 0.5);
    v_max_ = declare_parameter<double>("v_max", 1.0);
    dt_lookahead_ = declare_parameter<double>("dt_lookahead", 0.1);
    tol_pos_ = declare_parameter<double>("tol_pos", 0.02);
    tol_vel_ = declare_parameter<double>("tol_vel", 0.05);
    brake_acc_ = declare_parameter<double>("brake_acc", -1.5);

    // creep（蠕行）：车已停但未到容差时，强制小速度推过去克服死区
    creep_speed_thresh_ = declare_parameter<double>("creep_speed_thresh", 0.03);
    creep_v_ = declare_parameter<double>("creep_v", 0.08);
    creep_acc_ = declare_parameter<double>("creep_acc", 0.15);

    // —— 横向参数 ——
    enable_lateral_ = declare_parameter<bool>("enable_lateral", true);
    k_lat_ = declare_parameter<double>("k_lat", 0.6);          // [rad/m]
    k_yaw_ = declare_parameter<double>("k_yaw", 1.0);          // [rad/rad]
    max_steer_ = declare_parameter<double>("max_steer", 0.6);  // [rad]
    tol_lat_ = declare_parameter<double>("tol_lat", 0.02);     // [m]
    tol_yaw_ = declare_parameter<double>("tol_yaw", 0.02);     // [rad] ≈1.15°
    // 选择 creep 方向时，用此系数把横向误差等价折算到纵向；
    // 若纯纵向已到位但横向未收敛，仍可允许小幅蠕行以借助前后移动修正横向。
    lat_to_long_weight_ = declare_parameter<double>("lat_to_long_weight", 1.0);

    // —— 订阅 ——
    sub_cmd_ = create_subscription<autoware_control_msgs::msg::Control>(
      "~/input/control_cmd", rclcpp::QoS(1),
      [this](autoware_control_msgs::msg::Control::SharedPtr msg) { on_control_cmd(*msg); });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/kinematic_state", rclcpp::QoS(1),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = *msg; });

    sub_traj_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", rclcpp::QoS(1),
      [this](autoware_planning_msgs::msg::Trajectory::SharedPtr msg) { latest_traj_ = *msg; });

    rclcpp::QoS goal_qos(1);
    goal_qos.transient_local();
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/input/goal_pose", goal_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_goal_ = *msg; });

    // —— 发布 ——
    pub_cmd_ = create_publisher<autoware_control_msgs::msg::Control>(
      "~/output/control_cmd", rclcpp::QoS(1));

    RCLCPP_INFO(get_logger(),
                "goal_position_holder up. enable=%s lateral=%s "
                "kp=%.2f kv=%.2f k_lat=%.2f k_yaw=%.2f tol_pos=%.3f tol_lat=%.3f",
                enable_ ? "true" : "false",
                enable_lateral_ ? "true" : "false",
                kp_, kv_, k_lat_, k_yaw_, tol_pos_, tol_lat_);
  }

private:
  // ===== 工具 =====
  static double yaw_from_quat(double qz, double qw)
  {
    return 2.0 * std::atan2(qz, qw);
  }

  static double normalize_angle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // 返回 (s_long, s_lat)，沿 ref_yaw 投影
  static std::pair<double, double> project(
    double dx, double dy, double ref_yaw)
  {
    const double fx = std::cos(ref_yaw), fy = std::sin(ref_yaw);
    const double s_long = dx * fx + dy * fy;
    const double s_lat = -dx * fy + dy * fx;
    return {s_long, s_lat};
  }

  // ===== 主回调 =====
  void on_control_cmd(const autoware_control_msgs::msg::Control & cmd_in)
  {
    auto out = cmd_in;  // 默认透传，含横向

    if (!enable_) {
      pub_cmd_->publish(out);
      return;
    }

    auto override = compute_hold_override();
    if (override.has_value()) {
      const auto & ov = *override;
      out.longitudinal.velocity = ov.v_cmd;
      out.longitudinal.acceleration = ov.a_cmd;
      if (enable_lateral_ && ov.lateral_valid) {
        out.lateral.steering_tire_angle = static_cast<float>(ov.steer_cmd);
        // 给一个温和的转向速率上限，避免阶跃突变
        out.lateral.steering_tire_rotation_rate = 0.5f;
        out.lateral.is_defined_steering_tire_rotation_rate = true;
      }
      if (!hold_logged_) {
        RCLCPP_INFO(get_logger(),
                    "[HOLD] enter: 末段位置闭环接管 "
                    "(s_long=%.3f s_lat=%.3f yaw_err=%.3f v=%.3f)",
                    last_s_long_, last_s_lat_, last_yaw_err_, last_ego_v_);
        hold_logged_ = true;
      }
    } else {
      if (hold_logged_) {
        RCLCPP_INFO(get_logger(), "[HOLD] exit: 退出末段位置闭环");
        hold_logged_ = false;
      }
    }

    pub_cmd_->publish(out);
  }

  struct Override
  {
    double v_cmd = 0.0;
    double a_cmd = 0.0;
    double steer_cmd = 0.0;
    bool lateral_valid = false;
  };

  std::optional<Override> compute_hold_override()
  {
    if (!latest_odom_.has_value() || !latest_traj_.has_value() ||
        !latest_goal_.has_value()) {
      return std::nullopt;
    }
    const auto & traj = *latest_traj_;
    if (traj.points.empty()) return std::nullopt;

    const auto & goal_pose = latest_goal_->pose;
    const double gx = goal_pose.position.x;
    const double gy = goal_pose.position.y;
    const double goal_yaw =
      yaw_from_quat(goal_pose.orientation.z, goal_pose.orientation.w);

    // 1) 规划末点是否已收敛到 goal 附近？
    const auto & last_pt = traj.points.back().pose.position;
    const double d_traj_goal = std::hypot(last_pt.x - gx, last_pt.y - gy);
    if (d_traj_goal > traj_to_goal_max_) {
      return std::nullopt;
    }

    // 2) ego 状态
    const auto & op = latest_odom_->pose.pose.position;
    const auto & oq = latest_odom_->pose.pose.orientation;
    const double ego_v = latest_odom_->twist.twist.linear.x;
    const double ego_yaw = yaw_from_quat(oq.z, oq.w);
    const auto [s_long, s_lat] = project(op.x - gx, op.y - gy, goal_yaw);
    const double yaw_err = normalize_angle(ego_yaw - goal_yaw);
    last_s_long_ = s_long;
    last_s_lat_ = s_lat;
    last_yaw_err_ = yaw_err;
    last_ego_v_ = ego_v;

    // 3) 距 goal 还远 / 速度还大 → 不接管
    if (std::abs(s_long) > activation_distance_) return std::nullopt;
    if (std::abs(ego_v) > activation_speed_) return std::nullopt;

    // —— 横向 P 控制（HOLD 内统一使用） ——
    // s_lat>0 (ego 在 goal 左侧) → 需要右打方向 → δ<0 → 系数取负
    const double steer_raw = -k_lat_ * s_lat - k_yaw_ * yaw_err;
    const double steer_cmd = std::clamp(steer_raw, -max_steer_, max_steer_);

    Override out;
    out.steer_cmd = steer_cmd;
    out.lateral_valid = true;

    const bool lat_converged =
      (std::abs(s_lat) < tol_lat_) && (std::abs(yaw_err) < tol_yaw_);
    const bool long_converged =
      (std::abs(s_long) < tol_pos_) && (std::abs(ego_v) < tol_vel_);

    // 4) 全收敛 → 强制刹停 + 方向盘回正（避免静止打方向）
    if (lat_converged && long_converged) {
      out.v_cmd = 0.0;
      out.a_cmd = brake_acc_;
      out.steer_cmd = 0.0;
      return out;
    }

    // 4.5) 几乎停下但未收敛 → 蠕行
    //      用 s_long 与折算后的 s_lat 共同决定是否还需要前/后蠕行。
    //      为修正横向误差，借助一段小幅前进让转向作用起来。
    if (std::abs(ego_v) < creep_speed_thresh_) {
      // 残差合成：纵向想前进(s_long<0)记正；横向未收敛时也允许蠕行
      const double residual = -s_long
        + ((!lat_converged) ? lat_to_long_weight_ * std::abs(s_lat) : 0.0);
      // 不允许倒车：residual<=0 时（已越过 goal 或仅纵向到位）→ 刹停
      if (residual > tol_pos_) {
        out.v_cmd = creep_v_;
        out.a_cmd = creep_acc_;
      } else {
        out.v_cmd = 0.0;
        out.a_cmd = brake_acc_;
      }
      return out;
    }

    // 5) 标准纵向 PD
    double a_cmd = -kp_ * s_long - kv_ * ego_v;
    a_cmd = std::clamp(a_cmd, a_min_, a_max_);

    double v_cmd = ego_v + a_cmd * dt_lookahead_;
    if (v_cmd < 0.0) v_cmd = 0.0;       // 不允许倒车
    if (v_cmd > v_max_) v_cmd = v_max_;
    if (s_long > 0.0) {                 // 已越过 goal → 强刹
      v_cmd = 0.0;
      a_cmd = brake_acc_;
    }
    out.v_cmd = v_cmd;
    out.a_cmd = a_cmd;
    return out;
  }

  // —— 参数 ——
  bool enable_;
  double activation_distance_, activation_speed_, traj_to_goal_max_;
  // 纵向
  double kp_, kv_;
  double a_min_, a_max_, v_max_, dt_lookahead_;
  double tol_pos_, tol_vel_, brake_acc_;
  double creep_speed_thresh_, creep_v_, creep_acc_;
  // 横向
  bool enable_lateral_;
  double k_lat_, k_yaw_, max_steer_, tol_lat_, tol_yaw_, lat_to_long_weight_;

  // —— 状态 ——
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<autoware_planning_msgs::msg::Trajectory> latest_traj_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  bool hold_logged_ = false;
  double last_s_long_ = 0.0;
  double last_s_lat_ = 0.0;
  double last_yaw_err_ = 0.0;
  double last_ego_v_ = 0.0;

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_cmd_;
};

}  // namespace goal_position_holder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<goal_position_holder::GoalPositionHolderNode>());
  rclcpp::shutdown();
  return 0;
}
