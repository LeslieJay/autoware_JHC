// Copyright 2026 BYD
//
// 末点对齐节点
// ------------
// 作用：把 trajectory 末点替换为 goal_pose 的精确 (x, y, yaw)，消除 planning
// 离散化（output_resolution ~0.5 m, resample_ds ~0.1 m）造成的末点偏移。
//
// 拓扑：
//   velocity_smoother → /planning/scenario_planning/trajectory
//        ↓ (本节点订阅)
//   goal_exact_point_appender
//        ↓ 发布
//   /planning/scenario_planning/trajectory_goal_aligned
//        ↓ trajectory_follower（input/reference_trajectory remap）
//
// 行为：
//   未收到 goal_pose / 节点 disabled / 末点距 goal > activation_distance
//     → 透传
//   末点距 goal ≤ activation_distance：
//     - strategy = "replace"：把末点 pose 替换为 goal（保留原末点的速度=0）
//     - strategy = "insert" ：在末点后追加一个 goal 点（速度=0）

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace goal_exact_point_appender
{

class GoalExactPointAppenderNode : public rclcpp::Node
{
public:
  GoalExactPointAppenderNode() : Node("goal_exact_point_appender")
  {
    enable_ = declare_parameter<bool>("enable", true);
    activation_distance_ = declare_parameter<double>("activation_distance", 0.5);
    strategy_ = declare_parameter<std::string>("strategy", "replace");
    force_zero_velocity_at_goal_ =
      declare_parameter<bool>("force_zero_velocity_at_goal", true);

    if (strategy_ != "replace" && strategy_ != "insert") {
      RCLCPP_WARN(get_logger(),
                  "unknown strategy '%s', fallback to 'replace'", strategy_.c_str());
      strategy_ = "replace";
    }

    sub_traj_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", rclcpp::QoS(1),
      [this](autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
        on_trajectory(*msg);
      });

    rclcpp::QoS goal_qos(1);
    goal_qos.transient_local();
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/input/goal_pose", goal_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_goal_ = *msg; });

    pub_traj_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", rclcpp::QoS(1));

    RCLCPP_INFO(get_logger(),
                "goal_exact_point_appender up. enable=%s strategy=%s "
                "activation_distance=%.2f",
                enable_ ? "true" : "false", strategy_.c_str(),
                activation_distance_);
  }

private:
  void on_trajectory(const autoware_planning_msgs::msg::Trajectory & traj_in)
  {
    auto out = traj_in;

    if (!enable_ || !latest_goal_.has_value() || out.points.empty()) {
      pub_traj_->publish(out);
      return;
    }

    const auto & gp = latest_goal_->pose;
    const auto & last = out.points.back().pose.position;
    const double d = std::hypot(last.x - gp.position.x, last.y - gp.position.y);

    if (d > activation_distance_) {
      // 末点离 goal 太远（一般是中途轨迹），不动
      pub_traj_->publish(out);
      return;
    }

    if (strategy_ == "replace") {
      out.points.back().pose = gp;
      if (force_zero_velocity_at_goal_) {
        out.points.back().longitudinal_velocity_mps = 0.0f;
        out.points.back().acceleration_mps2 = 0.0f;
      }
    } else {  // "insert"
      autoware_planning_msgs::msg::TrajectoryPoint pt = out.points.back();
      pt.pose = gp;
      if (force_zero_velocity_at_goal_) {
        pt.longitudinal_velocity_mps = 0.0f;
        pt.acceleration_mps2 = 0.0f;
      }
      out.points.push_back(pt);
    }

    if (!log_once_) {
      RCLCPP_INFO(get_logger(),
                  "[ALIGN] 末点对齐生效: prev_d=%.3fm strategy=%s",
                  d, strategy_.c_str());
      log_once_ = true;
    }

    pub_traj_->publish(out);
  }

  bool enable_;
  double activation_distance_;
  std::string strategy_;
  bool force_zero_velocity_at_goal_;
  bool log_once_ = false;

  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_traj_;
};

}  // namespace goal_exact_point_appender

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<goal_exact_point_appender::GoalExactPointAppenderNode>());
  rclcpp::shutdown();
  return 0;
}
