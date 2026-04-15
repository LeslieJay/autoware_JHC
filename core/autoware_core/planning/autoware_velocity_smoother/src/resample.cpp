// Copyright 2021 Tier IV, Inc.
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

#include "autoware/velocity_smoother/resample.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <algorithm>
#include <optional>
#include <vector>

namespace autoware::velocity_smoother
{
namespace resampling
{
namespace detail
{
constexpr double stop_distance_overshoot_tolerance = 0.5;

std::optional<double> calcStopDistanceWithOvershootTolerance(
  const TrajectoryPoints & input, const geometry_msgs::msg::Pose & current_pose,
  const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  if (input.size() < 2) {
    return std::nullopt;
  }

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(input);
  if (!stop_idx) {
    return std::nullopt;
  }

  const auto current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);

  const double signed_stop_distance = autoware::motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, *stop_idx);

  const double euclidean_stop_distance =
    autoware_utils::calc_distance2d(current_pose.position, input.at(*stop_idx).pose.position);
  const double signed_euclidean_stop_distance =
    signed_stop_distance < 0.0 ? -euclidean_stop_distance : euclidean_stop_distance;

  if (signed_euclidean_stop_distance < -stop_distance_overshoot_tolerance) {
    return std::nullopt;
  }

  return std::max(0.0, signed_euclidean_stop_distance);
}
}  // namespace detail

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const double v_current,
  const geometry_msgs::msg::Pose & current_pose, const double nearest_dist_threshold,
  const double nearest_yaw_threshold, const ResampleParam & param, const bool use_zoh_for_v)
{
  // Arc length from the initial point to the closest point
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double negative_front_arclength_value = autoware::motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, input.at(0).pose.position, 0);
  const auto front_arclength_value = std::fabs(negative_front_arclength_value);

  const auto dist_to_closest_stop_point = detail::calcStopDistanceWithOvershootTolerance(
    input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);

  // Get the resample size from the closest point
  const double trajectory_length = autoware::motion_utils::calcArcLength(input);
  const double Nt = param.resample_time / std::max(param.dense_resample_dt, 0.001);
  const double ds_nominal =
    std::max(v_current * param.dense_resample_dt, param.dense_min_interval_distance);
  const double Ns = param.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  constexpr double front_ds = 0.1;
  for (double ds = 0.0; ds <= front_arclength_value; ds += front_ds) {
    out_arclength.push_back(ds);
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i = 0.0;
  bool is_zero_point_included = false;
  bool is_endpoint_included = false;
  for (size_t i = 1; static_cast<double>(i) <= N; ++i) {
    double ds = ds_nominal;
    if (i > Nt) {
      // if the planning time is not enough to see the desired distance,
      // change the interval distance to see far.
      ds = std::max(param.sparse_min_interval_distance, param.sparse_resample_dt * v_current);
    }

    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > param.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= trajectory_length) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (i > Nt && dist_i >= param.min_trajectory_length) {
      if (
        std::fabs(out_arclength.back() - (param.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = param.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(param.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // Handle Close Stop Point
    if (
      dist_to_closest_stop_point && dist_i > *dist_to_closest_stop_point &&
      !is_zero_point_included) {
      if (std::fabs(dist_i - *dist_to_closest_stop_point) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (*dist_to_closest_stop_point + front_arclength_value)) <
            1e-3) {
          out_arclength.back() = *dist_to_closest_stop_point + front_arclength_value;
        } else {
          out_arclength.push_back(*dist_to_closest_stop_point + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = *dist_to_closest_stop_point;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  if (input.size() < 2 || out_arclength.size() < 2 || trajectory_length < out_arclength.back()) {
    return input;
  }

  const auto output_traj = autoware::motion_utils::resampleTrajectory(
    autoware::motion_utils::convertToTrajectory(input), out_arclength, false, true, use_zoh_for_v);
  auto output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (autoware_utils::calc_distance2d(output.back(), input.back()) < ep_dist) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }

  return output;
}

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const geometry_msgs::msg::Pose & current_pose,
  const double nearest_dist_threshold, const double nearest_yaw_threshold,
  const ResampleParam & param, const double nominal_ds, const bool use_zoh_for_v)
{
  // input arclength
  const double trajectory_length = autoware::motion_utils::calcArcLength(input);
  const auto dist_to_closest_stop_point = detail::calcStopDistanceWithOvershootTolerance(
    input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);

  // 将可选的 stop distance 转换成下面重采样要使用的 stop 弧长。
  //
  // - 如果前方存在可用的 stop point，stop_arclength_value 就是 ego 到该 stop point 的距离。
  // - 如果当前没有可用的 stop point，则退化为普通的规划长度上限。
  //
  // 这个值是“distance to stop”影响最终发布轨迹的关键入口：
  // 一旦它突然变得很小，stop point 就会被几乎直接插到 ego 附近，留给减速尾巴的展开空间也会随之变少。
  double stop_arclength_value = param.max_trajectory_length;
  if (dist_to_closest_stop_point) {
    stop_arclength_value = std::min(*dist_to_closest_stop_point, stop_arclength_value);
  }
  if (param.min_trajectory_length < stop_arclength_value) {
    stop_arclength_value = param.min_trajectory_length;
  }

  // 在 stop point 前 3 m 构造一段密集采样区。
  //
  // 当 stop_arclength_value 正常时，终点前会有很多很密的采样点，优化器就能生成更平滑的减速尾巴。
  // 当 stop_arclength_value 突然逼近 0 时，这段密集采样区也会一起塌到 ego 附近，
  // 后处理重采样就没有足够空间去塑造一个渐进停车过程。
  const double start_stop_arclength_value = std::max(stop_arclength_value - 3.0, 0.0);

  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  // 计算从轨迹起点到 ego 当前投影位置的弧长。
  //
  // 输出重采样网格是基于“原始轨迹起点”的弧长坐标系来构建的。
  // front_arclength_value 表示 ego 相对轨迹起点的偏移，因此 Step2 里新增的每个采样点
  // 最后都要加上这个偏移量，才能正确映射回原始轨迹坐标系。
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double negative_front_arclength_value = autoware::motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, input.at(0).pose.position,
    static_cast<size_t>(0));
  const auto front_arclength_value = std::fabs(negative_front_arclength_value);
  for (double s = 0.0; s <= front_arclength_value; s += nominal_ds) {
    out_arclength.push_back(s);
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i{0.0};
  bool is_zero_point_included{false};
  bool is_endpoint_included{false};
  while (rclcpp::ok()) {
    double ds = nominal_ds;
    if (start_stop_arclength_value <= dist_i && dist_i <= stop_arclength_value) {
      // 在 stop point 前使用极密采样。
      // 这样终点减速过程会由很多个点来表达，而不是退化成“前面一段平台速度 + 最后一个点归零”。
      ds = 0.01;
    }
    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > param.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= trajectory_length) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (dist_i >= param.min_trajectory_length) {
      if (
        std::fabs(out_arclength.back() - (param.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = param.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(param.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // 当累计距离跨过 stop point 时，把 stop point 显式插入到重采样网格中。
    //
    // 这样输出轨迹里就一定会有一个点精确落在 stop 位置，后续步骤才能稳定保留终点的 0 速度点。
    //
    // 如果 stop_arclength_value 很小，这个插入动作就会几乎发生在 ego 附近。
    // 这也是为什么 distance-to-stop 一旦不稳定，终点的 0 速度点会突然被“拉近”到当前车位前方。
    if (dist_i > stop_arclength_value && !is_zero_point_included) {
      if (std::fabs(dist_i - stop_arclength_value) > 1e-3) {
        // 当前采样步长已经跨过 stop point，需要先把精确的 stop 弧长补进去，
        // 再继续后面的常规重采样。
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (stop_arclength_value + front_arclength_value)) < 1e-3) {
          out_arclength.back() = stop_arclength_value + front_arclength_value;
        } else {
          out_arclength.push_back(stop_arclength_value + front_arclength_value);
        }
      } else {
        // 当前采样点已经非常接近 stop point，直接把它视为 stop 的精确弧长。
        dist_i = stop_arclength_value;
      }

      is_zero_point_included = true;
    }

    // 添加 ego 前方的当前采样点。
    // 由于这里额外加上了 front_arclength_value，因此虽然逻辑上是在“从 ego 往前采样”，
    // 生成出来的弧长网格依然和原始轨迹的全局弧长坐标保持对齐。
    out_arclength.push_back(dist_i + front_arclength_value);
  }

  if (input.size() < 2 || out_arclength.size() < 2 || trajectory_length < out_arclength.back()) {
    return input;
  }

  const auto output_traj = autoware::motion_utils::resampleTrajectory(
    autoware::motion_utils::convertToTrajectory(input), out_arclength, false, true, use_zoh_for_v);
  auto output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);

  // 如果循环是因为到达原始轨迹终点而结束，就把原始终点显式补回去。
  // 这样最终轨迹末点可以保留原始终点的速度语义。
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (autoware_utils::calc_distance2d(output.back(), input.back()) < ep_dist) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }

  return output;
}

}  // namespace resampling
}  // namespace autoware::velocity_smoother
