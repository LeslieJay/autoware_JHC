// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_static_obstacle_avoidance_module/scene.hpp"

#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/debug.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>

#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
bool isDrivingSameLane(
  const lanelet::ConstLanelets & previous_lanes, const lanelet::ConstLanelets & current_lanes)
{
  std::multiset<lanelet::Id> prev_ids{};
  std::multiset<lanelet::Id> curr_ids{};
  std::multiset<lanelet::Id> same_ids{};

  std::for_each(
    previous_lanes.begin(), previous_lanes.end(), [&](const auto & l) { prev_ids.insert(l.id()); });
  std::for_each(
    current_lanes.begin(), current_lanes.end(), [&](const auto & l) { curr_ids.insert(l.id()); });

  std::set_intersection(
    prev_ids.begin(), prev_ids.end(), curr_ids.begin(), curr_ids.end(),
    std::inserter(same_ids, same_ids.end()));

  return !same_ids.empty();
}

bool isBestEffort(const std::string & policy)
{
  return policy == "best_effort";
}

lanelet::BasicLineString3d toLineString3d(const std::vector<Point> & bound)
{
  lanelet::BasicLineString3d ret{};
  std::for_each(
    bound.begin(), bound.end(), [&](const auto & p) { ret.emplace_back(p.x, p.y, p.z); });
  return ret;
}
}  // namespace

StaticObstacleAvoidanceModule::StaticObstacleAvoidanceModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  helper_{std::make_shared<AvoidanceHelper>(parameters)},
  parameters_{parameters},
  generator_{parameters}
{
}

/**
 * @brief 判断是否需要执行静态障碍物回避模块
 *
 * 该函数用于确定是否需要启动执行当前静态障碍物回避行为模块。主要通过以下逻辑判断：
 * 1. 如果存在需要停车的目标障碍物（stop_target_object），返回true，表示请求执行回避模块。
 * 2. 如果不存在新的偏移线（new_shift_line），说明当前无回避需求，返回false。
 * 3. 遍历当前目标障碍物（target_objects），只要存在可以回避的静态障碍物，则返回true。
 *
 * @return 如果需要执行静态障碍物回避模块则返回true，否则返回false。
 */
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{

  updateInfoMarker(avoid_data_);
  updateDebugMarker(BehaviorModuleOutput{}, avoid_data_, path_shifter_, debug_data_);

  // 如果有需要停车的目标障碍物，直接请求执行
  if (!!avoid_data_.stop_target_object) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 1000,
      "[DEBUG] isExecutionRequested: TRUE - Found stop_target_object (ID: %s)",
      to_hex_string(avoid_data_.stop_target_object->object.object_id).c_str());
    return true;
  }

  // 如果新的偏移线为空，不需要执行
  if (avoid_data_.new_shift_line.empty()) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 1000,
      "[DEBUG] isExecutionRequested: FALSE - new_shift_line is empty (target_objects count: %zu)",
      avoid_data_.target_objects.size());
    return false;
  }

  // 判断是否存在可以回避的目标障碍物
  const bool has_avoidable_object = std::any_of(
    avoid_data_.target_objects.begin(), avoid_data_.target_objects.end(),
    [this](const auto & o) { return !helper_->isAbsolutelyNotAvoidable(o); });

  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 1000,
    "[DEBUG] isExecutionRequested: %s - has_avoidable_object=%s, target_objects=%zu, "
    "new_shift_line=%zu",
    has_avoidable_object ? "TRUE" : "FALSE", has_avoidable_object ? "true" : "false",
    avoid_data_.target_objects.size(), avoid_data_.new_shift_line.size());

  return has_avoidable_object;
}

/**
 * @brief 判断当前是否准备好执行静态障碍物回避模块
 *
 * 本函数用于判断回避行为是否满足执行条件。判断依据如下：
 * 1. 路径是否安全（safe）
 * 2. 回避行为是否舒适（comfortable）
 * 3. 生成的回避路径是否合法（valid）
 * 4. 静态障碍物回避规划数据准备完毕（ready）
 *
 * 当上述四个条件均为true时，返回true，表示准备好执行回避模块，否则返回false。
 * 除此之外，该函数还会周期性输出调试信息和原因提示，便于开发和排查异常。
 *
 * @return 如果满足所有回避执行条件，返回true，否则返回false
 */
bool StaticObstacleAvoidanceModule::isExecutionReady() const
{
  RCLCPP_DEBUG_STREAM(getLogger(), "---Avoidance GO/NO-GO status---");
  RCLCPP_DEBUG_STREAM(getLogger(), std::boolalpha << "SAFE:" << avoid_data_.safe);
  RCLCPP_DEBUG_STREAM(getLogger(), std::boolalpha << "COMFORTABLE:" << avoid_data_.comfortable);
  RCLCPP_DEBUG_STREAM(getLogger(), std::boolalpha << "VALID:" << avoid_data_.valid);
  RCLCPP_DEBUG_STREAM(getLogger(), std::boolalpha << "READY:" << avoid_data_.ready);

  const bool is_ready = avoid_data_.safe && avoid_data_.comfortable && avoid_data_.valid && avoid_data_.ready;
  // 添加打印输出进入了函数
  // RCLCPP_INFO_THROTTLE(getLogger(), *clock_, 1000, "[DEBUG] Entered isExecutionReady function");

  if (!is_ready) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] isExecutionReady: FALSE - Cannot execute avoidance. "
      "Reasons: SAFE=%s, COMFORTABLE=%s, VALID=%s, READY=%s",
      avoid_data_.safe ? "OK" : "FAIL", avoid_data_.comfortable ? "OK" : "FAIL",
      avoid_data_.valid ? "OK" : "FAIL", avoid_data_.ready ? "OK" : "FAIL");
  }

  return is_ready;
}

AvoidanceState StaticObstacleAvoidanceModule::getCurrentModuleState(
  const AvoidancePlanningData & data) const
{
  const bool has_avoidance_target = std::any_of(
    data.target_objects.begin(), data.target_objects.end(),
    [this](const auto & o) { return !helper_->isAbsolutelyNotAvoidable(o); });

  if (has_avoidance_target) {
    return AvoidanceState::RUNNING;
  }

  // If the ego is on the shift line, keep RUNNING.
  {
    const size_t idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
    const auto within = [](const auto & line, const size_t idx) {
      return line.start_idx < idx && idx < line.end_idx;
    };
    for (const auto & shift_line : path_shifter_.getShiftLines()) {
      if (within(shift_line, idx)) {
        return AvoidanceState::RUNNING;
      }
    }
  }

  const bool has_shift_point = !path_shifter_.getShiftLines().empty();
  const bool has_base_offset =
    std::abs(path_shifter_.getBaseOffset()) > parameters_->lateral_execution_threshold;

  if (has_base_offset) {
    return AvoidanceState::RUNNING;
  }

  // Nothing to do. -> EXIT.
  if (!has_shift_point) {
    return AvoidanceState::SUCCEEDED;
  }

  // Be able to canceling avoidance path. -> EXIT.
  if (!helper_->isShifted() && parameters_->enable_cancel_maneuver) {
    return AvoidanceState::CANCEL;
  }

  return AvoidanceState::RUNNING;
}

bool StaticObstacleAvoidanceModule::canTransitSuccessState()
{
  const auto & data = avoid_data_;

  // Change input lane. -> EXIT.
  if (!isDrivingSameLane(helper_->getPreviousDrivingLanes(), data.current_lanelets)) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 3000, "Previous module lane is updated. Exit.");
    return true;
  }

  helper_->setPreviousDrivingLanes(data.current_lanelets);

  // Reach input path end point. -> EXIT.
  {
    const auto idx = planner_data_->findEgoIndex(data.reference_path.points);
    if (idx == data.reference_path.points.size() - 1) {
      arrived_path_end_ = true;
    }

    constexpr double THRESHOLD = 1.0;
    const auto is_further_than_threshold =
      calc_distance2d(getEgoPose(), autoware_utils::get_pose(data.reference_path.points.back())) >
      THRESHOLD;
    if (is_further_than_threshold && arrived_path_end_) {
      RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 3000, "Reach path end point. Exit.");
      return true;
    }
  }

  return data.state == AvoidanceState::CANCEL || data.state == AvoidanceState::SUCCEEDED;
}

void StaticObstacleAvoidanceModule::fillFundamentalData(
  AvoidancePlanningData & data, DebugData & debug)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (getPreviousModuleOutput().reference_path.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Previous module reference path is empty. Skip processing.");
    return;
  }

  // reference pose
  data.reference_pose =
    utils::getUnshiftedEgoPose(getEgoPose(), helper_->getPreviousSplineShiftPath());

  // lanelet info
  data.current_lanelets = utils::static_obstacle_avoidance::getCurrentLanesFromPath(
    getPreviousModuleOutput().reference_path, planner_data_);

  if (data.current_lanelets.empty()) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "Current lanelets is empty. Skip processing.");
    return;
  }

  data.extend_lanelets = utils::static_obstacle_avoidance::getExtendLanes(
    data.current_lanelets, getEgoPose(), planner_data_);

  lanelet::ConstLanelet closest_lanelet{};
  if (lanelet::utils::query::getClosestLanelet(
        data.current_lanelets, getEgoPose(), &closest_lanelet))
    data.closest_lanelet = closest_lanelet;

  // expand drivable lanes
  const auto is_within_current_lane =
    utils::static_obstacle_avoidance::isWithinLanes(data.closest_lanelet, planner_data_);
  const auto red_signal_lane_itr = std::find_if(
    data.current_lanelets.begin(), data.current_lanelets.end(), [&](const auto & lanelet) {
      if (utils::traffic_light::isTrafficSignalStop({lanelet}, planner_data_)) {
        return true;
      }
      const auto next_lanes = planner_data_->route_handler->getNextLanelets(lanelet);
      return utils::traffic_light::isTrafficSignalStop(next_lanes, planner_data_);
    });
  const auto not_use_adjacent_lane =
    is_within_current_lane && red_signal_lane_itr != data.current_lanelets.end();

  std::for_each(
    data.current_lanelets.begin(), data.current_lanelets.end(), [&](const auto & lanelet) {
      if (!not_use_adjacent_lane || red_signal_lane_itr->id() != lanelet.id()) {
        data.drivable_lanes.push_back(
          utils::static_obstacle_avoidance::generateExpandedDrivableLanes(
            lanelet, planner_data_, parameters_));
      } else {
        data.drivable_lanes.push_back(
          utils::static_obstacle_avoidance::generateNotExpandedDrivableLanes(lanelet));
        data.red_signal_lane = lanelet;
      }
    });

  // calc drivable bound
  auto tmp_path = getPreviousModuleOutput().path;
  const auto shorten_lanes = utils::cutOverlappedLanes(tmp_path, data.drivable_lanes);
  const auto use_left_side_hatched_road_marking_area = [&]() {
    if (!not_use_adjacent_lane) {
      return true;
    }
    return !planner_data_->route_handler->getRoutingGraphPtr()->left(*red_signal_lane_itr);
  }();
  const auto use_right_side_hatched_road_marking_area = [&]() {
    if (!not_use_adjacent_lane) {
      return true;
    }
    return !planner_data_->route_handler->getRoutingGraphPtr()->right(*red_signal_lane_itr);
  }();
  data.left_bound = utils::calcBound(
    getPreviousModuleOutput().path, planner_data_, shorten_lanes,
    use_left_side_hatched_road_marking_area, parameters_->use_intersection_areas,
    parameters_->use_freespace_areas, true);
  data.right_bound = utils::calcBound(
    getPreviousModuleOutput().path, planner_data_, shorten_lanes,
    use_right_side_hatched_road_marking_area, parameters_->use_intersection_areas,
    parameters_->use_freespace_areas, false);

  // reference path
  if (isDrivingSameLane(helper_->getPreviousDrivingLanes(), data.current_lanelets)) {
    data.reference_path_rough = extendBackwardLength(getPreviousModuleOutput().path);
  } else {
    data.reference_path_rough = getPreviousModuleOutput().path;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000,
      "Previous module lane is updated. Don't use latest reference path.");
  }

  // resampled reference path
  data.reference_path = utils::resamplePathWithSpline(
    data.reference_path_rough, parameters_->resample_interval_for_planning);

  // closest index
  data.ego_closest_path_index = planner_data_->findEgoIndex(data.reference_path.points);

  // arclength from ego pose (used in many functions)
  data.arclength_from_ego = utils::calcPathArcLengthArray(
    data.reference_path, 0, data.reference_path.points.size(),
    autoware::motion_utils::calcSignedArcLength(data.reference_path.points, getEgoPosition(), 0));

  data.is_allowed_goal_modification =
    utils::isAllowedGoalModification(planner_data_->route_handler);
  data.distance_to_red_traffic_light = utils::traffic_light::calcDistanceToRedTrafficLight(
    data.current_lanelets, data.reference_path_rough, planner_data_);

  data.to_return_point = utils::static_obstacle_avoidance::calcDistanceToReturnDeadLine(
    data.current_lanelets, data.reference_path_rough, planner_data_, parameters_,
    data.distance_to_red_traffic_light, data.is_allowed_goal_modification);

  data.to_start_point = utils::static_obstacle_avoidance::calcDistanceToAvoidStartLine(
    data.current_lanelets, parameters_, data.distance_to_red_traffic_light);

  // filter only for the latest detected objects.
  fillAvoidanceTargetObjects(data, debug);

  // compensate lost object which was avoidance target. if the time hasn't passed more than
  // threshold since perception module lost the target yet, this module keeps it as avoidance
  // target.
  utils::static_obstacle_avoidance::compensateLostTargetObjects(
    registered_objects_, data, clock_->now(), planner_data_, parameters_);

  // once an object filtered for boundary clipping, this module keeps the information until the end
  // of execution.
  utils::static_obstacle_avoidance::updateClipObject(clip_objects_, data);

  // calculate various data for each target objects.
  fillAvoidanceTargetData(data.target_objects);

  // sort object order by longitudinal distance
  std::sort(data.target_objects.begin(), data.target_objects.end(), [](auto a, auto b) {
    return a.longitudinal < b.longitudinal;
  });

  // set base path
  path_shifter_.setPath(data.reference_path);
}

void StaticObstacleAvoidanceModule::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  using utils::static_obstacle_avoidance::filterTargetObjects;
  using utils::static_obstacle_avoidance::separateObjectsByPath;
  using utils::static_obstacle_avoidance::updateRoadShoulderDistance;

  // Separate dynamic objects based on whether they are inside or outside of the expanded lanelets.
  constexpr double MARGIN = 10.0;
  const auto forward_detection_range = [&]() {
    if (!data.distance_to_red_traffic_light.has_value()) {
      return helper_->getForwardDetectionRange(data.closest_lanelet);
    }
    return std::min(
      helper_->getForwardDetectionRange(data.closest_lanelet),
      data.distance_to_red_traffic_light.value());
  }();

  const auto [object_within_target_lane, object_outside_target_lane] = separateObjectsByPath(
    helper_->getPreviousReferencePath(), helper_->getPreviousSplineShiftPath().path, planner_data_,
    data, parameters_, forward_detection_range + MARGIN, debug);

  for (const auto & object : object_outside_target_lane.objects) {
    ObjectData other_object = createObjectData(data, object);
    other_object.info = ObjectInfo::OUT_OF_TARGET_AREA;
    data.other_objects.push_back(other_object);
  }

  ObjectDataArray objects;
  for (const auto & object : object_within_target_lane.objects) {
    objects.push_back(createObjectData(data, object));
  }

  // Filter out the objects to determine the ones to be avoided.
  filterTargetObjects(objects, data, forward_detection_range, planner_data_, parameters_);
  updateRoadShoulderDistance(data, planner_data_, parameters_);

  // debug
  {
    std::vector<AvoidanceDebugMsg> debug_info_array;
    const auto append = [&](const auto & o) {
      AvoidanceDebugMsg debug_info;
      debug_info.object_id = to_hex_string(o.object.object_id);
      debug_info.longitudinal_distance = o.longitudinal;
      debug_info.lateral_distance_from_centerline = o.to_centerline;
      debug_info_array.push_back(debug_info);
    };

    std::for_each(objects.begin(), objects.end(), [&](const auto & o) { append(o); });

    updateAvoidanceDebugData(debug_info_array);
  }

  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 1000,
    "[fillAvoidanceTargetObjects] perception_objects=%zu → target_objects=%zu, "
    "other_objects=%zu",
    objects.size(), data.target_objects.size(), data.other_objects.size());
  for (const auto & o : data.target_objects) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 1000,
      "  [TARGET] id=%s longitudinal=%.1f avoid_required=%s avoid_margin=%.2f "
      "is_parked=%s is_on_ego_lane=%s",
      to_hex_string(o.object.object_id).substr(0, 8).c_str(),
      o.longitudinal, o.avoid_required ? "TRUE" : "FALSE",
      o.avoid_margin.has_value() ? o.avoid_margin.value() : -1.0,
      o.is_parked ? "true" : "false", o.is_on_ego_lane ? "true" : "false");
  }
}

void StaticObstacleAvoidanceModule::fillAvoidanceTargetData(ObjectDataArray & objects) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  using utils::static_obstacle_avoidance::fillAvoidanceNecessity;
  using utils::static_obstacle_avoidance::fillObjectStoppableJudge;

  // Calculate the distance needed to safely decelerate the ego vehicle to a stop line.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  const auto feasible_stop_distance = helper_->getFeasibleDecelDistance(0.0, false);
  std::for_each(objects.begin(), objects.end(), [&, this](auto & o) {
    fillAvoidanceNecessity(o, registered_objects_, vehicle_width, parameters_);
    o.to_stop_line = calcDistanceToStopLine(o);
    fillObjectStoppableJudge(o, registered_objects_, feasible_stop_distance, parameters_);
  });
}

ObjectData StaticObstacleAvoidanceModule::createObjectData(
  const AvoidancePlanningData & data, const PredictedObject & object) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  using boost::geometry::return_centroid;

  const auto & path_points = data.reference_path.points;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index =
    autoware::motion_utils::findNearestIndex(path_points, object_pose.position);
  const auto object_closest_pose = path_points.at(object_closest_index).point.pose;
  const auto object_type = utils::getHighestProbLabel(object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);

  ObjectData object_data{};

  object_data.object = object;

  const auto lower = parameters_->lower_distance_for_polygon_expansion;
  const auto upper = parameters_->upper_distance_for_polygon_expansion;
  const auto clamp =
    std::clamp(calc_distance2d(getEgoPose(), object_pose) - lower, 0.0, upper) / upper;
  object_data.distance_factor = object_parameter.max_expand_ratio * clamp + 1.0;

  // Calc envelop polygon.
  utils::static_obstacle_avoidance::fillObjectEnvelopePolygon(
    object_data, registered_objects_, object_closest_pose, parameters_);

  // calc object centroid.
  object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

  // Calc moving time.
  utils::static_obstacle_avoidance::fillObjectMovingTime(
    object_data, stopped_objects_, parameters_);

  // Calc lateral deviation from path to target object.
  object_data.direction = calc_lateral_deviation(object_closest_pose, object_pose.position) > 0.0
                            ? Direction::LEFT
                            : Direction::RIGHT;

  return object_data;
}

bool StaticObstacleAvoidanceModule::canYieldManeuver(const AvoidancePlanningData & data) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  // transit yield maneuver only when the avoidance maneuver is not initiated.
  if (helper_->isShifted()) {
    // RCLCPP_INFO_THROTTLE(
    //   getLogger(), *clock_, 1000,
    //   "[DEBUG] canYieldManeuver: FALSE - Avoidance maneuver already initiated (isShifted=true)");
    return false;
  }

  // prevent sudden steering.
  const auto registered_lines = path_shifter_.getShiftLines();
  if (!registered_lines.empty()) {
    const size_t idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
    const auto prepare_distance = autoware::motion_utils::calcSignedArcLength(
      path_shifter_.getReferencePath().points, idx, registered_lines.front().start_idx);
    if (!helper_->isEnoughPrepareDistance(prepare_distance)) {
      // RCLCPP_INFO_THROTTLE(
      //   getLogger(), *clock_, 1000,
      //   "[DEBUG] canYieldManeuver: FALSE - Insufficient prepare distance (prepare_distance=%.2f)",
      //   prepare_distance);
      return false;
    }
  }

  if (!data.stop_target_object) {
    // RCLCPP_INFO_THROTTLE(
    //   getLogger(), *clock_, 1000,
    //   "[DEBUG] canYieldManeuver: TRUE - Can pass by object safely without avoidance maneuver");
    return true;
  }

  constexpr double TH_STOP_SPEED = 0.01;
  constexpr double TH_STOP_POSITION = 0.5;

  const auto stopped_for_the_object =
    getEgoSpeed() < TH_STOP_SPEED && std::abs(data.to_stop_line) < TH_STOP_POSITION;

  const auto id = data.stop_target_object.value().object.object_id;
  const auto same_id_obj = std::find_if(
    ego_stopped_objects_.begin(), ego_stopped_objects_.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  // if the ego already started avoiding for the object, never transit yield maneuver again.
  if (same_id_obj != ego_stopped_objects_.end()) {
    return stopped_for_the_object;
  }

  // registered objects whom the ego stopped for at the moment of stopping.
  if (stopped_for_the_object) {
    ego_stopped_objects_.push_back(data.stop_target_object.value());
  }

  return true;
}

/**
 * @brief 填充并处理静态障碍物避让的偏移线（shift line）
 *
 * 此函数负责按照步骤生成、验证并设置避让的shift lines，并据此生成避让路径，最终对路径舒适性、安全性、可用性及准备状态进行检查和标记。
 * 
 * 具体步骤如下：
 * 1. 生成候选shift lines（偏移线），合并粗略shift lines并提取新的shift lines。
 * 2. 验证新生成的shift lines，有效条件为生成的避让路径不与ego呈现巨大偏移。
 * 3. 如果存在新shift lines，则添加到path_shifter以生成候选避让路径。
 * 4. 基于shift lines生成避让路径。
 * 5. 检查新生成的避让路径是否舒适、安全、可用且已经准备好执行，并打印调试信息。
 *
 * @param [in,out] data  避让规划数据，内部字段会被填充和更新。
 * @param [in,out] debug 调试信息相关数据。
 */
void StaticObstacleAvoidanceModule::fillShiftLine(
  AvoidancePlanningData & data, DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  auto path_shifter = path_shifter_;

  // STEP1: 生成候选shift lines，合并粗略线并提取新线
  data.new_shift_line = generator_.generate(data, debug);

  // STEP2: 验证新shift lines是否合法
  data.valid = isValidShiftLine(data.new_shift_line, path_shifter);
  const auto found_new_sl = data.new_shift_line.size() > 0;
  const auto registered = path_shifter.getShiftLines().size() > 0;
  data.found_avoidance_path = found_new_sl || registered;

  // STEP3: 如果有新shift lines则注册到path_shifter
  if (!data.new_shift_line.empty()) {
    addNewShiftLines(path_shifter, data.new_shift_line);
  }

  // STEP4: 生成避让路径
  ShiftedPath spline_shift_path =
    utils::static_obstacle_avoidance::toShiftedPath(data.reference_path);
  const auto success_spline_path_generation =
    path_shifter.generate(&spline_shift_path, true, SHIFT_TYPE::SPLINE);
  data.candidate_path = success_spline_path_generation
                          ? spline_shift_path
                          : utils::static_obstacle_avoidance::toShiftedPath(data.reference_path);

  // STEP5: 检查路径的舒适性、安全性、可用性和准备状态，并打印调试信息
  data.comfortable = helper_->isComfortable(data.new_shift_line);
  data.safe = isSafePath(data.candidate_path, debug);
  // const auto avoidance_ready = helper_->isReady(data.target_objects);
  // data.ready = helper_->isReady(data.new_shift_line, path_shifter_.getLastShiftLength()) &&
  //              avoidance_ready.first;
  // data.request_operator = avoidance_ready.second;
  data.ready = true;
  data.request_operator = false;
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 1000,
    "[fillShiftLine] new_shift_line=%zu, valid=%s, comfortable=%s, safe=%s, ready=%s, "
    "request_operator=%s, target_objects=%zu",
    data.new_shift_line.size(), data.valid ? "true" : "false",
    data.comfortable ? "true" : "false", data.safe ? "true" : "false",
    data.ready ? "true" : "false", data.request_operator ? "true" : "false",
    data.target_objects.size());

  if (!data.safe) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 2000,
      "[fillShiftLine] Path is NOT SAFE - This may cause vehicle to stop instead of avoid");
  }
  if (!data.comfortable) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 2000,
      "[fillShiftLine] Path is NOT COMFORTABLE - Shift may be too aggressive");
  }
  if (!data.valid) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 2000,
      "[fillShiftLine] Path is NOT VALID - Shift line validation failed");
  }
  if (data.new_shift_line.empty()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 2000,
      "[fillShiftLine] new_shift_line is EMPTY - No shift lines generated for target objects");
  }
}

/**
 * @brief 根据当前避让规划数据，填充自车状态变量，决定本周期的避障/让行/停车等高层状态流转。
 *
 * 该函数主要实现如下逻辑：
 * 1. 检查所有目标障碍物，若检测到必须避让的对象，设置avoid_required、stop_target_object等相关输出。
 * 2. 若外部锁定了路径输出，则保持上一周期轨迹、不做更新。
 * 3. 判断是否存在任何移位线被“强制失效”并处于可让行状态，此时尝试进入让行流程。
 * 4. 若路径被判定为安全，则可直接执行规避行为（yield_required=false）。
 * 5. 若禁用让行策略，即使规避路径不安全也照常规避。
 * 6. 若当前不可让行（如规避已中途开始、准备距离不足），将路径强制判为安全并执行规避。
 * 7. 若候选/已注册的移位线被“强制激活”，则直接执行规避。
 * 8. 默认流程为进入让行（yield），清除已注册移位线，并准备停车。
 * 9. 最后，若已经存在已批准的移位线但当前路径不安全，则发出警告并取消已批准路径。
 * 10. 最极端情况下，车辆将在危险障碍物前停车，不再尝试避让。
 *
 * @param[in,out] data  避让路径及决策相关规划数据，会被填充更新
 * @param[in,out] debug 调试数据（未使用）
 */
void StaticObstacleAvoidanceModule::fillEgoStatus(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  data.state = getCurrentModuleState(data);

  /**
   * Find the nearest object that should be avoid. When the ego follows reference path,
   * if the both of following two conditions are satisfied, the module surely avoid the object.
   * Condition1: there is risk to collide with object without avoidance.
   * Condition2: there is enough space to avoid.
   * In NEED_DECELERATION condition, it is possible to avoid object by deceleration even if the flag
   * is_avoidable is FALSE. So, the module inserts stop point for such a object.
   */
  for (const auto & o : data.target_objects) {
    if (o.avoid_required && !helper_->isAbsolutelyNotAvoidable(o)) {
      data.avoid_required = true;
      data.stop_target_object = o;
      data.to_stop_line = o.to_stop_line;
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000,
        "[fillEgoStatus] Found avoid_required object id=%s to_stop_line=%.2f "
        "is_avoidable=%s is_parked=%s longitudinal=%.1f",
        to_hex_string(o.object.object_id).substr(0, 8).c_str(), o.to_stop_line,
        o.is_avoidable ? "true" : "false", o.is_parked ? "true" : "false", o.longitudinal);
      break;
    }
  }
  if (!data.avoid_required) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 2000,
      "[fillEgoStatus] No avoid_required object in %zu target_objects - vehicle does NOT need to avoid",
      data.target_objects.size());
  }

  const auto can_yield_maneuver = canYieldManeuver(data);

  /**
   * If the output path is locked by outside of this module, don't update output path.
   */
  if (isOutputPathLocked()) {
    data.safe_shift_line.clear();
    data.candidate_path = helper_->getPreviousSplineShiftPath();
    // RCLCPP_INFO_THROTTLE(
    //   getLogger(), *clock_, 500, "this module is locked now. keep current path.");
    return;
  }

  const auto registered_sl_force_deactivated =
    [&](const std::string & direction, const RegisteredShiftLineArray & shift_line_array) {
      return std::any_of(
        shift_line_array.begin(), shift_line_array.end(), [&](const auto & shift_line) {
          return rtc_interface_ptr_map_.at(direction)->isForceDeactivated(shift_line.uuid);
        });
    };

  const auto is_force_deactivated = registered_sl_force_deactivated("left", left_shift_array_) ||
                                    registered_sl_force_deactivated("right", right_shift_array_);
  if (is_force_deactivated && can_yield_maneuver) {
    data.yield_required = true;
    data.safe_shift_line = data.new_shift_line;
    data.force_deactivated = true;
    // RCLCPP_INFO_THROTTLE(
    //   getLogger(), *clock_, 3000, "this module is force deactivated. wait until reactivation");
    return;
  }

  /**
   * If the avoidance path is safe, use unapproved_new_sl for avoidance path generation.
   */
  if (data.safe) {
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 1000,
      "[fillEgoStatus] Path is SAFE - Will execute avoidance (yield_required=false, "
      "new_shift_line=%zu)",
      data.new_shift_line.size());
    return;
  }

  /**
   * If the yield maneuver is disabled, use unapproved_new_sl for avoidance path generation even if
   * the shift line is unsafe.
   */
  if (!parameters_->enable_yield_maneuver) {
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 2000,
      "[fillEgoStatus] Yield maneuver is DISABLED - Will attempt avoidance even if unsafe "
      "(enable_yield_maneuver=false)");
    return;
  }

  /**
   * TODO(Satoshi OTA) Think yield maneuver in the middle of avoidance.
   * Even if it is determined that a yield is necessary, the yield maneuver is not executed
   * if the avoidance has already been initiated.
   */
  if (!can_yield_maneuver) {
    data.safe = true;  // overwrite safety judge.
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 500,
      "[fillEgoStatus] Cannot transit to yield - path unsafe but cannot yield. "
      "Forcing safe=true (avoidance already initiated or insufficient prepare distance)");
    return;
  }

  auto candidate_sl_force_activated = [&](const std::string & direction) {
    // If statement to avoid unnecessary warning occurring from isForceActivated function
    if (candidate_uuid_ == uuid_map_.at(direction)) {
      if (rtc_interface_ptr_map_.at(direction)->isForceActivated(candidate_uuid_)) {
        return true;
      }
    }
    return false;
  };

  auto registered_sl_force_activated =
    [&](const std::string & direction, const RegisteredShiftLineArray & shift_line_array) {
      return std::any_of(
        shift_line_array.begin(), shift_line_array.end(), [&](const auto & shift_line) {
          return rtc_interface_ptr_map_.at(direction)->isForceActivated(shift_line.uuid);
        });
    };

  /**
   * Check if the candidate avoidance path is force activated
   */
  if (candidate_sl_force_activated("left") || candidate_sl_force_activated("right")) {
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
    return;
  }

  /**
   * Check if any registered shift line is force activated
   */
  if (
    registered_sl_force_activated("left", left_shift_array_) ||
    registered_sl_force_activated("right", right_shift_array_)) {
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "unsafe but force executed");
    return;
  }

  /**
   * Transit yield maneuver. Clear shift lines and output yield path.
   */
  {
    data.yield_required = true;
    data.safe_shift_line = data.new_shift_line;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[fillEgoStatus] YIELD MANEUVER triggered - vehicle will STOP instead of avoid. "
      "safe=%s avoid_required=%s found_avoidance_path=%s stop_target=%s new_shift_line=%zu",
      data.safe ? "true" : "false", data.avoid_required ? "true" : "false",
      data.found_avoidance_path ? "true" : "false",
      data.stop_target_object.has_value() ? "exists" : "none",
      data.new_shift_line.size());
  }

  /**
   * Even if data.avoid_required is false, the module cancels registered shift point when the
   * approved avoidance path is not safe.
   */
  const auto approved_path_exist = !path_shifter_.getShiftLines().empty();
  if (approved_path_exist) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000,
      "[DEBUG] fillEgoStatus: Unsafe path detected - Canceling approved path (approved_path_exist=true)");
    return;
  }

  /**
   * If the avoidance path is not safe in situation where the ego should avoid object, the ego
   * stops in front of the front object with the necessary distance to avoid the object.
   */
  // {
  //   RCLCPP_WARN_THROTTLE(
  //     getLogger(), *clock_, 5000,
  //     "[DEBUG] fillEgoStatus: Transit to YIELD MANEUVER - Path is unsafe, vehicle will STOP instead of AVOID. "
  //     "Reason: safe=%s, avoid_required=%s, found_avoidance_path=%s, stop_target_object=%s",
  //     data.safe ? "true" : "false", data.avoid_required ? "true" : "false",
  //     data.found_avoidance_path ? "true" : "false",
  //     data.stop_target_object.has_value() ? "exists" : "none");
  // }
}

void StaticObstacleAvoidanceModule::fillDebugData(
  const AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!data.stop_target_object) {
    return;
  }

  if (helper_->isShifted()) {
    return;
  }

  if (data.new_shift_line.empty()) {
    return;
  }

  const auto o_front = data.stop_target_object.value();
  const auto object_type = utils::getHighestProbLabel(o_front.object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);
  const auto constant_distance = helper_->getFrontConstantDistance(o_front);
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;

  const auto lateral_hard_margin = o_front.is_parked
                                     ? object_parameter.lateral_hard_margin_for_parked_vehicle
                                     : object_parameter.lateral_hard_margin;
  const auto max_avoid_margin = lateral_hard_margin * o_front.distance_factor +
                                object_parameter.lateral_soft_margin + 0.5 * vehicle_width;

  const auto avoidance_distance = helper_->getSharpAvoidanceDistance(helper_->getShiftLength(
    o_front, utils::static_obstacle_avoidance::isOnRight(o_front), max_avoid_margin));
  const auto prepare_distance = helper_->getNominalPrepareDistance();
  const auto total_avoid_distance = prepare_distance + avoidance_distance + constant_distance;

  const auto dead_pose_opt = autoware::motion_utils::calcLongitudinalOffsetPose(
    data.reference_path.points, getEgoPosition(), o_front.longitudinal - total_avoid_distance);
  dead_pose_ = dead_pose_opt.has_value()
                 ? PoseWithDetail(dead_pose_opt.value())
                 : PoseWithDetail(autoware_utils::get_pose(data.reference_path.points.front()));
}

void StaticObstacleAvoidanceModule::updateEgoBehavior(
  const AvoidancePlanningData & data, ShiftedPath & path)
{
  if (parameters_->path_generation_method == "optimization_base") {
    return;
  }

  insertPrepareVelocity(path);
  insertAvoidanceVelocity(path);

  const auto insert_velocity = [this, &data, &path]() {
    if (data.yield_required) {
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 3000,
        "[DEBUG] updateEgoBehavior: Inserting WAIT POINT (yield_required=true) - Vehicle will STOP");
      insertWaitPoint(isBestEffort(parameters_->policy_deceleration), path);
      return;
    }

    if (!data.avoid_required) {
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 3000,
        "[DEBUG] updateEgoBehavior: No avoid_required - No velocity modification needed");
      return;
    }

    if (!data.found_avoidance_path) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 3000,
        "[DEBUG] updateEgoBehavior: Inserting WAIT POINT - No avoidance path found (found_avoidance_path=false)");
      insertWaitPoint(isBestEffort(parameters_->policy_deceleration), path);
      return;
    }

    if (isWaitingApproval() && path_shifter_.getShiftLines().empty()) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 3000,
        "[DEBUG] updateEgoBehavior: Inserting WAIT POINT - Waiting approval and shift_lines empty");
      insertWaitPoint(isBestEffort(parameters_->policy_deceleration), path);
      return;
    }

    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] updateEgoBehavior: Inserting STOP POINT - Path is unsafe during shifting");
    insertStopPoint(isBestEffort(parameters_->policy_deceleration), path);
  };

  insert_velocity();

  insertReturnDeadLine(isBestEffort(parameters_->policy_deceleration), path);

  set_longitudinal_planning_factor(path.path);
}

bool StaticObstacleAvoidanceModule::isSafePath(
  ShiftedPath & shifted_path, [[maybe_unused]] DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & p = planner_data_->parameters;

  if (force_deactivated_) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] isSafePath: FALSE - Module is force_deactivated");
    return false;
  }

  if (!parameters_->enable_safety_check) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] isSafePath: TRUE - Safety check is disabled (enable_safety_check=false)");
    return true;  // if safety check is disabled, it always return safe.
  }

  const auto ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);

  const auto has_left_shift = [&]() {
    for (size_t i = ego_idx; i < shifted_path.shift_length.size(); i++) {
      const auto length = shifted_path.shift_length.at(i);

      if (parameters_->lateral_execution_threshold < length) {
        return true;
      }
    }

    return false;
  }();

  const auto has_right_shift = [&]() {
    for (size_t i = ego_idx; i < shifted_path.shift_length.size(); i++) {
      const auto length = shifted_path.shift_length.at(i);

      if (parameters_->lateral_execution_threshold < -1.0 * length) {
        return true;
      }
    }

    return false;
  }();

  if (!has_left_shift && !has_right_shift) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] isSafePath: TRUE - No lateral shift detected (has_left_shift=false, has_right_shift=false)");
    return true;
  }

  const auto hysteresis_factor = safe_ ? 1.0 : parameters_->hysteresis_factor_expand_rate;

  const auto safety_check_target_objects =
    utils::static_obstacle_avoidance::getSafetyCheckTargetObjects(
      avoid_data_, planner_data_, parameters_, has_left_shift, has_right_shift, debug);

  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 3000,
    "[DEBUG] isSafePath: Checking safety - has_left_shift=%s, has_right_shift=%s, "
    "safety_check_target_objects=%zu, hysteresis_factor=%.2f",
    has_left_shift ? "true" : "false", has_right_shift ? "true" : "false",
    safety_check_target_objects.size(), hysteresis_factor);

  if (safety_check_target_objects.empty()) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] isSafePath: TRUE - No safety check target objects");
    return true;
  }

  const bool limit_to_max_velocity = false;
  const auto ego_predicted_path_params =
    std::make_shared<utils::path_safety_checker::EgoPredictedPathParams>(
      parameters_->ego_predicted_path_params);
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(shifted_path.path.points);
  const auto ego_predicted_path_for_front_object = utils::path_safety_checker::createPredictedPath(
    ego_predicted_path_params, shifted_path.path.points, getEgoPose(), getEgoSpeed(), ego_seg_idx,
    true, limit_to_max_velocity);
  const auto ego_predicted_path_for_rear_object = utils::path_safety_checker::createPredictedPath(
    ego_predicted_path_params, shifted_path.path.points, getEgoPose(), getEgoSpeed(), ego_seg_idx,
    false, limit_to_max_velocity);

  for (const auto & object : safety_check_target_objects) {
    auto current_debug_data = utils::path_safety_checker::createObjectDebug(object);

    const auto obj_polygon = autoware_utils::to_polygon2d(object.initial_pose, object.shape);

    const auto is_object_front = utils::path_safety_checker::isTargetObjectFront(
      getEgoPose(), obj_polygon, p.vehicle_info.max_longitudinal_offset_m);

    const auto & object_twist = object.initial_twist;
    const auto v_norm = std::hypot(object_twist.linear.x, object_twist.linear.y);
    const auto object_type = object.classification;
    const auto object_parameter = parameters_->object_parameters.at(object_type.label);
    const auto is_object_moving = v_norm > object_parameter.moving_speed_threshold;

    const auto is_object_oncoming =
      is_object_moving &&
      utils::path_safety_checker::isTargetObjectOncoming(getEgoPose(), object.initial_pose);

    const auto obj_predicted_paths = utils::path_safety_checker::getPredictedPathFromObj(
      object, parameters_->check_all_predicted_path);

    const auto & ego_predicted_path = is_object_front && !is_object_oncoming
                                        ? ego_predicted_path_for_front_object
                                        : ego_predicted_path_for_rear_object;

    for (const auto & obj_path : obj_predicted_paths) {
      if (!utils::path_safety_checker::checkCollision(
            shifted_path.path, ego_predicted_path, object, obj_path, p, parameters_->rss_params,
            hysteresis_factor, parameters_->collision_check_yaw_diff_threshold,
            current_debug_data.second)) {
        utils::path_safety_checker::updateCollisionCheckDebugMap(
          debug.collision_check, current_debug_data, false);

        RCLCPP_WARN_THROTTLE(
          getLogger(), *clock_, 3000,
          "[DEBUG] isSafePath: FALSE - COLLISION DETECTED! Object ID: %s, type: %s, "
          "is_object_front=%s, is_object_moving=%s, is_object_oncoming=%s, v_norm=%.2f, "
          "hysteresis_factor=%.2f. This is why vehicle stops instead of avoiding!",
          to_hex_string(object.uuid).c_str(), 
          autoware::object_recognition_utils::convertLabelToString(object_type.label).c_str(), 
          is_object_front ? "true" : "false",
          is_object_moving ? "true" : "false", is_object_oncoming ? "true" : "false", v_norm,
          hysteresis_factor);

        safe_count_ = 0;
        return false;
      }
    }
    utils::path_safety_checker::updateCollisionCheckDebugMap(
      debug.collision_check, current_debug_data, true);
  }

  safe_count_++;

  const bool is_safe = safe_ || safe_count_ > parameters_->hysteresis_factor_safe_count;
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 3000,
    "[DEBUG] isSafePath: %s - safe_=%s, safe_count_=%zu, hysteresis_factor_safe_count=%zu",
    is_safe ? "TRUE" : "FALSE", safe_ ? "true" : "false", safe_count_,
    parameters_->hysteresis_factor_safe_count);

  return is_safe;
}

PathWithLaneId StaticObstacleAvoidanceModule::extendBackwardLength(
  const PathWithLaneId & original_path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto previous_path = helper_->getPreviousReferencePath();

  const auto longest_dist_to_shift_point = [&]() {
    double max_dist = 0.0;
    auto lines = path_shifter_.getShiftLines();
    if (lines.empty()) {
      return max_dist;
    }
    std::sort(lines.begin(), lines.end(), [](const auto & a, const auto & b) {
      return a.start_idx < b.start_idx;
    });
    return std::max(
      max_dist, autoware::motion_utils::calcSignedArcLength(
                  previous_path.points, lines.front().start.position, getEgoPosition()));
  }();

  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length = std::max(
    planner_data_->parameters.backward_path_length, longest_dist_to_shift_point + extra_margin);

  const size_t orig_ego_idx = planner_data_->findEgoIndex(original_path.points);
  const auto prev_ego_idx = autoware::motion_utils::findNearestSegmentIndex(
    previous_path.points, autoware_utils::get_pose(original_path.points.at(orig_ego_idx)),
    std::numeric_limits<double>::max(), planner_data_->parameters.ego_nearest_yaw_threshold);
  if (!prev_ego_idx) {
    return original_path;
  }

  size_t clip_idx = 0;
  double accumulated_length = 0.0;
  for (size_t i = prev_ego_idx.value(); i > 0; i--) {
    accumulated_length +=
      autoware_utils::calc_distance2d(previous_path.points.at(i - 1), previous_path.points.at(i));
    if (accumulated_length > backward_length) {
      clip_idx = i;
      break;
    }
  }

  PathWithLaneId extended_path{};
  {
    extended_path.points.insert(
      extended_path.points.end(), previous_path.points.begin() + clip_idx,
      previous_path.points.begin() + *prev_ego_idx);
  }

  // overwrite backward path velocity by latest one.
  std::for_each(extended_path.points.begin(), extended_path.points.end(), [&](auto & p) {
    p.point.longitudinal_velocity_mps =
      original_path.points.at(orig_ego_idx).point.longitudinal_velocity_mps;
  });

  {
    extended_path.points.insert(
      extended_path.points.end(), original_path.points.begin() + orig_ego_idx,
      original_path.points.end());
  }

  return extended_path;
}

auto StaticObstacleAvoidanceModule::getTurnSignal(
  const ShiftedPath & spline_shift_path, const ShiftedPath & linear_shift_path) -> TurnSignalInfo
{
  using autoware::motion_utils::calcSignedArcLength;

  const auto is_ignore_signal = [this](const UUID & uuid) {
    return ignore_signal_ids_.find(to_hex_string(uuid)) != ignore_signal_ids_.end();
  };

  const auto update_ignore_signal = [this](const UUID & uuid, const bool is_ignore) {
    if (is_ignore) {
      ignore_signal_ids_.insert(to_hex_string(uuid));
    }
  };

  const auto is_large_deviation = [this](const auto & path) {
    constexpr double threshold = 1.0;
    const auto current_seg_idx = planner_data_->findEgoSegmentIndex(path.points);
    const auto lateral_deviation =
      autoware::motion_utils::calcLateralOffset(path.points, getEgoPosition(), current_seg_idx);
    return std::abs(lateral_deviation) > threshold;
  };

  auto shift_lines = path_shifter_.getShiftLines();
  if (shift_lines.empty()) {
    return getPreviousModuleOutput().turn_signal_info;
  }

  if (is_large_deviation(spline_shift_path.path)) {
    return getPreviousModuleOutput().turn_signal_info;
  }

  const auto target_shift_line = [&]() {
    const auto & s1 = shift_lines.front();

    for (size_t i = 1; i < shift_lines.size(); i++) {
      const auto & s2 = shift_lines.at(i);

      const auto s1_relative_length = s1.start_shift_length - s1.end_shift_length;
      const auto s2_relative_length = s2.start_shift_length - s2.end_shift_length;

      // same side shift
      if (s1_relative_length > 0.0 && s2_relative_length > 0.0) {
        continue;
      }

      // same side shift
      if (s1_relative_length < 0.0 && s2_relative_length < 0.0) {
        continue;
      }

      // different side shift
      const auto & points = path_shifter_.getReferencePath().points;
      const size_t idx = planner_data_->findEgoIndex(points);

      // output turn signal for near shift line.
      if (calcSignedArcLength(points, idx, s1.start_idx) > 0.0) {
        return s1;
      }

      // output turn signal for far shift line.
      if (
        calcSignedArcLength(points, idx, s2.start_idx) <
        getEgoSpeed() * parameters_->max_prepare_time) {
        return s2;
      }

      // output turn signal for near shift line.
      return s1;
    }

    return s1;
  }();

  if (is_ignore_signal(target_shift_line.id)) {
    return getPreviousModuleOutput().turn_signal_info;
  }

  const auto original_signal = getPreviousModuleOutput().turn_signal_info;

  constexpr bool is_driving_forward = true;
  constexpr bool egos_lane_is_shifted = true;

  const auto [new_signal, is_ignore] = planner_data_->getBehaviorTurnSignalInfo(
    linear_shift_path, target_shift_line, avoid_data_.current_lanelets, helper_->getEgoShift(),
    is_driving_forward, egos_lane_is_shifted);

  update_ignore_signal(target_shift_line.id, is_ignore);

  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(spline_shift_path.path.points);
  return planner_data_->turn_signal_decider.overwrite_turn_signal(
    spline_shift_path.path, getEgoPose(), current_seg_idx, original_signal, new_signal,
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);
}
/**
 * @brief 静态障碍物避让主路径规划输出函数
 *
 * 此函数是静态障碍物避让模块的主要规划入口。根据当前周期的避让规划数据，结合车道状态和历史信息，
 * 进行如下处理流程：
 *
 * 1. 状态出口判断（优先快速返回）:
 *    - SUCCEEDED：避让成功，清除已注册的移位线，直接输出上一周期模块结果。
 *    - CANCEL：避让被取消，清除已注册移位线，返回上一周期结果。
 *    - yield_required：需要让行，清除已注册移位线，不直接返回，上游还可覆盖新路径。
 *
 * 2. 基于参考路径生成加移位点的路径：
 *    - 生成线性移位路径和样条移位路径。
 *    - 尝试用 PathShifter 进行路径生成。
 *    - 若两种都生成成功，则记录本周期结果，否则回退使用上一周期的规划路径。
 *
 * 3. 填充输出及调试信息：
 *    - 调用 getTurnSignal 输出转向灯建议。
 *    - 对生成的路径做稀疏重采样，降低后续计算压力。
 *    - 更新自车行为状态与调试可视化 Marker。
 *
 * 4. 路径选择逻辑：
 *    - 若本周期依然位于上一周期的同一组车道中，则路径输出用当前样条移位路径。
 *    - 若已切换到新车道，直接输出上一周期路径并警告，不主动改变路径。
 *
 * 5. 路径裁剪（clip）:
 *    - 依据车辆当前位置，将路径裁剪，前向/后向最大长度由参数指定，防止路径冗余。
 *
 * 6. 可行驶区域生成：
 *    - 基于当前规划涉及所有车道，生成并扩展可行驶 Lane。
 *    - 按参数配置处理斜线区、交叉口、自由空间、障碍物多边形等额外可行驶/不可行区域。
 *    - 当前可行驶区域会与上一周期模块输出的区域合并。
 *    - 刷新模块内部可行驶 Lane 集合。
 *
 * @return BehaviorModuleOutput 包含路径、可行驶区、多态转向灯建议等的输出结构体
 */
BehaviorModuleOutput StaticObstacleAvoidanceModule::plan()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  // 复位候选路径和参考路径
  resetPathCandidate();
  resetPathReference();

  // 依据本周期安全移位线信息，更新 PathShifter 内部数据
  updatePathShifter(data.safe_shift_line);

  // --- 1. 状态出口优先处理 ---
  if (data.state == AvoidanceState::SUCCEEDED) {
    removeRegisteredShiftLines(State::SUCCEEDED);
    return getPreviousModuleOutput();
  }
  if (data.state == AvoidanceState::CANCEL) {
    removeRegisteredShiftLines(State::FAILED);
    return getPreviousModuleOutput();
  }
  if (data.yield_required) {
    removeRegisteredShiftLines(State::FAILED);
  }

  // --- 2. 生成移位路径（线性/样条） ---
  ShiftedPath linear_shift_path = utils::static_obstacle_avoidance::toShiftedPath(data.reference_path);
  ShiftedPath spline_shift_path = utils::static_obstacle_avoidance::toShiftedPath(data.reference_path);
  const auto success_spline_path_generation =
    path_shifter_.generate(&spline_shift_path, true, SHIFT_TYPE::SPLINE);
  const auto success_linear_path_generation =
    path_shifter_.generate(&linear_shift_path, true, SHIFT_TYPE::LINEAR);

  // 记录本周期路径或回退到上周期
  if (success_spline_path_generation && success_linear_path_generation) {
    helper_->setPreviousLinearShiftPath(linear_shift_path);
    helper_->setPreviousSplineShiftPath(spline_shift_path);
    helper_->setPreviousReferencePath(path_shifter_.getReferencePath());
  } else {
    spline_shift_path = helper_->getPreviousSplineShiftPath();
  }

  BehaviorModuleOutput output;

  // --- 3. 结果输出与可视化处理 ---
  // (1) 填充转向灯建议
  output.turn_signal_info = getTurnSignal(spline_shift_path, linear_shift_path);

  // (2) 对输出路径做稀疏重采样以减小点数
  spline_shift_path.path = utils::resamplePathWithSpline(
    spline_shift_path.path, parameters_->resample_interval_for_output);

  // (3) 更新自车避障状态、调试 Marker 等可视化信息
  updateEgoBehavior(data, spline_shift_path);
  updateInfoMarker(avoid_data_);
  updateDebugMarker(output, avoid_data_, path_shifter_, debug_data_);

  // --- 4. 路径选择决策 ---
  if (isDrivingSameLane(helper_->getPreviousDrivingLanes(), data.current_lanelets)) {
    // 若车道类型未发生变化，按当前规划路径输出
    output.path = spline_shift_path.path;
  } else {
    // 若有车道变化，则保持使用上一周期路径并输出警告
    output.path = getPreviousModuleOutput().path;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000, "Previous module lane is updated. Do nothing.");
  }

  // 参考路径用于后续可行驶区域生成和候选输出
  output.reference_path = getPreviousModuleOutput().reference_path;
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  // --- 5. 路径裁剪（依据自车里程，只保留有效段） ---
  const size_t ego_idx = planner_data_->findEgoIndex(output.path.points);
  utils::clipPathLength(
    output.path, ego_idx, planner_data_->parameters.forward_path_length,
    planner_data_->parameters.backward_path_length);

  // --- 6. 可行驶区域生成及障碍物填充 ---
  {
    DrivableAreaInfo current_drivable_area_info;
    // 生成所有可行驶 Lane
    std::for_each(
      data.current_lanelets.begin(), data.current_lanelets.end(), [&](const auto & lanelet) {
        current_drivable_area_info.drivable_lanes.push_back(
          utils::static_obstacle_avoidance::generateExpandedDrivableLanes(
            lanelet, planner_data_, parameters_));
      });

    // 各项可行驶区域扩展开关
    current_drivable_area_info.enable_expanding_hatched_road_markings =
      parameters_->use_hatched_road_markings;
    current_drivable_area_info.enable_expanding_intersection_areas =
      parameters_->use_intersection_areas;
    current_drivable_area_info.enable_expanding_freespace_areas = parameters_->use_freespace_areas;
    current_drivable_area_info.obstacles.clear();

    // 若为优化/混合路径生成模式，添加障碍物多边形
    if (
      parameters_->path_generation_method == "optimization_base" ||
      parameters_->path_generation_method == "both") {
      current_drivable_area_info.obstacles =
        utils::static_obstacle_avoidance::generateObstaclePolygonsForDrivableArea(
          clip_objects_, parameters_, planner_data_->parameters.vehicle_width / 2.0);
    }

    // 合成当前与前周期可行驶区域
    output.drivable_area_info = utils::combineDrivableAreaInfo(
      current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);

    // 内部存储最新可行驶 lane 集合
    setDrivableLanes(output.drivable_area_info.drivable_lanes);
  }

  return output;
}

CandidateOutput StaticObstacleAvoidanceModule::planCandidate() const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  CandidateOutput output;

  auto shifted_path = data.candidate_path;

  if (data.safe_shift_line.empty()) {
    const size_t ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);
    utils::clipPathLength(
      shifted_path.path, ego_idx, planner_data_->parameters.forward_path_length,
      planner_data_->parameters.backward_path_length);

    output.path_candidate = shifted_path.path;
    return output;
  }

  const auto sl = helper_->getMainShiftLine(data.safe_shift_line);
  const auto sl_front = data.safe_shift_line.front();
  const auto sl_back = data.safe_shift_line.back();

  utils::clipPathLength(
    shifted_path.path, sl_front.start_idx, std::numeric_limits<double>::max(), 0.0);

  output.lateral_shift = helper_->getRelativeShiftToPath(sl);
  output.start_distance_to_path_change = sl_front.start_longitudinal;
  output.finish_distance_to_path_change = sl_back.end_longitudinal;

  const uint16_t planning_factor_direction = std::invoke([&output]() {
    return output.lateral_shift > 0.0 ? PlanningFactor::SHIFT_LEFT : PlanningFactor::SHIFT_RIGHT;
  });

  planning_factor_interface_->add(
    output.start_distance_to_path_change, output.finish_distance_to_path_change, sl_front.start,
    sl_back.end, planning_factor_direction,
    utils::path_safety_checker::to_safety_factor_array(debug_data_.collision_check), true, 0.0,
    output.lateral_shift);

  output.path_candidate = shifted_path.path;
  return output;
}

BehaviorModuleOutput StaticObstacleAvoidanceModule::planWaitingApproval()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  BehaviorModuleOutput out = plan();

  if (path_shifter_.getShiftLines().empty()) {
    out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  }

  path_candidate_ = std::make_shared<PathWithLaneId>(planCandidate().path_candidate);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  return out;
}

void StaticObstacleAvoidanceModule::updatePathShifter(const AvoidLineArray & shift_lines)
{
  if (parameters_->path_generation_method == "optimization_base") {
    return;
  }

  if (shift_lines.empty()) {
    return;
  }

  if (!isActivated()) {
    return;
  }

  addNewShiftLines(path_shifter_, shift_lines);

  generator_.setRawRegisteredShiftLine(shift_lines, avoid_data_);

  const auto sl = helper_->getMainShiftLine(shift_lines);
  const auto & sl_front = shift_lines.front();
  const auto & sl_back = shift_lines.back();
  const auto relative_longitudinal = sl_back.end_longitudinal - sl_front.start_longitudinal;

  if (helper_->getRelativeShiftToPath(sl) > 0.0) {
    left_shift_array_.push_back(
      {uuid_map_.at("left"), sl_front.start, sl_back.end, relative_longitudinal});
  } else if (helper_->getRelativeShiftToPath(sl) < 0.0) {
    right_shift_array_.push_back(
      {uuid_map_.at("right"), sl_front.start, sl_back.end, relative_longitudinal});
  }

  uuid_map_.at("left") = generate_uuid();
  uuid_map_.at("right") = generate_uuid();
  candidate_uuid_ = generate_uuid();

  lockNewModuleLaunch();
}

/**
 * set new shift points. remove old shift points if it has a conflict.
 */
void StaticObstacleAvoidanceModule::addNewShiftLines(
  PathShifter & path_shifter, const AvoidLineArray & new_shift_lines) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  ShiftLineArray future = utils::static_obstacle_avoidance::toShiftLineArray(new_shift_lines);

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sl : new_shift_lines) {
    min_start_idx = std::min(min_start_idx, sl.start_idx);
  }

  const auto current_shift_lines = path_shifter.getShiftLines();
  const auto front_new_shift_line = new_shift_lines.front();
  const auto new_shift_length = front_new_shift_line.end_shift_length;
  const auto new_shift_end_idx = front_new_shift_line.end_idx;

  RCLCPP_DEBUG(getLogger(), "min_start_idx = %lu", min_start_idx);

  // Remove shift points that starts later than the new_shift_line from path_shifter.
  //
  // Why? Because shifter sorts by start position and applies shift points, so if there is a
  // shift point that starts after the one you are going to put in, new one will be affected
  // by the old one.
  //
  // Is it ok? This is a situation where the vehicle was originally going to avoid at the farther
  // point, but decided to avoid it at a closer point. In this case, it is reasonable to cancel the
  // farther avoidance.
  for (const auto & sl : current_shift_lines) {
    if (sl.start_idx >= min_start_idx) {
      RCLCPP_DEBUG(
        getLogger(), "sl.start_idx = %lu, this sl starts after new proposal. remove this one.",
        sl.start_idx);
      continue;
    }

    if (sl.end_idx > new_shift_end_idx) {
      if (
        sl.end_shift_length > -1e-3 && new_shift_length > -1e-3 &&
        sl.end_shift_length < new_shift_length) {
        continue;
      }

      if (
        sl.end_shift_length < 1e-3 && new_shift_length < 1e-3 &&
        sl.end_shift_length > new_shift_length) {
        continue;
      }
    }

    RCLCPP_DEBUG(getLogger(), "sl.start_idx = %lu, no conflict. keep this one.", sl.start_idx);
    future.push_back(sl);
  }

  const double road_velocity = avoid_data_.reference_path.points.at(front_new_shift_line.start_idx)
                                 .point.longitudinal_velocity_mps;
  const double shift_time = autoware::motion_utils::calc_shift_time_from_jerk(
    front_new_shift_line.getRelativeLength(), helper_->getLateralMaxJerkLimit(),
    helper_->getLateralMaxAccelLimit());
  const double longitudinal_acc =
    std::clamp(road_velocity / shift_time, 0.0, parameters_->max_acceleration);

  path_shifter.setShiftLines(future);
  path_shifter.setVelocity(getEgoSpeed());
  path_shifter.setLongitudinalAcceleration(longitudinal_acc);
  path_shifter.setLateralAccelerationLimit(helper_->getLateralMaxAccelLimit());
}

bool StaticObstacleAvoidanceModule::isValidShiftLine(
  const AvoidLineArray & shift_lines, const PathShifter & shifter) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (shift_lines.empty()) {
    return true;
  }

  auto shifter_for_validate = shifter;

  addNewShiftLines(shifter_for_validate, shift_lines);

  ShiftedPath proposed_shift_path;
  shifter_for_validate.generate(&proposed_shift_path);

  debug_data_.proposed_spline_shift = proposed_shift_path.shift_length;

  // check offset between new shift path and ego position.
  {
    const auto new_idx = planner_data_->findEgoIndex(proposed_shift_path.path.points);
    const auto new_shift_length = proposed_shift_path.shift_length.at(new_idx);

    constexpr double THRESHOLD = 0.1;
    const auto offset = std::abs(new_shift_length - helper_->getEgoShift());
    if (offset > THRESHOLD) {
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 3000, "new shift line is invalid. [HUGE OFFSET (%.2f)]", offset);
      return false;
    }
  }

  const auto is_return_shift =
    [](const double start_shift_length, const double end_shift_length, const double threshold) {
      return std::abs(start_shift_length) > threshold && std::abs(end_shift_length) < threshold;
    };

  // check if the vehicle is in road. (yaw angle is not considered)
  {
    const auto minimum_distance = 0.5 * planner_data_->parameters.vehicle_width +
                                  parameters_->hard_drivable_bound_margin -
                                  parameters_->max_deviation_from_lane;

    for (const auto & shift_line : shift_lines) {
      const size_t start_idx = shift_line.start_idx;
      const size_t end_idx = shift_line.end_idx;

      if (is_return_shift(
            shift_line.start_shift_length, shift_line.end_shift_length,
            parameters_->lateral_small_shift_threshold)) {
        continue;
      }

      const auto path = shifter_for_validate.getReferencePath();
      const auto left_bound = lanelet::utils::to2D(toLineString3d(avoid_data_.left_bound));
      const auto right_bound = lanelet::utils::to2D(toLineString3d(avoid_data_.right_bound));
      for (size_t i = start_idx; i <= end_idx; ++i) {
        const auto p = get_point(path.points.at(i));
        lanelet::BasicPoint2d basic_point{p.x, p.y};

        const auto shift_length = proposed_shift_path.shift_length.at(i);
        const auto THRESHOLD = minimum_distance + std::abs(shift_length);

        if (
          boost::geometry::distance(basic_point, (shift_length > 0.0 ? left_bound : right_bound)) <
          THRESHOLD) {
          RCLCPP_INFO_THROTTLE(
            getLogger(), *clock_, 3000,
            "following latest new shift line may cause deviation from drivable area.");
          return false;
        }
      }
    }
  }

  return true;  // valid shift line.
}

void StaticObstacleAvoidanceModule::updateData()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  using utils::static_obstacle_avoidance::toShiftedPath;

  helper_->setData(planner_data_);

  if (!helper_->isInitialized()) {
    helper_->setPreviousSplineShiftPath(toShiftedPath(getPreviousModuleOutput().path));
    helper_->setPreviousLinearShiftPath(toShiftedPath(getPreviousModuleOutput().path));
    helper_->setPreviousReferencePath(getPreviousModuleOutput().path);
    helper_->setPreviousDrivingLanes(utils::static_obstacle_avoidance::getCurrentLanesFromPath(
      getPreviousModuleOutput().reference_path, planner_data_));
  }

  debug_data_ = DebugData();
  avoid_data_ = AvoidancePlanningData();

  // update base path and target objects.
  fillFundamentalData(avoid_data_, debug_data_);

  // an empty path will kill further processing
  if (avoid_data_.reference_path.points.empty()) {
    return;
  }

  // update shift line generator.
  generator_.setData(planner_data_);
  generator_.setPathShifter(path_shifter_);
  generator_.setHelper(helper_);
  generator_.update(avoid_data_, debug_data_);

  // update shift line and check path safety.
  fillShiftLine(avoid_data_, debug_data_);

  // update ego behavior.
  fillEgoStatus(avoid_data_, debug_data_);

  // update debug data.
  fillDebugData(avoid_data_, debug_data_);

  // update rtc status.
  updateRTCData();

  // update interest objects data
  for (const auto & [uuid, data] : debug_data_.collision_check) {
    const auto color = data.is_safe ? ColorName::GREEN : ColorName::RED;
    setObjectsOfInterestData(data.current_obj_pose, data.obj_shape, color);
  }

  safe_ = avoid_data_.safe;

  if (!force_deactivated_) {
    last_deactivation_triggered_time_ = clock_->now();
    force_deactivated_ = avoid_data_.force_deactivated;
    return;
  }

  if (
    (clock_->now() - last_deactivation_triggered_time_).seconds() >
    parameters_->force_deactivate_duration_time) {
    RCLCPP_INFO_THROTTLE(getLogger(), *clock_, 3000, "The force deactivation is released");
    force_deactivated_ = false;
  }
}

void StaticObstacleAvoidanceModule::processOnEntry()
{
  initVariables();
}

void StaticObstacleAvoidanceModule::processOnExit()
{
  initVariables();
  initRTCStatus();
}

void StaticObstacleAvoidanceModule::initVariables()
{
  helper_->reset();
  generator_.reset();
  path_shifter_ = PathShifter{};
  debug_data_ = DebugData();
  debug_marker_.markers.clear();
  resetPathCandidate();
  resetPathReference();
  arrived_path_end_ = false;
  ignore_signal_ids_.clear();
}

void StaticObstacleAvoidanceModule::initRTCStatus()
{
  left_shift_array_.clear();
  right_shift_array_.clear();
  uuid_map_.at("left") = generate_uuid();
  uuid_map_.at("right") = generate_uuid();
  candidate_uuid_ = generate_uuid();
}

void StaticObstacleAvoidanceModule::updateRTCData()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  updateRegisteredRTCStatus(helper_->getPreviousSplineShiftPath().path);

  const auto candidates = data.safe ? data.safe_shift_line : data.new_shift_line;

  if (candidates.empty()) {
    removeCandidateRTCStatus();
    return;
  }

  const auto shift_line = helper_->getMainShiftLine(candidates);
  if (helper_->getRelativeShiftToPath(shift_line) > 0.0) {
    removePreviousRTCStatusRight();
  } else if (helper_->getRelativeShiftToPath(shift_line) < 0.0) {
    removePreviousRTCStatusLeft();
  } else {
    RCLCPP_WARN_STREAM(getLogger(), "Direction is UNKNOWN");
  }

  CandidateOutput output;

  const auto & sl_front = candidates.front();
  const auto & sl_back = candidates.back();

  output.path_candidate = data.candidate_path.path;
  output.lateral_shift = helper_->getRelativeShiftToPath(shift_line);
  output.start_distance_to_path_change = sl_front.start_longitudinal;
  output.finish_distance_to_path_change = sl_back.end_longitudinal;

  updateCandidateRTCStatus(output);
}

void StaticObstacleAvoidanceModule::updateInfoMarker(const AvoidancePlanningData & data) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  using utils::static_obstacle_avoidance::createAmbiguousObjectsMarkerArray;
  using utils::static_obstacle_avoidance::createStopTargetObjectMarkerArray;
  using utils::static_obstacle_avoidance::createTargetObjectsMarkerArray;

  info_marker_.markers.clear();
  append_marker_array(
    createTargetObjectsMarkerArray(data.target_objects, "target_objects"), &info_marker_);
  append_marker_array(createStopTargetObjectMarkerArray(data), &info_marker_);
  append_marker_array(
    createAmbiguousObjectsMarkerArray(
      data.target_objects, getEgoPose(), parameters_->policy_ambiguous_vehicle),
    &info_marker_);
}

void StaticObstacleAvoidanceModule::updateDebugMarker(
  const BehaviorModuleOutput & output, const AvoidancePlanningData & data,
  const PathShifter & shifter, const DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  debug_marker_.markers.clear();
  debug_marker_ = utils::static_obstacle_avoidance::createDebugMarkerArray(
    output, data, shifter, debug, parameters_);
}

void StaticObstacleAvoidanceModule::updateAvoidanceDebugData(
  std::vector<AvoidanceDebugMsg> & avoidance_debug_msg_array) const
{
  debug_data_.avoidance_debug_msg_array.avoidance_info.clear();
  auto & debug_data_avoidance = debug_data_.avoidance_debug_msg_array.avoidance_info;
  debug_data_avoidance = avoidance_debug_msg_array;
  if (!debug_avoidance_initializer_for_shift_line_.empty()) {
    const bool is_info_old_ =
      (clock_->now() - debug_avoidance_initializer_for_shift_line_time_).seconds() > 0.1;
    if (!is_info_old_) {
      debug_data_avoidance.insert(
        debug_data_avoidance.end(), debug_avoidance_initializer_for_shift_line_.begin(),
        debug_avoidance_initializer_for_shift_line_.end());
    }
  }
}

double StaticObstacleAvoidanceModule::calcDistanceToStopLine(const ObjectData & object) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & p = parameters_;
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;

  //         D5
  //      |<---->|                               D4
  //      |<----------------------------------------------------------------------->|
  // +-----------+            D1                 D2                      D3         +-----------+
  // |           |        |<------->|<------------------------->|<----------------->|           |
  // |    ego    |======= x ======= x ========================= x ==================|    obj    |
  // |           |    stop_point  avoid                       avoid                 |           |
  // +-----------+                start                        end                  +-----------+
  //
  // D1: p.min_prepare_distance
  // D2: min_avoid_distance
  // D3: longitudinal_avoid_margin_front (margin + D5)
  // D4: o_front.longitudinal
  // D5: additional_buffer_longitudinal (base_link2front or 0 depending on the
  // use_conservative_buffer_longitudinal)

  const auto object_type = utils::getHighestProbLabel(object.object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);

  const auto lateral_hard_margin = object.is_parked
                                     ? object_parameter.lateral_hard_margin_for_parked_vehicle
                                     : object_parameter.lateral_hard_margin;
  const auto avoid_margin = lateral_hard_margin * object.distance_factor +
                            object_parameter.lateral_soft_margin + 0.5 * vehicle_width;
  const auto avoidance_distance = helper_->getMinAvoidanceDistance(helper_->getShiftLength(
    object, utils::static_obstacle_avoidance::isOnRight(object), avoid_margin));
  const auto constant_distance = helper_->getFrontConstantDistance(object);
  const auto prepare_distance = helper_->getNominalPrepareDistance(0.0);

  return object.longitudinal -
         std::min(
           avoidance_distance + constant_distance + prepare_distance + p->stop_buffer,
           p->stop_max_distance);
}

void StaticObstacleAvoidanceModule::insertReturnDeadLine(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  if (data.to_return_point > planner_data_->parameters.forward_path_length) {
    RCLCPP_DEBUG(getLogger(), "return dead line is far enough.");
    return;
  }

  const auto shift_length = path_shifter_.getLastShiftLength();

  if (std::abs(shift_length) < 1e-3) {
    RCLCPP_DEBUG(getLogger(), "don't have to consider return shift.");
    return;
  }

  if (data.new_shift_line.empty()) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 3000, "module doesn't have return shift line.");
    return;
  }

  if (!helper_->isFeasible(data.new_shift_line)) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000, "return shift line is not feasible. do nothing..");
    return;
  }

  // Consider the difference in path length between the shifted path and original path (the path
  // that is shifted inward has a shorter distance to the end of the path than the other one.)
  const auto & to_reference_path_end = data.arclength_from_ego.back();
  const auto to_shifted_path_end = autoware::motion_utils::calcSignedArcLength(
    shifted_path.path.points, getEgoPosition(), shifted_path.path.points.size() - 1);
  const auto buffer = std::max(0.0, to_shifted_path_end - to_reference_path_end);

  const auto min_return_distance =
    helper_->getMinAvoidanceDistance(shift_length) + helper_->getNominalPrepareDistance(0.0);
  const auto to_stop_line = data.to_return_point - min_return_distance - buffer;
  if (to_stop_line < -1.0 * parameters_->stop_buffer) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000, "ego overran return shift dead line. do nothing.");
    return;
  }

  // If we don't need to consider deceleration constraints, insert a deceleration point
  // and return immediately
  if (!use_constraints_for_decel) {
    utils::static_obstacle_avoidance::insertDecelPoint(
      getEgoPosition(), to_stop_line - parameters_->stop_buffer, 0.0, shifted_path.path,
      stop_pose_);
    return;
  }

  // If the stop distance is not enough for comfortable stop, don't insert wait point.
  const auto is_comfortable_stop = helper_->getFeasibleDecelDistance(0.0) < to_stop_line;
  if (!is_comfortable_stop) {
    RCLCPP_DEBUG(getLogger(), "stop distance is not enough.");
    return;
  }

  utils::static_obstacle_avoidance::insertDecelPoint(
    getEgoPosition(), to_stop_line - parameters_->stop_buffer, 0.0, shifted_path.path, stop_pose_);

  // insert slow down speed.
  const double current_target_velocity = autoware::motion_utils::calc_feasible_velocity_from_jerk(
    shift_length, helper_->getLateralMinJerkLimit(), to_stop_line);
  if (current_target_velocity < getEgoSpeed()) {
    RCLCPP_DEBUG(getLogger(), "current velocity exceeds target slow down speed.");
    return;
  }

  const auto start_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  for (size_t i = start_idx; i < shifted_path.path.points.size(); ++i) {
    const auto distance_from_ego =
      autoware::motion_utils::calcSignedArcLength(shifted_path.path.points, start_idx, i);

    // slow down speed is inserted only in front of the object.
    const auto shift_longitudinal_distance = to_stop_line - distance_from_ego;
    if (shift_longitudinal_distance < 0.0) {
      break;
    }

    // target speed with nominal jerk limits.
    const double v_target = autoware::motion_utils::calc_feasible_velocity_from_jerk(
      shift_length, helper_->getLateralMinJerkLimit(), shift_longitudinal_distance);
    const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
    const double v_insert =
      std::max(v_target - parameters_->buf_slow_down_speed, parameters_->min_slow_down_speed);

    shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_insert);
  }
}

void StaticObstacleAvoidanceModule::insertWaitPoint(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  // If avoidance path is NOT valid, don't insert any stop points.
  if (!data.valid) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertWaitPoint: Skipping - Path is NOT valid (valid=false)");
    return;
  }

  if (!data.stop_target_object) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertWaitPoint: Skipping - No stop_target_object");
    return;
  }

  if (helper_->isShifted()) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertWaitPoint: Skipping - Already shifted (avoidance maneuver initiated)");
    return;
  }

  if (data.to_stop_line < -1.0 * parameters_->stop_buffer) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000, "ego overran avoidance dead line. do nothing.");
    return;
  }

  // If we don't need to consider deceleration constraints, insert a deceleration point
  // and return immediately
  if (!use_constraints_for_decel) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertWaitPoint: Inserting decel point (to_stop_line=%.2f, use_constraints=false)",
      data.to_stop_line);
    utils::static_obstacle_avoidance::insertDecelPoint(
      getEgoPosition(), data.to_stop_line, 0.0, shifted_path.path, stop_pose_);
    return;
  }

  // If the stop distance is not enough for comfortable stop, don't insert wait point.
  const auto is_comfortable_stop = helper_->getFeasibleDecelDistance(0.0) < data.to_stop_line;
  const auto is_slow_speed = getEgoSpeed() < parameters_->min_slow_down_speed;
  if (!is_comfortable_stop && !is_slow_speed) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertWaitPoint: Skipping - Not comfortable stop (feasible_decel_distance=%.2f > "
      "to_stop_line=%.2f, ego_speed=%.2f >= min_slow_down_speed=%.2f)",
      helper_->getFeasibleDecelDistance(0.0), data.to_stop_line, getEgoSpeed(),
      parameters_->min_slow_down_speed);
    return;
  }

  // If target object can be stopped for, insert a deceleration point and return
  if (data.stop_target_object.value().is_stoppable) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertWaitPoint: Inserting decel point - Object is stoppable (to_stop_line=%.2f)",
      data.to_stop_line);
    utils::static_obstacle_avoidance::insertDecelPoint(
      getEgoPosition(), data.to_stop_line, 0.0, shifted_path.path, stop_pose_);
    return;
  }

  // If the object cannot be stopped for, calculate a "mild" deceleration distance
  // and insert a deceleration point at that distance
  const auto stop_distance = helper_->getFeasibleDecelDistance(0.0, false);
  utils::static_obstacle_avoidance::insertDecelPoint(
    getEgoPosition(), stop_distance, 0.0, shifted_path.path, stop_pose_);
}

void StaticObstacleAvoidanceModule::insertStopPoint(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  if (data.safe) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertStopPoint: Skipping - Path is safe (safe=true)");
    return;
  }

  if (!parameters_->enable_yield_maneuver_during_shifting) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 3000,
      "[DEBUG] insertStopPoint: Skipping - Yield maneuver during shifting is disabled");
    return;
  }

  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 3000,
    "[DEBUG] insertStopPoint: Inserting STOP POINT - Path is unsafe during shifting (safe=false, "
    "enable_yield_maneuver_during_shifting=true)");

  const auto stop_idx = [&]() {
    const auto ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);
    for (size_t idx = ego_idx; idx < shifted_path.path.points.size(); ++idx) {
      const auto & estimated_pose = shifted_path.path.points.at(idx).point.pose;
      if (!utils::isEgoWithinOriginalLane(
            data.current_lanelets, estimated_pose, planner_data_->parameters)) {
        return idx - 1;
      }
    }

    return shifted_path.path.points.size() - 1;
  }();

  const auto stop_distance = autoware::motion_utils::calcSignedArcLength(
    shifted_path.path.points, getEgoPosition(), stop_idx);

  // If we don't need to consider deceleration constraints, insert a deceleration point
  // and return immediately
  if (!use_constraints_for_decel) {
    utils::static_obstacle_avoidance::insertDecelPoint(
      getEgoPosition(), stop_distance, 0.0, shifted_path.path, stop_pose_);
    return;
  }

  // Otherwise, consider deceleration constraints before inserting deceleration point
  const auto decel_distance = helper_->getFeasibleDecelDistance(0.0, false);
  if (stop_distance < decel_distance) {
    return;
  }

  constexpr double MARGIN = 1.0;
  utils::static_obstacle_avoidance::insertDecelPoint(
    getEgoPosition(), stop_distance - MARGIN, 0.0, shifted_path.path, stop_pose_);
}

void StaticObstacleAvoidanceModule::insertPrepareVelocity(ShiftedPath & shifted_path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  // If avoidance path is NOT safe, don't insert any slow down sections.
  if (!data.safe && !data.stop_target_object) {
    return;
  }

  // If avoidance path is NOT safe, don't insert any slow down sections.
  if (!data.valid) {
    return;
  }

  // do nothing if there is no avoidance target.
  if (data.target_objects.empty()) {
    return;
  }

  // insert slow down speed only when the avoidance maneuver is not initiated.
  if (helper_->isShifted()) {
    return;
  }

  // insert slow down speed only when no shift line is approved.
  if (!path_shifter_.getShiftLines().empty()) {
    return;
  }

  const auto itr = std::find_if(
    data.target_objects.begin(), data.target_objects.end(),
    [](const auto & o) { return o.avoid_required; });

  const auto object = [&]() -> std::optional<ObjectData> {
    if (!data.yield_required) {
      return data.target_objects.front();
    }

    if (itr == data.target_objects.end()) {
      return std::nullopt;
    }

    return *itr;
  }();

  // do nothing if it can't avoid at the moment and avoidance is NOT definitely necessary.
  if (!object.has_value()) {
    return;
  }

  if (helper_->isAbsolutelyNotAvoidable(object.value())) {
    return;
  }

  // calculate shift length for front object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  const auto object_type = utils::getHighestProbLabel(object.value().object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);
  const auto lateral_hard_margin = object.value().is_parked
                                     ? object_parameter.lateral_hard_margin_for_parked_vehicle
                                     : object_parameter.lateral_hard_margin;
  const auto avoid_margin = lateral_hard_margin * object.value().distance_factor +
                            object_parameter.lateral_soft_margin + 0.5 * vehicle_width;
  const auto shift_length = helper_->getShiftLength(
    object.value(), utils::static_obstacle_avoidance::isOnRight(object.value()), avoid_margin);

  // check slow down feasibility
  const auto min_avoid_distance = helper_->getMinAvoidanceDistance(shift_length);
  const auto distance_to_object = object.value().longitudinal;
  const auto remaining_distance = distance_to_object - min_avoid_distance;
  const auto decel_distance = helper_->getFeasibleDecelDistance(parameters_->velocity_map.front());
  if (remaining_distance < decel_distance) {
    return;
  }

  // decide slow down lower limit.
  const auto lower_speed = object.value().avoid_required ? 0.0 : parameters_->min_slow_down_speed;

  // insert slow down speed.
  const double current_target_velocity = autoware::motion_utils::calc_feasible_velocity_from_jerk(
    shift_length, helper_->getLateralMinJerkLimit(), distance_to_object);
  if (current_target_velocity < getEgoSpeed() + parameters_->buf_slow_down_speed) {
    utils::static_obstacle_avoidance::insertDecelPoint(
      getEgoPosition(), decel_distance, parameters_->velocity_map.front(), shifted_path.path,
      slow_pose_);
    return;
  }

  const auto start_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  for (size_t i = start_idx; i < shifted_path.path.points.size(); ++i) {
    const auto distance_from_ego =
      autoware::motion_utils::calcSignedArcLength(shifted_path.path.points, start_idx, i);

    // slow down speed is inserted only in front of the object.
    const auto shift_longitudinal_distance = distance_to_object - distance_from_ego;
    if (shift_longitudinal_distance < min_avoid_distance) {
      break;
    }

    // target speed with nominal jerk limits.
    const double v_target = autoware::motion_utils::calc_feasible_velocity_from_jerk(
      shift_length, helper_->getLateralMinJerkLimit(), shift_longitudinal_distance);
    const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
    const double v_insert = std::max(v_target - parameters_->buf_slow_down_speed, lower_speed);

    shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_insert);
  }

  const auto slow_pose_opt = autoware::motion_utils::calcLongitudinalOffsetPose(
    shifted_path.path.points, start_idx, distance_to_object);
  slow_pose_ = slow_pose_opt.has_value() ? PoseWithDetailOpt(PoseWithDetail(slow_pose_opt.value()))
                                         : std::nullopt;
}

void StaticObstacleAvoidanceModule::insertAvoidanceVelocity(ShiftedPath & shifted_path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & data = avoid_data_;

  // do nothing if no shift line is approved.
  if (path_shifter_.getShiftLines().empty()) {
    return;
  }

  // do nothing if there is no avoidance target.
  if (data.target_objects.empty()) {
    return;
  }

  const auto [distance_to_accel_end_point, v_max] =
    helper_->getDistanceToAccelEndPoint(shifted_path.path);
  if (distance_to_accel_end_point < 1e-3) {
    return;
  }

  const auto start_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  for (size_t i = start_idx; i < shifted_path.path.points.size(); ++i) {
    const auto distance_from_ego =
      autoware::motion_utils::calcSignedArcLength(shifted_path.path.points, start_idx, i);

    // slow down speed is inserted only in front of the object.
    const auto accel_distance = distance_to_accel_end_point - distance_from_ego;
    if (accel_distance < 0.0) {
      break;
    }

    const double v_target_square =
      v_max * v_max - 2.0 * parameters_->max_acceleration * accel_distance;
    if (v_target_square < 1e-3) {
      break;
    }

    // target speed with nominal jerk limits.
    const double v_target = std::max(getEgoSpeed(), std::sqrt(v_target_square));
    const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
    shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_target);
  }

  const auto slow_pose_opt = autoware::motion_utils::calcLongitudinalOffsetPose(
    shifted_path.path.points, start_idx, distance_to_accel_end_point);
  slow_pose_ = slow_pose_opt.has_value() ? PoseWithDetailOpt(PoseWithDetail(slow_pose_opt.value()))
                                         : std::nullopt;
}

std::shared_ptr<AvoidanceDebugMsgArray> StaticObstacleAvoidanceModule::get_debug_msg_array() const
{
  debug_data_.avoidance_debug_msg_array.header.stamp = clock_->now();
  return std::make_shared<AvoidanceDebugMsgArray>(debug_data_.avoidance_debug_msg_array);
}

void StaticObstacleAvoidanceModule::acceptVisitor(
  const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitAvoidanceModule(this);
  }
}

void SceneModuleVisitor::visitAvoidanceModule(const StaticObstacleAvoidanceModule * module) const
{
  avoidance_visitor_ = module->get_debug_msg_array();
}
}  // namespace autoware::behavior_path_planner
