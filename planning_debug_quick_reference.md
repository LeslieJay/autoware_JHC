# Planning 调试速查表

适用场景：现场联调、RViz 快速定位、短时间判断问题落在哪一级。

## 1. 先盯主链路

按顺序看这 6 个话题：

1. `/planning/mission_planning/route_marker`
2. `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
3. `/planning/scenario_planning/lane_driving/behavior_planning/path`
4. `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
5. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
6. `/planning/scenario_planning/trajectory`

判断原则：

- 前一级正常、后一级异常，问题大概率就在两者之间的模块。
- `route_marker` 错，优先查 route、map、定位。
- `path_with_lane_id` 错，优先查 path_generator。
- `path` 开始异常，优先查 behavior_velocity_planner。
- `motion_velocity_planner/trajectory` 开始异常，优先查 motion velocity planner。
- 只有最终 `/planning/scenario_planning/trajectory` 异常，优先查 velocity_smoother。

## 2. 无故停车时看什么

优先看这 5 个：

1. `/planning/planning_factors/motion_velocity_planner`
2. `/planning/planning_factors/obstacle_stop`
3. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info`
4. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`
5. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`

快速判断：

- 有 `virtual_walls`，通常说明已有模块在插入停车点。
- `planning_info` 持续输出，说明 obstacle_stop 正在工作。
- `planning_factors` 和 `/api/planning/velocity_factors` 能直接告诉你是谁触发停车或限速。

## 3. 转弯异常时看什么

优先看这 5 个：

1. `/planning/mission_planning/route`
2. `/planning/mission_planning/route_marker`
3. `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
4. `/localization/kinematic_state`
5. `/map/vector_map`

快速判断：

- `route_marker` 贴错车道，不是后级轨迹问题。
- `path_with_lane_id` 已经拐坏，问题通常在 route、map、定位或 path_generator。
- 轨迹看着偏，先确认是不是速度被打零造成的视觉误判。

## 4. RViz 最少要开哪些显示

基础显示：

- `/planning/mission_planning/route_marker`
- `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
- `/planning/scenario_planning/lane_driving/behavior_planning/path`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
- `/planning/scenario_planning/trajectory`

停车问题补充：

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`
- `/perception/object_recognition/objects`
- `/perception/obstacle_segmentation/pointcloud`

## 5. 最常用命令

查看链路输出：

```bash
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory --once
ros2 topic echo /planning/scenario_planning/trajectory --once
```

查看停车原因：

```bash
ros2 topic echo /planning/planning_factors/motion_velocity_planner
ros2 topic echo /planning/planning_factors/obstacle_stop
ros2 topic echo /api/planning/velocity_factors
```

说明：

- 当前运行版本里通常没有 `/planning/scenario_planning/status/stop_reasons`
- 旧文档中的 `/planning/velocity_factors/motion_velocity_planner` 在你当前系统里也不存在


查看频率：

```bash
ros2 topic hz /planning/scenario_planning/trajectory
ros2 topic hz /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
ros2 topic hz /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory
```

## 6. 一句话排查顺序

先看 route，再看 path_with_lane_id，再看 path，再看 motion_velocity_planner/trajectory，最后看 scenario_planning/trajectory；谁先变坏，就先查谁对应的模块。