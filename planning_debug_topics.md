# Planning 模块路径异常排查

本文档总结了 Autoware planning 模块中，用于排查路径异常、异常停车、轨迹形状异常的关键话题与建议排查顺序。

相关文档：

- 现场速查版: [planning_debug_quick_reference.md](planning_debug_quick_reference.md)
- 开发详版: [planning_debug_developer_guide.md](planning_debug_developer_guide.md)

## 1. 主链路话题

按模块链路，最关键的话题如下：

1. 路由层
   - `/planning/mission_planning/route`
   - `/planning/mission_planning/route_marker`
   - `/planning/mission_planning/state`

2. 几何路径层
   - `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
   - `/planning/scenario_planning/lane_driving/behavior_planning/path`

3. 轨迹转换层
   - `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`

4. 速度与障碍物修正层
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
  - `/planning/planning_factors/motion_velocity_planner`
  - `/planning/planning_factors/obstacle_stop`
  - `/api/planning/velocity_factors`
   - `/planning/scenario_planning/max_velocity_candidates`
   - `/planning/scenario_planning/clear_velocity_limit`

5. 最终输出层
   - `/planning/scenario_planning/trajectory`
   - `/planning/scenario_planning/current_max_velocity`

## 2. 每层话题的作用

### 2.1 路由层

- `/planning/mission_planning/route`
  - 表示 mission planner 最终选定的 lanelet route。
  - 如果这里就错了，后面所有 path 和 trajectory 都会跟着错。

- `/planning/mission_planning/route_marker`
  - 用于在 RViz 中直观看 route 是否走对车道。

- `/planning/mission_planning/state`
  - 用于确认 route 状态是否正常更新。

### 2.2 几何路径层

- `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
  - path_generator 输出的原始几何路径，保留 lane_id。
  - 如果这里已经异常，优先怀疑 route、map、定位或者 path_generator。

- `/planning/scenario_planning/lane_driving/behavior_planning/path`
  - behavior_velocity_planner 输出的 path。
  - 如果上游正常、这里异常，通常是行为速度规划模块插入了 stop 或修改了速度。

### 2.3 轨迹转换层

- `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
  - 由 path 转成 trajectory 的中间层。
  - 正常情况下主要是格式转换，不应大幅改变几何形状。

### 2.4 速度与障碍物修正层

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
  - motion_velocity_planner 输出的带速度轨迹。
  - 如果前面的 path 正常，而这里开始异常停车或速度被清零，问题一般在 motion velocity planner。

- `/planning/planning_factors/motion_velocity_planner`
  - 用于查看 motion_velocity_planner 聚合后的规划因素。

- `/planning/planning_factors/obstacle_stop`
  - 用于查看 obstacle_stop 插入停车因素的原因与位置。

- `/api/planning/velocity_factors`
  - 当前运行版本实际可用的统一速度因素接口。

- `/planning/scenario_planning/max_velocity_candidates`
  - 候选限速信息。

- `/planning/scenario_planning/clear_velocity_limit`
  - 清除限速命令。

### 2.5 最终输出层

- `/planning/scenario_planning/trajectory`
  - velocity_smoother 输出的最终轨迹。
  - 如果 motion velocity planner 输出正常，但这个话题异常，优先检查 velocity_smoother。

- `/planning/scenario_planning/current_max_velocity`
  - 当前最终生效的速度上限。

## 3. 最重要的 debug 话题

### 3.1 behavior_velocity_planner

- `/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path`
  - 用于观察 behavior_velocity_planner 在哪里做了 stop 或速度变化。

### 3.2 motion_velocity_planner

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`
  - 用于排查检测区域、碰撞点、障碍物选择结果。

### 3.3 obstacle_stop 模块

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info`
  - 查看 obstacle_stop 的内部判定结果。

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/processing_time_ms`
  - 查看 obstacle_stop 的处理时间。

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`
  - 在 RViz 中查看停车墙位置。

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/debug_markers`
  - 查看 obstacle_stop 模块自己的 marker 输出。

### 3.4 velocity_smoother

- `/planning/scenario_planning/velocity_smoother/debug/trajectory_raw`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_external_velocity_limited`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled`
- `/planning/scenario_planning/velocity_smoother/distance_to_stopline`

这些话题用于确认是哪个平滑或约束步骤把最终轨迹改掉了。

## 4. 必须结合查看的上游输入话题

- `/map/vector_map`
- `/localization/kinematic_state`
- `/localization/acceleration`
- `/perception/object_recognition/objects`
- `/perception/obstacle_segmentation/pointcloud`
- `/perception/occupancy_grid_map/map`
- `/perception/traffic_light_recognition/traffic_signals`
- `/planning/scenario_planning/max_velocity`
- `/planning/scenario_planning/max_velocity_default`

这些输入如果存在 frame 错误、时间戳延迟、障碍物误检、定位漂移等问题，下游路径和轨迹都会表现异常。

## 5. 两类典型异常的排查方法

### 5.1 无故停车

建议按以下顺序检查：

1. 看最终轨迹是否已经被打成 0 速
   - `/planning/scenario_planning/trajectory`

2. 对比 motion_velocity_planner 前后输出
   - `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`

3. 看停车原因
  - `/planning/planning_factors/motion_velocity_planner`
  - `/planning/planning_factors/obstacle_stop`
  - `/api/planning/velocity_factors`

4. 看 obstacle_stop 是否在工作
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`

5. 看是否是外部限速造成的
   - `/planning/scenario_planning/max_velocity`
   - `/planning/scenario_planning/max_velocity_candidates`
   - `/planning/scenario_planning/current_max_velocity`

如果 path_optimizer 正常，而 motion_velocity_planner 输出开始出现停车点，基本可以判定是 motion velocity planner 或其插件在插入 stop。

### 5.2 拐弯异常或轨迹形状异常

建议按以下顺序检查：

1. 看 route 是否正确
   - `/planning/mission_planning/route`
   - `/planning/mission_planning/route_marker`

2. 看 path_generator 输出是否已经异常
   - `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`

3. 看 behavior_velocity_planner 前后是否发生变化
   - `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
   - `/planning/scenario_planning/lane_driving/behavior_planning/path`

4. 看 trajectory 转换和下游速度模块是否造成视觉上的异常
   - `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
   - `/planning/scenario_planning/trajectory`

5. 检查定位和地图是否一致
   - `/localization/kinematic_state`
   - `/map/vector_map`

如果 route_marker 在 RViz 中就已经贴错车道，那不是后级轨迹优化问题，而是 route 本身或 map/定位问题。

## 6. RViz 推荐同时打开的显示

### 6.1 基础链路显示

- `/planning/mission_planning/route_marker`
- `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
- `/planning/scenario_planning/lane_driving/behavior_planning/path`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
- `/planning/scenario_planning/trajectory`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`

### 6.2 无故停车场景补充显示

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`
- `/perception/object_recognition/objects`
- `/perception/obstacle_segmentation/pointcloud`
- `/perception/occupancy_grid_map/map`

### 6.3 转弯异常场景补充显示

- `/localization/kinematic_state`
- `/map/vector_map`

## 7. 常用命令

### 7.1 查看发布频率

```bash
ros2 topic hz /planning/scenario_planning/trajectory
ros2 topic hz /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
ros2 topic hz /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory
ros2 topic hz /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info
```

### 7.2 查看停车和限速原因

```bash
ros2 topic echo /planning/planning_factors/motion_velocity_planner
ros2 topic echo /planning/planning_factors/obstacle_stop
ros2 topic echo /api/planning/velocity_factors
ros2 topic echo /planning/scenario_planning/max_velocity
ros2 topic echo /planning/scenario_planning/current_max_velocity
```

说明：

- 当前运行版本里通常没有 `/planning/scenario_planning/status/stop_reasons`
- 旧文档中的 `/planning/velocity_factors/motion_velocity_planner` 在你当前系统里也不存在

### 7.3 单次抓取关键链路输出

```bash
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory --once
ros2 topic echo /planning/scenario_planning/trajectory --once
```

## 8. 推荐的最省时间排查顺序

1. 在 RViz 中同时打开以下话题
   - `/planning/mission_planning/route_marker`
   - `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
   - `/planning/scenario_planning/lane_driving/behavior_planning/path`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
   - `/planning/scenario_planning/trajectory`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`

2. 如果是无故停车，再补看以下话题
  - `/planning/planning_factors/motion_velocity_planner`
  - `/planning/planning_factors/obstacle_stop`
  - `/api/planning/velocity_factors`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info`
   - `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`

3. 如果是转弯异常，再补看以下话题
   - `/planning/mission_planning/route`
   - `/localization/kinematic_state`
   - `/map/vector_map`

## 9. 当前场景下最建议优先看的话题

如果现象类似前方无明显障碍但轨迹提前停车，优先查看：

1. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`
2. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`
3. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info`
4. `/planning/planning_factors/obstacle_stop`
5. `/api/planning/velocity_factors`

如果这些话题明确指向 obstacle_stop，则应继续排查 perception 输入、occupancy grid 和 obstacle_stop 参数，而不是优先怀疑 path_generator。
