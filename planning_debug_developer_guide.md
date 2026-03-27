# Planning 调试开发详版

适用场景：开发排障、模块定位、源码联动分析、参数和输入问题拆分。

## 1. 目标

当出现以下现象时，使用本文档定位问题来源：

- 路径几何形状异常
- 转弯轨迹不合理
- 无故停车
- 提前停车
- 轨迹速度异常降低
- RViz 中看到路径正常但控制前轨迹异常

## 2. 模块链路与责任边界

planning 主链路可以按以下顺序理解：

1. mission_planner
   - 产出 route
   - 决定车应该沿哪些 lanelet 前进

2. path_generator
   - 将 route 转成 `path_with_lane_id`
   - 本质上决定几何路径骨架

3. behavior_velocity_planner
   - 在 path 上基于规则插入停车、减速等行为
   - 输出 `path`

4. path_to_trajectory_converter
   - 将 `path` 转换为 trajectory 形式
   - 主要是消息层转换

5. motion_velocity_planner
   - 根据障碍物、occupancy grid 等再做速度约束
   - 输出带速度轨迹

6. velocity_smoother
   - 根据加速度、jerk、横向加速度、转向速率等约束平滑最终轨迹

开发排查时，最重要的是区分：

- 几何路径问题
- 速度规划问题
- 输入感知问题
- 地图或定位问题

## 3. 关键话题及开发含义

### 3.1 mission_planner

- `/planning/mission_planning/route`
  - route 本体
  - 用于确认 lanelet 选择是否正确

- `/planning/mission_planning/route_marker`
  - route 的可视化结果
  - 建议在 RViz 中与地图直接叠加

- `/planning/mission_planning/state`
  - route 状态机输出

如果这里异常：

- 先查 map 是否正确加载
- 再查定位是否落在正确 lanelet
- 再查 route 请求本身是否合理

### 3.2 path_generator

- `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
  - 开发上最关键的几何路径输入输出检查点
  - 一旦这里偏离预期，后面所有模块都只是在坏输入上继续处理

开发判断建议：

- 查看转弯入口和转弯出口是否与 lanelet centerline 一致
- 查看 path 是否突然截断
- 查看回弯场景中是否存在路径自交导致裁剪

### 3.3 behavior_velocity_planner

- `/planning/scenario_planning/lane_driving/behavior_planning/path`
  - 在 `path_with_lane_id` 基础上处理后的 path

- `/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path`
  - 用于确认是哪个行为模块改动了 path 或速度分布

- `/planning/planning_factors/stop_line`
  - stop line 模块的停车/减速因素输出

- `/planning/planning_factors/traffic_light`
  - traffic light 模块的停车/减速因素输出

- `/planning/planning_factors/crosswalk`
  - crosswalk 模块的停车/减速因素输出

如果 `path_with_lane_id` 正常、`path` 异常：

- 重点查 behavior_velocity_planner 的启用模块
- 重点查 `stop_line`、`traffic_light`、`crosswalk` 等规则模块对应的 planning factors

### 3.4 motion_velocity_planner

- `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
  - motion_velocity_planner 的上游输入参考

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`
  - motion_velocity_planner 输出结果

- `/planning/planning_factors/motion_velocity_planner`
  - motion_velocity_planner 聚合后的规划因素输出

- `/api/planning/velocity_factors`
  - 当前运行版本里实际可用的统一速度因素接口

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/markers`
  - 建议作为第一优先级 debug marker

如果 `path_optimizer/trajectory` 正常而 `motion_velocity_planner/trajectory` 异常：

- 优先怀疑 obstacle_stop 等插件
- 再看 perception、pointcloud、occupancy grid 输入

### 3.5 obstacle_stop

重点话题：

- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/processing_time_ms`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls`
- `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/debug_markers`

开发排查要点：

- `virtual_walls` 确认停车墙插入位置
- `debug_markers` 确认检测区域、碰撞点、被选中障碍物
- `planning_info` 确认模块内部是否持续判定到 stop obstacle

### 3.6 velocity_smoother

重点话题：

- `/planning/scenario_planning/trajectory`
- `/planning/scenario_planning/current_max_velocity`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_raw`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_external_velocity_limited`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited`
- `/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled`
- `/planning/scenario_planning/velocity_smoother/distance_to_stopline`

如果 motion_velocity_planner 输出正常，但最终 trajectory 异常：

- 检查外部限速是否过低
- 检查横向加速度约束是否在转弯处压速过多
- 检查时间重采样是否造成轨迹观感异常

## 4. 上游输入如何影响 planning

必须联动检查：

- `/map/vector_map`
- `/localization/kinematic_state`
- `/localization/acceleration`
- `/perception/object_recognition/objects`
- `/perception/obstacle_segmentation/pointcloud`
- `/perception/occupancy_grid_map/map`
- `/perception/traffic_light_recognition/traffic_signals`
- `/planning/scenario_planning/max_velocity`
- `/planning/scenario_planning/max_velocity_default`

典型影响关系：

- map 错: route 直接错
- localization 偏: 最近 lanelet 选错，path 可能偏出车道
- objects 误检: obstacle_stop 插入虚拟墙
- occupancy grid 残留: motion velocity planner 误判前方不可通行
- max_velocity 过低: 看起来像“无故减速或停车”

## 5. 两类问题的开发排查流程

### 5.1 无故停车

建议流程：

1. 先确认最终轨迹是否真的插入 0 速点
2. 再对比 `path_optimizer/trajectory` 和 `motion_velocity_planner/trajectory`
3. 查看 `planning_factors` 和 `/api/planning/velocity_factors`
4. 再看 obstacle_stop 的 `planning_info`、`debug_markers`、`virtual_walls`
5. 最后回看 perception 和 occupancy grid 输入

开发结论判据：

- 只有 motion_velocity_planner 后异常: 插件问题或感知输入问题
- 只有最终 trajectory 异常: velocity_smoother 或限速链路问题

### 5.2 转弯异常或轨迹形状异常

建议流程：

1. route 和 route_marker 对 map
2. path_with_lane_id 对 route
3. path 对 path_with_lane_id
4. trajectory 对 path
5. 最终 trajectory 对 motion_velocity_planner/trajectory

开发结论判据：

- route 已错: mission planner、map、定位问题
- path_with_lane_id 已错: path_generator、map、定位问题
- path 才错: behavior_velocity_planner 规则干预
- 只有最终轨迹看起来不顺: velocity_smoother 约束问题

## 6. 建议保留的调试命令

### 6.1 单次对比链路

```bash
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory --once
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory --once
ros2 topic echo /planning/scenario_planning/trajectory --once
```

### 6.2 停车原因与限速

```bash
ros2 topic echo /planning/planning_factors/motion_velocity_planner
ros2 topic echo /planning/planning_factors/obstacle_stop
ros2 topic echo /planning/planning_factors/traffic_light
ros2 topic echo /api/planning/velocity_factors
ros2 topic echo /planning/scenario_planning/max_velocity
ros2 topic echo /planning/scenario_planning/max_velocity_candidates
ros2 topic echo /planning/scenario_planning/current_max_velocity
```

说明：

- 当前运行版本中通常没有 `/planning/scenario_planning/status/stop_reasons`
- 旧文档中的 `/planning/velocity_factors/motion_velocity_planner` 在你当前系统里也不存在
- 对应功能已由 `/planning/planning_factors/*` 与 `/api/planning/velocity_factors` 替代

### 6.3 频率监控

```bash
ros2 topic hz /planning/scenario_planning/trajectory
ros2 topic hz /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
ros2 topic hz /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory
ros2 topic hz /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info
```

## 7. 现场现象与模块推断

### 现象 1: 前方没有明显障碍物，但出现红色停车墙

优先怀疑：

- obstacle_stop
- occupancy grid 残留
- pointcloud 误检
- objects 误检或时间戳异常

### 现象 2: 路线拐弯方向明显不对

优先怀疑：

- route 本身错误
- map 中 lanelet 连接关系
- localization 偏到错误 lanelet

### 现象 3: 几何路径正常，但车速在弯道前被压得很低

优先怀疑：

- velocity_smoother 横向加速度约束
- 外部限速
- motion velocity planner 插件限速

## 8. 使用建议

开发调试时不要只看最终 `/planning/scenario_planning/trajectory`。

最有效的方法是把 route、path、trajectory、debug marker 和 perception 输入同时叠加到 RViz 中，逐层判断第一个发生异常的输出节点。