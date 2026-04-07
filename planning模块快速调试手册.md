<!--
 * @Author: leslie leslie@byd.com
 * @Date: 2026-03-30 16:33:44
 * @LastEditors: leslie
 * @LastEditTime: 2026-04-07 08:53:14
 * @FilePath: /autoware_JHC/planning模块快速调试手册.md
 * @Description: Do not edit
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
-->

## 数据回放与分析

### 关键话题
录制以下9个关键话题，可疑选择每个话题单独1个log文件或者录制所有话题录制在1个bag包，推荐录制bag方便可视化分析

一条命令，保存9条话题到当前路径下：

 ```cpp
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/root_reference_path --once > root_reference_path.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once > path_with_lane_id.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once > path.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path --once > path_smoother.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory --once > path_optimizer.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/trajectory --once > trajectory_lane_driving.log && \
ros2 topic echo /planning/scenario_planning/scenario_selector/trajectory --once > trajectory_scenario_selector.log && \
ros2 topic echo /planning/scenario_planning/velocity_smoother/trajectory --once > trajectory_velocity_smoother.log && \
ros2 topic echo /planning/scenario_planning/trajectory --once > trajectory_final.log
 ```

录制9条话题的数据的命令:
ros2 bag record \
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/root_reference_path \
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id \
/planning/scenario_planning/lane_driving/behavior_planning/path \
/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path \
/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory \
/planning/scenario_planning/lane_driving/trajectory \
/planning/scenario_planning/scenario_selector/trajectory \
/planning/scenario_planning/velocity_smoother/trajectory \
/planning/scenario_planning/trajectory \
-o path_record

### 所有的planning话题
打包录制所有的planning相关话题

ros2 bag record -e "^/planning($|/)" -o planning_topics_bag

### 可视化分析

1. 启动autoware的planning仿真

ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/num_3-num_8-map/ vehicle_model:=byd_vehicle sensor_model:=byd_sensor_kit

2. 播放录制的rosbag包

ros2 bag play planning_topics_bag -l

3. 查看自己想要的话题和数据

### 一般流程

当测试过程中发现，无法生成路径线，请按照以下步骤进行排查：

1. 检查 planning 模块是否已经启动：

- 在 autoware.launch.xml 中检查 launch_planning 是否设置为true
- ros2 node list | grep "planning", 检查planning模块的节点是否存在

2. 粗定位到planning模块的某一层出现问题：

- planning模块的内部分为6层，层级之间的数据传递是按照串行顺序执行
- 某一层的输入数据正常，但是输出数据异常，则说明该层出现问题，例如，`/planning/mission_planning/route`话题正常，但是 `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`话题没有输出或者异常，则说明是 behavior_path_planning 层出现问题。

| 层级 | 输出话题 |
|------|---------|
| mission_planner | `/planning/mission_planning/route` |
| behavior_path_planning | `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` |
| behavior_velocity_planning | `/planning/scenario_planning/lane_driving/behavior_planning/path` |
| path_smoother | `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path` |
| path_optimizer | `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` |
| motion_velocity_planner | `/planning/scenario_planning/lane_driving/trajectory` |
| scenario_selector | `/planning/scenario_planning/scenario_selector/trajectory` |
| velocity_smoother | `/planning/scenario_planning/velocity_smoother/trajectory` |
| planning_validator | `/planning/scenario_planning/trajectory` |

3. 精定位到某层的某一个组件

- 某个组件生成的路径的话题为 /planning/path_reference/*，例如静态障碍物避让模块生成的路径，话题名为：/planning/path_candidate/static_obstacle_avoidance，查看该话题来检查路径是否正确
- 查看 behavior_path_planner 内部组件的激活状态，可以查看话题 /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state


### 排查绕障模块

1. 查看 static_obstacle_avoidance 绕障组件内部的规划信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/static_obstacle_avoidance --once

- 关键字段: 'avoidance_info''allow_avoidance' 'longitudinal_distance'

2. 查看障碍物信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array --once

*特殊情况*

- 绕障曲线已生成，但是车停止在障碍物前方

由于障碍物不满足th_offset_from_centerline参数要求，障碍物被判定为ambiguous_vehicle可疑车辆，采取wait and see策略，而可疑车辆的规则是需要手动启动避障，所以停车等待手动批准，因此修改参数配置，减小判定阈值，更容易判断为可执行避障的物体，且自动通过审批

查看 th_offset_from_centerline 参数的命令
`ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.parked_vehicle.th_offset_from_centerline 2>/dev/null`

设置策略为自动批准的命令
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.avoidance_for_ambiguous_vehicle.policy "auto"

查看静态障碍物关键参数配置
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance --once 2>/dev/null | grep -E "ratio:|lateral:|clip:|stoppable:|parked|is_on"

### 排查velocity_smoother模块

1. 先查看当前时刻所规划轨迹点上的速度 
/planning/scenario_planning/velocity_smoother/closest_velocity

2. 分别查看使用横向加速度限制/转向角速度限制/优化器优化后的轨迹点。与原始轨迹对比，看是速度平滑的那一部分出现了问题，例如`/planning/scenario_planning/velocity_smoother/debug/trajectory_raw`正常，但是`/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered`异常，说明是横向加速度限制部分有问题

/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered
/planning/scenario_planning/velocity_smoother/debug/trajectory_raw
/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited
/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled

### 排查轨迹速度为0（意外停车）

工作流程：

1. 先确认最终轨迹 `/planning/scenario_planning/trajectory` 是否真的插入 0 速点，如果有那就说明是规划的问题，如果没有那就是下游模块的问题(很大可能是控制部分，建议优先排查)
2. 再对比查看 `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` 和 `/planning/scenario_planning/lane_driving/trajectory`，是否是该模块意外触发了避障组件，导致速度为0
3. 查看 `planning_factors` 和 `/api/planning/velocity_factors`，查看规划模块基于什么原因，选择将当前速度设置为0
4. 再看 obstacle_stop 的 `planning_info`、`debug_markers`、`virtual_walls`，查看避障组件计算得到的障碍物信息，可以查看是否和实际的相同
5. 最后回看感知模块和定位模块的输入，感知模块主要看 occupancy grid 和 **车道线内** 的物体

开发结论判据：

- 只有 motion_velocity_planner 后异常: 插件问题或感知输入问题
- 只有最终 trajectory 异常: velocity_smoother 或限速链路问题

