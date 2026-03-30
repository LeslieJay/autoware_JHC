### 

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

3. 