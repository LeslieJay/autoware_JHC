<!--
 * @Author: leslie leslie@byd.com
 * @Date: 2026-02-26 10:14:33
 * @LastEditors: wei.canming
 * @LastEditTime: 2026-04-08 16:17:50
 * @FilePath: /autoware_JHC/src/autoware_JHC/core/autoware_core/planning/autoware_velocity_smoother/velocity_smoother_note.md
 * @Description: 
-->


### main function flowchart

1. 主函数 onCurrentTrajectory
1.1 接收里程计/加速度/外部速度限制/操作模式等数据 
1.2 将输入轨迹转为轨迹点队列 // convertToTrajectoryPointArray
1.3 计算当前位置在上一输出轨迹中的索引
1.4 计算外部速度限制并更新
1.5 计算轨迹点上的速度 // calcTrajectoryVelocity(input_points)
1.6 重采样轨迹点 // resampleTrajectory
1.7 发布轨迹点 // publishTrajectory(output_resampled)

2. calcTrajectoryVelocity
2.1 轨迹点中距离当前位置最近点的索引 //findNearestIndexFromEgo
2.2 在当前索引前后提取一定长度的路径 // extractPathAroundIndex
2.3 如果有停止点，则将距离停止点stopping_distance距离内的点所有速度设为stopping_velocity // applyStopApproachingVelocity
2.4 平滑速度 // smoothVelocity

3. smoothVelocity
3.1 根据当前车速、上一帧规划结果等，决定优化的初始速度/加速度 // calcInitialMotion(input, input_closest)
3.2 使用横向加速度滤波限制速度，在弯道处降低速度，防止横向加速度超限
3.3 使用转向角速率限制速度
3.4 重采样轨迹
3.5 设置优化器约束 （setMaxAccel，setMaxJerk）
3.6 优化求解+后处理

### 离当前位置最近点的最大速度

～/closest_max_velocity

### 横向加速度过滤限制速度  applyLateralAccelerationFilter

1. 设置轨迹点间隔，重采样轨迹点
2. 计算每个轨迹点曲率计算需要用到的索引距离
3. 根据索引距离计算曲率
4. 选择轨迹点i处前后距离内的曲率
5. 选择i点处一段距离内的最大曲率作为该点的速度限制曲率
6. 根据曲率和横向加速度限制速度

latacc_min_vel_arr 车辆动力学所允许的每个点最小的速度

### 转向速率限制  applySteeringRateLimit

1. 

### 动态修改参数

/planning/scenario_planning/velocity_smoother:

normal.max_acc normal.max_jerk normal.min_acc max_velocity

/control/trajectory_follower/controller_node_exe

/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner 

/planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/trajectory

关闭转向角速率限制
ros2 param set /planning/scenario_planning/velocity_smoother enable_steering_rate_limit false

### 测试问题记录

测试过程中出现的问题及解决方法

1. 速度在正常和全0之间反复跳变

- 根本原因：replan_vel_deviation 参数过小触发正反馈振荡

1.第一次规划，轨迹起点速度engage_vel=0.8
2.第二次规划，此时desired_velocity~=0.98，但此时的vehicle_speed=0.0，两者之间的差值大于replan_vel_deviation，触发LARGE_DEVIATION_REPLAN，初始速度被强制用当前车速（0）重新规划，输出全0轨迹，可以解释初始速度为0,为什么会全0？
3.上一帧轨迹全0，desired_vel=0，重新进入ENGAGING，轨迹又被engage_velocity拉起，ego点速度又大于0.8

2. 速度非常小

- 速度曲线分析

点1-50:    0.505 m/s (初始低速阶段 - 50个点)
点51-61:   0.505 → 2.0 m/s (加速阶段 - 11个点)
点62-82:   2.0 m/s (恒定巡航 - 21个点)
点83-85:   2.0 → 0.0 m/s (减速停车 - 3个点)

### 用来debug的话题名称

- 选择发布这些debug话题

文件路径: `src/universe/autoware_universe/launch/tier4_planning_launch/launch/scenario_planning/scenario_planning.launch.xml`

```
<param name="publish_debug_trajs" value="true"/>
```
- 数据传输方向

`onCurrentTrajectory` → `calcTrajectoryVelocity` → `smoothVelocity`

| 顺序 | Topic | 代码位置 | 含义 |
|:---:|---|---|---|
| ① | `debug/trajectory_raw` | node.cpp:599 | `extractPathAroundIndex` 之后，去除重叠点+提取自车附近路径，**速度未改变** |
| ② | `debug/trajectory_external_velocity_limited` | node.cpp:615 | `applyExternalVelocityLimit` + `applyStopApproachingVelocity` 之后 |
| ③ | `debug/trajectory_lateral_acc_filtered` | node.cpp:719 | 横向加速度滤波（`applyLateralAccelerationFilter`）之后 |
| ④ | `debug/trajectory_steering_rate_limited` | node.cpp:729 | 转向角速率限制（`applySteeringRateLimit`）之后 |
| ⑤ | `debug/trajectory_time_resampled` | node.cpp:724 | 基于自车速度重采样（`resampleTrajectory`）之后，也是进入 QP 求解的输入 |
| ⑥ | `debug/forward_filtered_trajectory` | node.cpp:1025 | QP 求解器前向 pass 结果 |
| ⑦ | `debug/backward_filtered_trajectory` | node.cpp:1026 | QP 求解器后向 pass 结果 |
| ⑧ | `debug/merged_filtered_trajectory` | node.cpp:1027 | 前向+后向合并后结果，`smoothVelocity` 的实际输出 |
| ⑨ | `trajectory` | node.cpp:311 | `output_resampled`，经过**最终后处理重采样**（`post_resample_param`）后发布 |
| ⑩ | `distance_to_stopline` | node.cpp:808 | 基于 `output`（未后处理重采样）计算停车距离 |

> 注意：`trajectory_time_resampled`（line 724）的 publish 语句在代码中写在 `trajectory_steering_rate_limited`（line 729）之前，但**实际处理顺序**是 lateral_acc → steering_rate → time_resample → QP，debug publish 代码顺序与处理顺序有轻微不一致，不影响数据含义。

- 排查 `/trajectory` 异常的步骤（逐步对比以下话题）

1. **`trajectory_raw` 异常** → 问题在输入轨迹本身（上游节点）
2. **`trajectory_external_velocity_limited` 开始异常** → 外部速度限制 / 停止接近速度逻辑问题
3. **`trajectory_lateral_acc_filtered` 开始异常** → 横向加速度滤波问题（曲率计算 / `latacc_min_vel_arr`）
4. **`trajectory_steering_rate_limited` 开始异常** → 转向角速率限制过激
5. **`trajectory_time_resampled` 开始异常** → 重采样参数问题
6. **`merged_filtered_trajectory` 开始异常** → QP 优化求解失败（检查 `max_acc`/`max_jerk`/`initial_motion` 参数）
7. **`merged_filtered_trajectory` 正常但 `/trajectory` 异常** → 后处理重采样（`post_resample_param`）或 `insertBehindVelocity` 逻辑问题
