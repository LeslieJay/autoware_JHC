# autoware 所有配置文件的存放目录

1. 本目录下的配置文件才是最终生效的配置
2. 某些目录下的 common.yaml的参数会覆盖子目录下的参数(max_vel会覆盖max_velocity)
3. autoware中默认参数是基于乘用车，考虑高速/复杂交通规则等来设置的，我们需要基于户外无人牵引车的实际运行场景(低速/封闭园区内道路/安全第一)来修改参数

## 各组件的推荐参数

### mission_planning

### preset

- 关闭用不上的节点
- 必须使用的节点，调参数，改逻辑

- *behavior path modules*:

1. side shift（关闭）  用于接受远程（remote control）发送的路径偏移指令
2. avoidance by lane change （必须） 换道避障，实际继承于 normal_lane_change
3. bidirectional_traffic_module （关闭）专门处理双向都可行驶通过的单车道情况，保证车行驶在右侧
4. dynamic_obstacle_avoidance_module (暂时关闭) 避让动态物体，减小复杂度
5. externalRequestLaneChangeRightModuleManager （关闭）接受外部发送的变道指令
6. goal planner （必须） 接近终点时触发，保证终点精确位姿，
7. lane change （必须）正常的换道功能，
8. sampling planner （关闭）
9. start planner （不确定）不开的话，如果从路边起步，好像靠control也可以走到路中间，待测试
10. static obstacle avoidance （必须开）同车道绕障 

- *motion velocity planner modules*:

1. boundary_departure_prevention_module (关闭)
2. dynamic_obstacle_stop_module (必须) 规避动态障碍物
3. obstacle_cruise_module (必须) 在障碍物后方巡游
4. obstacle_slow_down (必须) 前方有障碍物时降低速度
5. velocity_limit (必须) 在转弯时降低速度
6. out_of_lane (必须) 在存在跑出道路的情况下，降低速度或者停止
7. road_user_stop(必须) 检测到行人或者车辆时，降低速度
8. run_out (必须) 添加减速点和停止点，以防止与朝自车行驶路径方向运动的目标物体发生碰撞

### common

1. max_vel: 全局速度限制，优先级最高
2. normal/limit: 正常行驶/极限情况下的加速度/加加速度限制

### velocity_smoother

- 影响弯道速度的参数

1. enable_lateral_acc_limit/enable_steering_rate_limit: 分别通过横向加速度/转向角速率限制曲率不为0处的速度

2. curvature_calculation_distance: 曲率计算采用的距离的路径点

3. decel_distance_before_curve: 入弯前多长距离开始减速

4. min_curve_velocity: 弯道内的最小速度

5. max_steering_angle_rate: 最大转向速度上限越高，弯道内所允许的速度越大

- 影响轨迹的形状

1. extract_ahead_dist/extract_behind_dist: 影响轨迹的长度

2. resampling parameters for optimization: 影响轨迹点上的速度平滑优化

3. resampling parameters for post process: 后处理重采样参数 

4. extract_dist 影响最终输出路径的长度 两个trajectory_length只会影响中间过程的结果

5. JerkFiltered.param 影响优化器求解结果，想更平滑提高 jerk_weight 值，增加jerk权重

### static_obstacle_avoidance

1. target_object

target_object:
   car:
      th_moving_speed: 0.5                          # [m/s] Disabled forclosed park - not applicable
      th_moving_time: 10.0                          # [s]
      longitudinal_margin: 0.0                      # [m]
      lateral_margin:
         soft_margin: 0.0                            # [m]
         hard_margin: 0.0                            # [m]
         hard_margin_for_parked_vehicle: 0.0         # [m]
      max_expand_ratio: 0.0                         # [-] FOR DEVELOPER
      envelope_buffer_margin: 0.0                   # [m] FOR DEVELOPER
      th_error_eclipse_long_radius : 0.6            # [m]

- th_moving_speed/th_moving_time: 判断物体是否静止的阈值

- longitudinal_margin/lateral_margin: 横向和纵向的安全余量

- envelope_buffer_margin: 物体的包络多边形余量

- max_expand_ratio: 扩展比例系数，间接影响 avoid_margin

- th_error_eclipse_long_radius: 

- lower/upper_distance_for_polygon_expansion: 包络多边形扩展的上下限，和max_expand_ratio一起影响 avoid_margin

2. target_filtering 目标物体过滤

- target_type: 执行绕障操作的目标物体类型

- detection_area.static:

- detection_area.*_distance: 检测障碍物的距离

- safety_check: 安全检查

3. avoidance 执行绕障操作的参数

- lateral.th_avoid_execution 执行避让操作的阈值

- lateral.th_small_shift_length 最小偏移长度，避免频繁生成绕障曲线

- soft/hard_drivable_bound_margin 软边界安全距离，影响曲线能偏移的最大范围,硬边界安全距离，绕障路径不能超出该限制

- *max_left_shift_length* 最大右/左偏移量，决定曲线横向移动极限

- longitudinal.min_prepare_time 准备时间影响准备长度

- min_prepare_distance 一起影响准备长度

- min/buf_slow_down_speed 影响减速点的速度

- constraints.lateral/longitudinal 影响偏移曲线上轨迹点的速度

### behavior_path_planner

1. backward_path_length/forward_path_length: 最终决定路径长度的参数

2. input_path_interval/output_path_interval: 输入输出path的采样间隔

## 调试过程中的问题

1. 弯道速度太慢

   文件：`planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml`

   | 参数 | 原值 | 修改值 | 说明 |
   |---|---|---|---|
   | `max_lateral_accel` | `0.3` | `0.5` | 提高弯道允许的横向加速度，车辆在弯中可以跑更快 |
   | `min_curve_velocity` | `0.5` | `1.0` | 弯道内的最小速度；若弯道速度一直卡在某个值，很可能是被此参数截断 |

2. 刹车不平滑

   文件1：`planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml`

   | 参数 | 原值 | 修改值 | 说明 |
   |---|---|---|---|
   | `min_decel_for_lateral_acc_lim_filter` | `-5.0` | `-2.5` | 限制弯道减速时的最大减速度，避免急刹 |

   文件2：`planning/scenario_planning/common/common.param.yaml`

   | 参数 | 原值 | 修改值 | 说明 |
   |---|---|---|---|
   | `normal.min_acc` | `-2.5` | `-2.0` | 降低正常行驶最大减速度，刹车更柔和 |
   | `normal.min_jerk` | `-2.5` | `-1.5` | 减小加减速变化率，刹车过渡更平顺 |
   | `normal.max_jerk` | `2.5` | `1.5` | 减小加速变化率，加速过渡更平顺 |

   文件3：`planning/scenario_planning/common/autoware_velocity_smoother/JerkFiltered.param.yaml`

   | 参数 | 原值 | 修改值 | 说明 |
   |---|---|---|---|
   | `jerk_weight` | `10.0` | `20.0` | 优化器更注重平顺性，减少速度曲线的抖动 |

   > 注意：`min_jerk`/`max_jerk` 调小后刹车/加速更柔和，但响应会略有延迟。若响应感觉过于迟缓，可将值回调至 `±2.0`。

3. 进弯前速度太快

   文件：`planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml`

   | 参数 | 原值 | 修改值 | 说明 |
   |---|---|---|---|
   | `decel_distance_before_curve` | `20.0 m` | `35.0 m` | 提前更长距离开始减速进弯，留出足够的缓冲距离 |