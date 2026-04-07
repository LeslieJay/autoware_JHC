# autoware 所有配置文件的存放目录

1. 本目录下的配置文件才是最终生效的配置
2. 某些目录下的 common.yaml的参数会覆盖子目录下的参数(max_vel会覆盖max_velocity)
3. autoware中默认参数是基于乘用车，考虑高速/复杂交通规则等来设置的，我们需要基于户外无人牵引车的实际运行场景(低速/封闭园区内道路/安全第一)来修改参数

## 各组件的推荐参数

### mission_planning

### velocity_smoother

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