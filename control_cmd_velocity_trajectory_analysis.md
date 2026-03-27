# /control/command/control_cmd 速度跳变与轨迹突变原因分析

## 1. 现象描述

当前现象有两个：

- `/control/command/control_cmd` 的纵向速度一会儿为 0，一会儿又恢复为非 0
- `/planning/scenario_planning/trajectory` 或其上游轨迹存在突变

这两个现象通常不是孤立的，很多情况下是同一条链路中的状态切换或轨迹重规划共同导致的。

---

## 2. 整体链路

Autoware 中与该问题最相关的链路通常如下：

1. Planning 生成轨迹
2. Velocity Smoother 对轨迹速度进行平滑和限速
3. Trajectory Follower / PID Longitudinal Controller 根据轨迹生成控制命令
4. Vehicle Cmd Gate 对控制命令做最终门控并输出到 `/control/command/control_cmd`

重点 topic 链路：

1. `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
2. `/planning/scenario_planning/lane_driving/behavior_planning/path`
3. `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
4. `/planning/scenario_planning/lane_driving/trajectory`
5. `/planning/scenario_planning/scenario_selector/trajectory`
6. `/planning/scenario_planning/velocity_smoother/trajectory`
7. `/planning/scenario_planning/trajectory`
8. `/control/command/control_cmd`

### 2.1 当前项目实际生效的 launch 与 remap

结合当前仓库中的 launch 配置，这条链路在本项目里不是抽象概念，而是有明确 remap 关系的。

#### Planning 主链路

在 planning 总入口中：

- `scenario_planning` 的输出先进入 `velocity_smoother`
- `planning_validator` 再把 `velocity_smoother` 的输出发布为最终 `/planning/scenario_planning/trajectory`

因此当前项目中最终轨迹的真实关系是：

1. `/planning/scenario_planning/scenario_selector/trajectory`
2. `/planning/scenario_planning/velocity_smoother/trajectory`
3. `/planning/scenario_planning/trajectory`

这里要特别注意：

- `/planning/scenario_planning/trajectory` 不是 `velocity_smoother` 直接输出的最终 topic
- 它是经过 `planning_validator` 后重新发布出来的最终控制输入轨迹

也就是说，如果你看到 `/planning/scenario_planning/velocity_smoother/trajectory` 正常，但 `/planning/scenario_planning/trajectory` 异常，那么问题可能已经进入 validator 之后的链路；反过来如果前者已经异常，根因就在更上游。

#### Lane Driving 子链路

当前项目里 lane driving 这一段的实际 topic 关系是：

1. `behavior_path_planner` 输出 `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
2. `moion planning` 输出 `/planning/scenario_planning/lane_driving/behavior_planning/path`
3. `path_smoother` 输出 `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path`
4. `path_optimizer` 输出 `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
5. `motion_velocity_planner` 基于 `path_optimizer/trajectory` 生成带速度信息的轨迹
6. lane driving 最终输出 `/planning/scenario_planning/lane_driving/trajectory`

因此对你这个项目来说，lane driving 段最值得重点观察的不是单一一个 topic，而是下面这一组：

1. `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
2. `/planning/scenario_planning/lane_driving/behavior_planning/path`
3. `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path`
4. `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
5. `/planning/scenario_planning/lane_driving/trajectory`

这也与当前 [byd/path_save.sh](byd/path_save.sh) 中已经在抓取的 topic 基本一致，说明你当前脚本抓 planning 主链路的方向是对的。

#### Scenario Selector 与 Velocity Smoother 子链路

当前项目中：

- `scenario_selector` 输入：
  - `/planning/scenario_planning/lane_driving/trajectory`
  - `/planning/scenario_planning/parking/trajectory`
- `scenario_selector` 输出：
  - `/planning/scenario_planning/scenario`
  - `/planning/scenario_planning/scenario_selector/trajectory`

- `velocity_smoother` 输入：
  - `/planning/scenario_planning/scenario_selector/trajectory`
  - `/planning/scenario_planning/max_velocity`
  - `/localization/acceleration`
  - `/system/operation_mode/state`
- `velocity_smoother` 输出：
  - `/planning/scenario_planning/velocity_smoother/trajectory`
  - `/planning/scenario_planning/current_max_velocity`

因此如果轨迹在 `lane_driving/trajectory` 正常、而在 `scenario_selector/trajectory` 开始突变，优先看场景切换；如果 `scenario_selector/trajectory` 正常、而 `velocity_smoother/trajectory` 开始突变，优先看 velocity smoother 的 stop approaching、replan 或速度限制逻辑。

#### Control 主链路

当前项目中控制链路的真实 remap 是：

1. `trajectory_follower` 的 `~/input/reference_trajectory` 接到 `/planning/scenario_planning/trajectory`
2. `trajectory_follower` 的 `~/output/control_cmd` remap 到 `/control/trajectory_follower/control_cmd`
3. `vehicle_cmd_gate` 的 `input/auto/control_cmd` 接到 `/control/trajectory_follower/control_cmd`
4. `vehicle_cmd_gate` 的 `output/control_cmd` remap 到 `/control/command/control_cmd`

因此你现在看到的两个 control 侧关键 topic 实际上是：

1. `/control/trajectory_follower/control_cmd`
2. `/control/command/control_cmd`

它们的定位不同：

- `/control/trajectory_follower/control_cmd` 是控制器原始输出
- `/control/command/control_cmd` 是经过 `vehicle_cmd_gate` 之后的最终控制命令

如果前者已经抖动，问题在 controller 或其输入轨迹；如果前者稳定而后者抖动，问题更可能在 `vehicle_cmd_gate` 或模式切换上。

#### 当前项目中的监控重点

仓库里的组件监控配置也说明这两个 topic 被视为正式关键链路：

1. `/planning/scenario_planning/trajectory`
2. `/control/trajectory_follower/control_cmd`
3. `/control/command/control_cmd`

这也意味着，在你当前项目里，排查时必须至少同时对比这三个 topic，不能只看最终的 `/control/command/control_cmd`。

---

## 3. 最可能的根因

### 3.1 PID 纵向控制器状态机来回切换

最优先怀疑的是 PID 纵向控制器内部状态机在 `DRIVE`、`STOPPING`、`STOPPED` 之间来回切换。

典型表现：

- 在 `DRIVE` 状态时，控制器使用轨迹中的目标速度，输出非 0 速度
- 在 `STOPPING` 或 `STOPPED` 状态时，控制器输出 0 或接近 0 的速度

因此一旦状态机震荡，就会直接表现为 `/control/command/control_cmd` 的速度在 0 和非 0 之间切换。

常见触发原因：

- `stop_dist` 在停止阈值附近反复穿越
- 车辆速度在“已停止”的判定阈值附近来回波动
- 开启了“转向收敛前保持停车”逻辑，导致横向未收敛时一直维持 `STOPPED`

这类问题的本质是：控制状态存在迟滞不够或输入量在阈值附近抖动。

### 3.2 转向收敛检查导致反复保持停车

如果启用了类似 `enable_keep_stopped_until_steer_convergence` 的逻辑，那么车辆即使已经具备前进条件，只要转向误差未收敛，也会继续保持停车。

这会带来一种典型现象：

1. 一次循环中判断转向收敛，允许进入 `DRIVE`
2. 下一次循环中转向误差又略微超阈值，回到 `STOPPED`
3. 于是控制速度出现 0 和非 0 交替

如果此时轨迹本身也在更新，就会让用户感觉“轨迹也在跳”。

### 3.3 Velocity Smoother 触发重规划

Velocity Smoother 是规划链路中非常容易引入速度突变的环节。

重点风险点：

- 当实际车速和目标车速差异大于 `replan_vel_deviation` 时，触发重新初始化或重规划
- 轨迹末端通常会被强制设为 0 速度
- 接近停止点时，会在 `stopping_distance` 范围内把速度限制到较低值甚至 0

因此如果车辆正好在停止点附近或当前速度偏差在阈值附近波动，就可能发生：

1. 本周期触发重规划，轨迹速度被重新计算
2. 下一周期又不触发重规划，恢复另一条速度曲线
3. 表现为 trajectory 速度或形状发生突变

### 3.4 Scenario Selector 发生场景切换

如果车辆位于轨迹尾部附近，同时速度接近“已停止”阈值，Scenario Selector 有可能在不同场景之间切换，例如：

- `LANE_DRIVING`
- `PARKING`

一旦场景切换，对应的轨迹源就可能变化，表现为：

- `/planning/scenario_planning/scenario_selector/trajectory` 突变
- 最终 `/planning/scenario_planning/trajectory` 也突变

如果 `is_stopped` 的判断在阈值附近来回变化，场景切换就会抖动。

### 3.5 Motion Velocity Planner 的停车点或障碍物决策闪烁

如果障碍物距离、碰撞预测结果或者停车点插入逻辑在阈值附近不稳定，就会出现：

- 一帧有停车点，目标速度变为 0
- 下一帧停车点消失，目标速度恢复为正常巡航

这会把一个“停车/放行”的抖动直接传导到后续的 trajectory 和 control_cmd。

---

## 4. 为什么会同时出现“轨迹突变”和“control_cmd 跳变”

这两个问题往往是一条链上的前后级联：

1. 上游 planning 模块在停止点、障碍物或场景切换逻辑上发生抖动
2. Velocity Smoother 对速度曲线重新初始化或重规划
3. 最终输出的 `/planning/scenario_planning/trajectory` 出现突变
4. PID Longitudinal Controller 读取到新的轨迹速度后更新控制状态
5. 若又叠加 `STOPPED`/`DRIVE` 状态切换，`/control/command/control_cmd` 进一步表现为 0 和非 0 交替

所以看到这两个现象同时出现时，通常优先从 planning 轨迹是否已经异常入手，而不是只盯 control_cmd。

---

## 5. 建议排查方法

### 5.1 先判断问题是在 planning 还是 control

先看最终轨迹速度是否已经在跳变：

```bash
ros2 topic echo /planning/scenario_planning/trajectory --field points[0].longitudinal_velocity_mps
```

结论判断：

- 如果这里已经在 0 和非 0 之间跳变，问题优先在 planning 链路
- 如果这里稳定，而 `/control/command/control_cmd` 仍然跳变，问题优先在 control 链路

### 5.2 逐级向上游追踪轨迹在哪一级开始突变

建议按下面顺序逐级看速度或轨迹内容：

```bash
ros2 topic echo /planning/scenario_planning/velocity_smoother/trajectory --field points[0].longitudinal_velocity_mps
ros2 topic echo /planning/scenario_planning/scenario_selector/trajectory --field points[0].longitudinal_velocity_mps
ros2 topic echo /planning/scenario_planning/lane_driving/trajectory --field points[0].longitudinal_velocity_mps
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory --field points[0].longitudinal_velocity_mps
```

如果某一级开始出现明显跳变，那么根因通常就在该级或其上游一层。

对于当前项目，更推荐按实际 launch 生效顺序来查：

```bash
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path --once
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory --once
ros2 topic echo /planning/scenario_planning/lane_driving/trajectory --once
ros2 topic echo /planning/scenario_planning/scenario_selector/trajectory --once
ros2 topic echo /planning/scenario_planning/velocity_smoother/trajectory --once
ros2 topic echo /planning/scenario_planning/trajectory --once
```

这组命令和你当前 [byd/path_save.sh](byd/path_save.sh) 的抓取链路是一致的。建议在此基础上再补 control 侧两个 topic：

```bash
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /control/command/control_cmd --once
```

这样可以直接判断抖动是在 planning 侧、trajectory_follower 侧，还是 vehicle_cmd_gate 侧放大的。

### 5.3 查看 PID 纵向控制状态是否在跳

重点确认控制器是否在 `DRIVE`、`STOPPING`、`STOPPED` 之间反复变化。

可关注纵向控制诊断 topic，例如：

```bash
ros2 topic echo /control/trajectory_follower/longitudinal/diagnostic
```

常见状态编码：

- `0 = DRIVE`
- `1 = STOPPING`
- `2 = STOPPED`
- `3 = EMERGENCY`

如果 `control_state` 在 `0` 和 `2` 之间来回跳，那么 control_cmd 速度跳变就基本解释通了。

### 5.4 查看 velocity_smoother 是否在停止点附近反复重规划

建议观察：

```bash
ros2 topic echo /planning/scenario_planning/velocity_smoother/closest_velocity
ros2 topic echo /planning/scenario_planning/velocity_smoother/distance_to_stopline
```

若 `distance_to_stopline` 在某个阈值附近来回变化，而 `closest_velocity` 同步突变，那么问题很可能与 stop approaching 或 replan 初始化有关。

如果开启了 debug trajectory 发布，也建议重点看：

- `debug/trajectory_raw`
- `debug/trajectory_external_velocity_limited`
- `debug/trajectory_lateral_acc_filtered`
- `debug/trajectory_steering_rate_limited`
- `debug/trajectory_time_resampled`
- `debug/forward_filtered_trajectory`
- `debug/backward_filtered_trajectory`
- `debug/merged_filtered_trajectory`

### 5.5 查看 Scenario Selector 是否在切换场景

```bash
ros2 topic echo /planning/scenario_planning/scenario
```

如果场景在 lane driving 和 parking 之间跳，或者在轨迹末端附近发生来回切换，那么 trajectory 突变就很可能来自这里。

### 5.6 查看是否有停车点或障碍物决策闪烁

建议同时查看：

- `/planning/planning_factors/motion_velocity_planner`
- `/planning/planning_factors/obstacle_stop`
- `/api/planning/velocity_factors`
- motion velocity planner 的 obstacle stop / obstacle cruise debug topics

如果 stop reason 一会儿出现、一会儿消失，说明根因可能在障碍物判定或停车点生成逻辑。

---

## 6. 建议录包 topic

为了离线分析，建议一次性录制以下 topic：

```bash
ros2 bag record \
  /control/trajectory_follower/control_cmd \
  /control/command/control_cmd \
  /control/trajectory_follower/longitudinal/diagnostic \
  /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id \
  /planning/scenario_planning/lane_driving/behavior_planning/path \
  /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path \
  /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory \
  /planning/scenario_planning/trajectory \
  /planning/scenario_planning/velocity_smoother/trajectory \
  /planning/scenario_planning/scenario_selector/trajectory \
  /planning/scenario_planning/lane_driving/trajectory \
  /planning/scenario_planning/velocity_smoother/closest_velocity \
  /planning/scenario_planning/velocity_smoother/distance_to_stopline \
  /planning/scenario_planning/scenario \
  /localization/kinematic_state
```

如果还怀疑是 motion velocity planner 的停车点问题，可以额外录制：

- `/planning/planning_factors/motion_velocity_planner`
- `/planning/planning_factors/obstacle_stop`
- `/api/planning/velocity_factors`
- obstacle stop 相关 debug topic

---

## 7. 优先建议检查的参数

### 7.1 PID Longitudinal Controller

重点参数：

- `drive_state_stop_dist`
- `stopping_state_stop_dist`
- `stopped_state_entry_vel`
- `stopped_state_entry_duration_time`
- `enable_keep_stopped_until_steer_convergence`

建议：

- 拉大 `drive_state_stop_dist` 与 `stopping_state_stop_dist` 的差距，增加迟滞
- 如果确认是转向收敛逻辑引起，可临时关闭 `enable_keep_stopped_until_steer_convergence` 做对比实验

### 7.2 Velocity Smoother

重点参数：

- `replan_vel_deviation`
- `stopping_distance`
- `stopping_velocity`

建议：

- 适当调大 `replan_vel_deviation`，减少在阈值附近反复重规划
- 检查 `stopping_distance` 是否过大导致离停止点还很远就反复限速

### 7.3 Scenario Selector

重点参数：

- `th_stopped_velocity_mps`
- `th_arrived_distance_m`
- `lane_stopping_timeout_s`

建议：

- 减小 `th_stopped_velocity_mps`，避免低速微小波动被误判为停车
- 增大 `lane_stopping_timeout_s`，增强场景切换迟滞

### 7.4 Motion Velocity Planner / Obstacle Stop

重点检查：

- 停车距离阈值
- 障碍物距离判定阈值
- 感知目标是否稳定

建议：

- 如果停车点频繁插入/删除，先确认感知输入和 stop obstacle 判定是否抖动

---

## 8. 最终结论

从经验上看，这类问题最常见的根因排序如下：

1. PID 纵向控制器状态机在 `DRIVE` 和 `STOPPED` 之间震荡
2. Velocity Smoother 在停止点附近触发重规划或速度重置
3. Scenario Selector 在场景之间切换导致 trajectory 源变化
4. Motion Velocity Planner 的停车点或障碍物决策闪烁

最有效的排查路径不是只看 `/control/command/control_cmd`，而是：

1. 先确认 `/planning/scenario_planning/trajectory` 是否已异常
2. 再逐级往上游定位是哪个 planning 模块先发生跳变
3. 同时检查 PID longitudinal controller 的 `control_state` 是否在跳

如果 trajectory 已经突变，那么 control_cmd 的跳变大概率只是下游表现；如果 trajectory 稳定但 control_cmd 仍跳，则重点看 PID 状态机和 vehicle_cmd_gate。