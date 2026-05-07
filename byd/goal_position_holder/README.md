# goal_position_holder

末段位置闭环节点：在车辆接近 `goal_pose` 末点时**同时接管纵向和横向控制**：

- **纵向**：剩余纵向位移 PD + creep，把 `s_long` 压到 **±2 cm**
- **横向**：横向偏差 / 航向偏差 P 控制，输出 `steering_tire_angle`，把 `s_lat` 压到 **±2 cm**

弥补 trajectory_follower 速度环死区导致的"提前停 / 冲过头"，以及末端横向跟踪的稳态误差。`enable_lateral=false` 时退化为只接管纵向（与旧版行为一致）。

---

## 1. 解决了什么问题

Autoware 默认的 trajectory_follower（PID + Pure Pursuit/MPC）在末段会出现两类问题：

| 现象 | 原因 |
|---|---|
| 提前 5–30 cm 停下 | velocity_smoother 输出末点 v=0，PID 在 ~0.05 m/s 以下 ki/kp 失效，加上制动死区，车辆静摩擦把车摁住 |
| 冲过 goal 几 cm~几十 cm | 末段减速段太短，QP 无法平滑到 0；或位置环不存在，PID 只跟"速度"不跟"位置" |
| ArrivedGoal 提前触发后再也修正不了 | `mission_planner.arrival_check_distance` 一旦命中，状态机切走，控制器停止跟踪 |

本节点在控制管线**末端**（`trajectory_follower → vehicle_cmd_gate` 中间）插一个轻量的位置 PD：只在最后 3 m / 1 m/s 内激活，用 `goal_pose` 直接做位置闭环，不依赖速度规划。

---

## 2. 工作原理

### 2.1 拓扑

```
trajectory_follower
    │  /control/trajectory_follower/control_cmd
    ▼
goal_position_holder            ← 本节点
    │  /control/trajectory_follower/control_cmd_holded
    ▼
vehicle_cmd_gate
```

> 接入方式：在 `control.launch.xml` 把 `vehicle_cmd_gate` 的 `input/auto/control_cmd` 从 `/control/trajectory_follower/control_cmd` 改成 `/control/trajectory_follower/control_cmd_holded` 即可。

### 2.2 状态机

| 模式 | 进入条件 | 行为 |
|---|---|---|
| **PASSTHROUGH**（默认） | 任一不满足 HOLD 条件 | 透传上游 `Control` 消息 |
| **HOLD** | ① 规划末点距 goal `< traj_to_goal_max` (0.5 m)<br>② ego 距 goal 末点（沿 goal_yaw 投影） `< activation_distance` (3 m)<br>③ \|ego_v\| `< activation_speed` (1 m/s) | 用 PD 重写 `longitudinal.{velocity, acceleration}`，横向不变 |

### 2.3 HOLD 内部分支

在 goal 系下计算：
- `s_long = (ego - goal) · [cos goal_yaw, sin goal_yaw]`，`s_long > 0` 表示已越过 goal
- `s_lat  = (ego - goal) · [-sin goal_yaw, cos goal_yaw]`，`s_lat  > 0` 表示 ego 在 goal 方向左侧
- `yaw_err = normalize(ego_yaw - goal_yaw)`

横向（HOLD 内统一应用，符号：steering>0 = 左转）：
```text
steer = clamp(-k_lat * s_lat - k_yaw * yaw_err, ±max_steer)
```

纵向：
```text
long_converged = |s_long| < tol_pos AND |ego_v| < tol_vel
lat_converged  = |s_lat|  < tol_lat AND |yaw_err| < tol_yaw

if long_converged AND lat_converged:
    # 全收敛 → 死刹 + 方向盘回正（避免静止打方向）
    v_cmd = 0,  a_cmd = brake_acc,  steer = 0

elif |ego_v| < creep_speed_thresh:
    # 蠕行：车几乎停了，但纵向或横向还没到位 → 推一脚
    # residual = -s_long + (横向未收敛 ? lat_to_long_weight·|s_lat| : 0)
    # 借助小幅前进让 steer 实际作用，把 s_lat 也修正到 tol_lat
    if residual > tol_pos:
        v_cmd = creep_v,  a_cmd = creep_acc   # （steer 见上）
    else:
        v_cmd = 0,        a_cmd = brake_acc   # 不允许倒车

else:
    # 标准纵向 PD
    a_cmd = clamp(-kp * s_long - kv * ego_v, a_min, a_max)
    v_cmd = clamp(ego_v + a_cmd * dt_lookahead, 0, v_max)
    if s_long > 0:               # 已越过 goal → 强刹
        v_cmd = 0,  a_cmd = brake_acc
```

要点：
- **只能停**，不能倒车（`v_cmd ≥ 0`）
- 越过 goal 立刻 `brake_acc`，不靠 PD 慢慢回拉
- `creep` 是关键：单纯的 PD 在 v→0 时计算量也→0；creep 用一个固定小速度推过去，并让横向 steer 在这一段小幅运动中收敛 `s_lat`
- 全收敛后 steer 强制回 0，避免静止打方向损伤转向系统
- `enable=false` 时整节点降级为纯透传；`enable_lateral=false` 时只接管纵向

---

## 3. 话题

### 订阅

| 内部名 | 默认 remap | 类型 | 说明 |
|---|---|---|---|
| `~/input/control_cmd` | `/control/trajectory_follower/control_cmd` | `autoware_control_msgs/Control` | 上游控制指令（被覆盖/透传） |
| `~/input/kinematic_state` | `/localization/kinematic_state` | `nav_msgs/Odometry` | ego 位置 + 速度 |
| `~/input/trajectory` | `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/Trajectory` | 用末点判断"规划已收敛到 goal" |
| `~/input/goal_pose` | `/planning/mission_planning/echo_back_goal_pose` | `geometry_msgs/PoseStamped` | 当前 goal（QoS=transient_local） |

### 发布

| 内部名 | 默认 remap | 类型 |
|---|---|---|
| `~/output/control_cmd` | `/control/trajectory_follower/control_cmd_holded` | `autoware_control_msgs/Control` |

---

## 4. 参数

| 名称 | 默认 | 单位 | 含义 |
|---|---|---|---|
| `enable` | `true` | bool | `false` 时整节点透传 |
| `activation_distance` | `3.0` | m | ego 到 goal 的纵向投影距离 ≤ 此值才进入 HOLD |
| `activation_speed` | `1.0` | m/s | ego 速度 ≤ 此值才进入 HOLD |
| `traj_to_goal_max` | `0.5` | m | 规划末点离 goal ≤ 此值才认为"规划已收敛" |
| `kp` | `0.8` | 1/s² | 位置比例增益（`a = -kp · s_long - kv · v`） |
| `kv` | `3.0` | 1/s | 速度阻尼增益 |
| `a_min` / `a_max` | `-2.0 / 0.5` | m/s² | PD 输出加速度饱和 |
| `v_max` | `1.0` | m/s | PD 输出速度上限（HOLD 阶段绝不会超） |
| `dt_lookahead` | `0.1` | s | 由 a 估 v 的预瞄步长 |
| `tol_pos` | `0.02` | m | 位置容差，进入即死刹 |
| `tol_vel` | `0.05` | m/s | 速度容差，配合 tol_pos 一起判定"到位" |
| `brake_acc` | `-1.5` | m/s² | 死刹时的恒定减速度 |
| `creep_speed_thresh` | `0.03` | m/s | 低于此速度激活 creep |
| `creep_v` | `0.08` | m/s | creep 输出速度 |
| `creep_acc` | `0.15` | m/s² | creep 输出加速度 |
| `enable_lateral` | `true` | bool | `false` 时只接管纵向，横向透传上游 |
| `k_lat` | `0.6` | rad/m | 横向偏差比例增益（`δ = -k_lat·s_lat - k_yaw·yaw_err`） |
| `k_yaw` | `1.0` | rad/rad | 航向偏差比例增益 |
| `max_steer` | `0.6` | rad | 输出方向盘饱和（≈34°） |
| `tol_lat` | `0.02` | m | 横向位置容差 |
| `tol_yaw` | `0.02` | rad | 航向容差（≈1.15°） |
| `lat_to_long_weight` | `1.0` | – | 蠕行决策中把横向误差折算到纵向的权重 |

调参建议：
- **纵向冲过头** → 调大 `kp`（0.8 → 1.2）或调大 `kv`
- **纵向离 goal 还差 5–10 cm 静止** → 调大 `creep_v` (0.08 → 0.15) 或调小 `creep_speed_thresh`
- **HOLD 段纵向抖动** → 调小 `kp`、调大 `kv`、缩小 `dt_lookahead`
- **横向偏差收敛慢** → 调大 `k_lat`（0.6 → 1.0）、`k_yaw`（1.0 → 1.5）；必要时调大 `creep_v` 让车多走一点
- **横向抖动 / 末段方向盘晃** → 调小 `k_lat`、调小 `max_steer`、确认 `tol_yaw` 不要太小
- **想完全关掉横向做 A/B 对比** → `enable_lateral:=false`
- **想完全关掉做 A/B 对比** → `goal_position_holder_enable:=false`

---

## 5. 编译 & 运行

```bash
cd /media/f/nvme_storage/autoware
colcon build --symlink-install --packages-select goal_position_holder \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 单独启动
ros2 launch goal_position_holder goal_position_holder.launch.xml

# 关闭（透传模式）
ros2 launch goal_position_holder goal_position_holder.launch.xml \
  goal_position_holder_enable:=false
```

要让该节点真正生效，还要修改 `autoware_launch` 里 `vehicle_cmd_gate` 的输入：

```xml
<!-- control.launch.xml 中找到 vehicle_cmd_gate -->
<remap from="input/auto/control_cmd" to="/control/trajectory_follower/control_cmd_holded"/>
```

并在 `control.launch.xml` 末尾追加：

```xml
<include file="$(find-pkg-share goal_position_holder)/launch/goal_position_holder.launch.xml"/>
```

---

## 6. 调试

```bash
# 看 HOLD 进出日志
ros2 node info /goal_position_holder
ros2 topic echo /control/trajectory_follower/control_cmd_holded --field longitudinal

# 录制对比 bag（透传 vs 接管）
ros2 bag record -e "^/control/.*|^/localization/kinematic_state$|^/planning/mission_planning/echo_back_goal_pose$"
```

进入 HOLD 时会打印（每次 enter/exit 一次）：
```
[HOLD] enter: 末段位置闭环接管 (s_long=-2.341m v=0.823)
[HOLD] exit:  退出末段位置闭环
```

---

## 7. 已知限制

- **横向控制基于 P，无前馈**：极端低速 + 大初始横向误差时收敛较慢，依赖 creep 蠕行配合
- **不处理倒车场景**：HOLD 内 `v_cmd ≥ 0` 强约束，倒车泊车需禁用本节点
- **依赖 echo_back_goal_pose**：如果 mission_planner 没发该 topic，HOLD 永远不会激活，节点等同透传
- **goal_yaw 用四元数 (z,w) 简化解析**，假设车辆在地面（roll/pitch≈0）；非平地需替换 `yaw_from_quat`
- **不参与 emergency**：紧急制动通路不经过本节点，安全行为不受影响
