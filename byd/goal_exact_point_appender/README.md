# goal_exact_point_appender

把 `velocity_smoother` 输出的 trajectory **末点替换 / 追加成 goal_pose 的精确 (x, y, yaw)**，消除 planning 链路因离散化（`output_resolution ~0.5 m` + `resample_ds ~0.1 m`）造成的末点偏移。

与 [`goal_position_holder`](../goal_position_holder/README.md) 配合使用：
- 本节点修轨迹（让控制器的"目标"就是 goal 本身）
- `goal_position_holder` 修控制（让 ego 的"实际位置"压到 goal 附近）

---

## 1. 拓扑

```
velocity_smoother
    │  /planning/scenario_planning/trajectory
    ▼
goal_exact_point_appender                ← 本节点
    │  /planning/scenario_planning/trajectory_goal_aligned
    ▼
trajectory_follower
    │  /control/trajectory_follower/control_cmd
    ▼
goal_position_holder
    │  /control/trajectory_follower/control_cmd_holded
    ▼
vehicle_cmd_gate
```

---

## 2. 行为

| 条件 | 行为 |
|---|---|
| `enable=false` 或 没收到 goal_pose 或 trajectory 为空 | 透传 |
| 末点距 goal `>` `activation_distance` (默认 0.5 m) | 透传（避免远距离瞎插值） |
| 末点距 goal `≤` `activation_distance` 且 `strategy=replace` | **末点 pose 直接替换为 goal**，速度强置 0 |
| 同上但 `strategy=insert` | **在末点后追加一个 goal 点**，速度=0（保留原末点用于平滑） |

> **推荐 `replace`**：不会破坏 trajectory_follower 的 lookahead 计算；`insert` 只在末点和 goal 之间还有 dm 级跨度且需要保留原末点速度信息时使用。

---

## 3. 话题与参数

订阅：

| 内部名 | 默认 remap | 类型 |
|---|---|---|
| `~/input/trajectory` | `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/Trajectory` |
| `~/input/goal_pose` | `/planning/mission_planning/echo_back_goal_pose` | `geometry_msgs/PoseStamped`（QoS=transient_local） |

发布：

| 内部名 | 默认 remap |
|---|---|
| `~/output/trajectory` | `/planning/scenario_planning/trajectory_goal_aligned` |

参数：

| 名称 | 默认 | 含义 |
|---|---|---|
| `enable` | `true` | `false` 时透传 |
| `activation_distance` | `0.5` m | 末点距 goal 阈值，超出不动 |
| `strategy` | `"replace"` | `"replace"` 或 `"insert"` |
| `force_zero_velocity_at_goal` | `true` | 末点速度/加速度归 0 |

---

## 4. 编译 & 启动

```bash
cd /media/f/nvme_storage/autoware
colcon build --symlink-install --packages-select goal_exact_point_appender \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 单独启动
ros2 launch goal_exact_point_appender goal_exact_point_appender.launch.xml

# 透传模式做 A/B 对比
ros2 launch goal_exact_point_appender goal_exact_point_appender.launch.xml \
  goal_exact_point_appender_enable:=false
```

---

## 5. 接入 Autoware 控制管线

要让 `trajectory_follower` 真正读到对齐后的轨迹，把它的输入 remap 到本节点输出：

在 `control.launch.xml` 中找到 `trajectory_follower` 的 include，添加：

```xml
<remap from="input/reference_trajectory"
       to="/planning/scenario_planning/trajectory_goal_aligned"/>
```

并启动本节点：

```xml
<include file="$(find-pkg-share goal_exact_point_appender)/launch/goal_exact_point_appender.launch.xml"/>
```

> 实际 topic 名以你工程里 trajectory_follower 的接口为准（不同版本叫 `input/reference_trajectory` 或 `~/input/trajectory`）。

---

## 6. 与 goal_position_holder 的分工

| 误差来源 | 解决方 |
|---|---|
| **planning 离散化**（末点 9.5 vs goal 10.0） | **本节点** |
| **mission_planner 把 goal 吸附到 lanelet 中心线** | `enable_correct_goal_pose: false` |
| **控制器跟轨迹序列、不跟绝对位置** | goal_position_holder |
| **车辆模型延迟 + 速度死区滑行** | goal_position_holder（位置 PD + creep）|
| **状态机阈值容忍带** | mission_planner 调小 `arrival_check_distance` |

三件套组合：①yaml 关掉 goal 对齐 + ②本节点修轨迹 + ③goal_position_holder 修控制 → 实测可稳定 < 5 mm。

---

## 7. 已知限制

- **goal_pose 的 z/roll/pitch 直接照搬**：假设 goal 在地面，非平地需自行处理
- **不参与碰撞检查**：插入/替换的末点假设 goal 已在可行驶区域内（mission_planner 已校验）
- **不修中间点**：只动末点，中间离散化误差不消除（但末点是停车精度唯一关心的点）
- **首次激活会打印一次 `[ALIGN] 末点对齐生效`**，之后不再刷屏
