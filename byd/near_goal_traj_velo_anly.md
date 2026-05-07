# 到达终点段 速度/轨迹分析 Plan

利用录制的 rosbag2 包（`.bag/` 目录）和同次运行的 `.log` 文件，对到达终点（goal）前后进行速度与 velocity_smoother 各阶段轨迹分析。

> 输出目录与单文件命名 **必须** 与参考模板 `log/0424/planning_control_localization_05.bag_logs/` 一模一样。

---

## 0. 总览

分析流程（时间戳精确到 sec）：
1. 从 `.log` 提取 `goal_pose` 和到达终点时刻 `arrive_time`
2. 提取 `arrive_time ± 10s` 的控制指令（`control_cmd`），并定位实际刹停时刻 `zero_vel_time`（之后所有 `velocity` 均为 0）
3. 在 `arrive_time ± 10s` 的 control_cmd 窗口内找 `|dv| > 1.0 m/s` 的速度突变点，取突变后一帧的 `header_stamp`，`zero_vel_sec = floor(jump_stamp)`
4. 以 `zero_vel_sec` 为 target_sec 导出 velocity_smoother 9 个内部 topic
5. 合成 stage_overlay 单图汇总

---

## 1. 输入 / 输出 / 环境

### 输入
- `BAG_DIR`：rosbag2 目录，含 `metadata.yaml` + `*_0.db3`，例如 `log/0428/planning_control_bag_08`
- `LOG_FILE`：同次运行日志，例如 `log/0428/autoware_08.log`
- `OUT_DIR`：默认 `${BAG_DIR%/}_logs`（与 0424 模板同名风格）

### 环境
```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
```
- Python + `rosbag2_py.SequentialReader` + `rclpy.serialization.deserialize_message`
- 读 bag 时务必 `StorageFilter(topics=[...])` 仅筛需要的 topic
- 时间字段统一使用 `header.stamp`（不是 bag 接收时间 `t`）

### 关键 topic
| 用途 | topic | 类型 |
| --- | --- | --- |
| 终点 | `/planning/mission_planning/goal`（备选 `route.goal_pose`） | `geometry_msgs/PoseStamped` |
| 控制指令 | `/control/command/control_cmd` | `autoware_control_msgs/Control`，速度字段 `longitudinal.velocity` |
| 实际车速/位姿 | `/localization/kinematic_state` | `nav_msgs/Odometry`，速度 `twist.twist.linear.x`，位置 `pose.pose.position.{x,y}` |
| 规划轨迹 | `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/Trajectory` |

velocity_smoother 9 个 topic（**子目录名固定**）：
| 子目录 | topic |
| --- | --- |
| `raw` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_raw` |
| `ext_vel_limited` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_external_velocity_limited` |
| `lat_acc` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered` |
| `steer_rate_limited` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited` |
| `time_resampled` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled` |
| `forward_filtered` | `/planning/scenario_planning/velocity_smoother/debug/forward_filtered_trajectory` |
| `backward_filtered` | `/planning/scenario_planning/velocity_smoother/debug/backward_filtered_trajectory` |
| `merged_filtered` | `/planning/scenario_planning/velocity_smoother/debug/merged_filtered_trajectory` |
| `output` | `/planning/scenario_planning/velocity_smoother/trajectory` |

---

## 2. 输出目录结构（与 0424/05 完全一致）

> 以 bag_08 为例，`zero_vel_sec = 1777348039`，窗口 `[zero_vel_sec, zero_vel_sec + 2]`

```
planning_control_bag_08_logs/
├── 08_traj.log                                                  # 复制/链接自 LOG_FILE（保留原名前缀 NN_traj.log）
├── control_cmd.csv                                              # 全 bag control_cmd: 2 列
├── control_cmd_<sec0>_<sec1>.csv                                # 窗口 control_cmd: 3 列
├── control_cmd_velocity_curve.png                               # 全 bag 速度曲线
├── control_cmd_velocity_curve_summary.txt
├── control_cmd_velocity_curve_<sec0>_<sec1>.png                 # 窗口速度+加速度曲线
├── control_cmd_velocity_curve_<sec0>_<sec1>_summary.txt
├── control_cmd_velocity_only_<sec0>_<sec1>.png                  # 窗口仅速度曲线
├── control_cmd_velocity_only_<sec0>_<sec1>_summary.txt
├── kinematic_state.csv                                          # 全 bag odometry: 3 列 (header_stamp,x,y)
├── velocity_smoother_sec_<zero_vel_sec>_stage_overlay.png       # 9 阶段叠加图
├── velocity_smoother_sec_<zero_vel_sec>_stage_overlay.txt       # 叠加图说明
└── velocity_smoother_sec_<zero_vel_sec>_plots/
    ├── summary.txt
    ├── raw/                  NNN_<bag_ts_ns>.txt   NNN_<bag_ts_ns>.png
    ├── ext_vel_limited/      ...
    ├── lat_acc/
    ├── steer_rate_limited/
    ├── time_resampled/
    ├── forward_filtered/
    ├── backward_filtered/
    ├── merged_filtered/
    └── output/
```

**命名约定**：
- `<sec0>_<sec1>` = `<zero_vel_sec>_<zero_vel_sec + 2>`，例如 `1777348039_1777348041`
- `NNN` = 该 topic 内消息序号，3 位零填（`000` 起）
- `<bag_ts_ns>` = bag 接收时间纳秒（用作文件名以保证唯一）
- `08_traj.log` 中的 `08` 取自 `LOG_FILE` basename 的数字编号（`autoware_08.log` → `08`），即 `{NN}_traj.log`

---

## 3. 各文件格式规范（与 0424/05 字节级一致）

### 3.1 `control_cmd.csv`（全 bag）
```
header_stamp,velocity
1777001125.893651485,0.000000
1777001125.932006359,0.000000
...
```
- 列：`header_stamp`（小数秒，9 位精度），`velocity`（6 位精度）
- 范围：bag 内全部 `/control/command/control_cmd`

### 3.2 `control_cmd_<sec0>_<sec1>.csv`（窗口）
```
header_stamp,relative_time_s,velocity
1777001222.013647556,0.013647556,2.138910
...
```
- 多一列 `relative_time_s = header_stamp - sec0`
- 窗口闭区间 `[sec0, sec1)`，**严格 2 秒**

### 3.3 `kinematic_state.csv`（全 bag）
```
header_stamp,x,y
1777001093.631428719,-177.183591,-90.444884
...
```
- 列：`header_stamp,x,y`（**不含速度**，与 0424/05 保持一致）
- 范围：bag 内全部 `/localization/kinematic_state`

### 3.4 `control_cmd_velocity_curve.png` / `_summary.txt`（全 bag）
图：横轴 = bag 内时间（rel_t = stamp - first_stamp），纵轴 = velocity（含估计加速度的双 y 轴或单图视模板而定）

`_summary.txt` 格式：
```
input: <abs path>/control_cmd.csv
output_plot: <abs path>/control_cmd_velocity_curve.png
samples: <N>
start_stamp: <first stamp>
end_stamp: <last stamp>
duration_s: <last - first>
min_velocity_mps: <min>
max_velocity_mps: <max>

top_velocity_step_changes:
  rel_t=<rel>s dv=<dv>m/s dt=<dt>s prev=<v_prev> curr=<v_curr> est_acc=<dv/dt>m/s^2
  ...（取 |est_acc| 最大的 5 条）
```

### 3.5 `control_cmd_velocity_curve_<sec0>_<sec1>.png` / `_summary.txt`（窗口）
`_summary.txt` 比 3.4 多一行 `filtered_csv:` 和 `time_window_s:`，`top_velocity_step_changes` 每行多 `stamp=...`：
```
input: <abs>/control_cmd.csv
filtered_csv: <abs>/control_cmd_<sec0>_<sec1>.csv
output_plot: <abs>/control_cmd_velocity_curve_<sec0>_<sec1>.png
time_window_s: [<sec0>.000000000, <sec1>.000000000]
samples: <N>
first_stamp: <first>
last_stamp: <last>
duration_s: <last - first>
min_velocity_mps: <min>
max_velocity_mps: <max>

top_velocity_step_changes:
  rel_t=<rel>s stamp=<stamp> dv=<dv>m/s dt=<dt>s prev=<...> curr=<...> est_acc=<...>m/s^2
  ...（取 |est_acc| 最大的 8 条）
```

### 3.6 `control_cmd_velocity_only_<sec0>_<sec1>.png` / `_summary.txt`
- 图：只画 velocity 曲线（无加速度）
- summary 与 3.5 类似，**`top_velocity_step_changes` 行末尾去掉 `est_acc=...`**：
  ```
  rel_t=<rel>s stamp=<stamp> dv=<dv>m/s dt=<dt>s prev=<...> curr=<...>
  ```

### 3.7 velocity_smoother 单条 `.txt`
```
bag_timestamp_ns: <bag t in ns>
header_sec: <header.stamp.sec>
point_count: <points 数>
max_adjacent_jump: <max |v[i+1] - v[i]|>
jump_index: <发生 max_adjacent_jump 的 i>
jump_values: <v[jump_index]> -> <v[jump_index+1]>

idx,x,longitudinal_velocity_mps
0,<x0>,<v0>
1,<x1>,<v1>
...
```
- `x` 取 `pose.position.x`（**不是弧长**），9 位小数
- `longitudinal_velocity_mps` 取自 `TrajectoryPoint.longitudinal_velocity_mps`，9 位小数
- 头部数值精度：`max_adjacent_jump` / `jump_values` 9 位小数

### 3.8 velocity_smoother 单条 `.png`
- 横轴 `idx`（或 `x`），纵轴 `longitudinal_velocity_mps`，单图无样式特殊设置

### 3.9 `velocity_smoother_sec_<sec>_plots/summary.txt`
```
bag: <BAG_DIR 相对路径或缩写>
target_sec: <sec>
filter: header_sec == <sec>

total_plots: <Σ msg_count>
[raw]  topic=...
  msg_count=<N>
  out_dir=<相对/缩写路径>/raw

[ext_vel_limited]  topic=...
  msg_count=<N>
  out_dir=...
... （9 段，顺序：raw, ext_vel_limited, lat_acc, steer_rate_limited, time_resampled,
                   forward_filtered, backward_filtered, merged_filtered, output）
```

### 3.10 `velocity_smoother_sec_<sec>_stage_overlay.txt`
```
source_bag: <abs bag path>
control_jump_stamp: <在 zero_vel 附近、|dv/dt| 最大的 control_cmd stamp>
stage_sample_index: <NNN>
note: stage sample is the velocity_smoother cycle immediately before the control_cmd drop.

raw: ts_ns=<ts> points=<n> v_range=[<min>, <max>] max_adjacent_jump=<...> jump_values=<a> -> <b>
external limited: ts_ns=...
lateral acc: ts_ns=...
steer rate: ts_ns=...
time resampled: ts_ns=...
forward filtered: ts_ns=...
backward filtered: ts_ns=...
merged filtered: ts_ns=...
velocity_smoother out: ts_ns=...
```
- 选取规则：`control_jump_stamp` = 窗口内 `|dv/dt|` 最大的一对 `control_cmd` 的后一帧 stamp
- 9 个 topic 各取 `header.stamp ≤ control_jump_stamp` 中最近一帧（"drop 之前最后一帧"）

### 3.11 `velocity_smoother_sec_<sec>_stage_overlay.png`
- 同一坐标轴叠加 9 条 (`x`, `longitudinal_velocity_mps`) 折线，按 3.10 中标签命名图例

---

## 4. AI 执行步骤（流程伪码）

```python
# argparse: BAG_DIR LOG_FILE [OUT_DIR]
goal_xy            = parse_goal_from_log(LOG_FILE)               # 兜底取 bag /planning/mission_planning/goal
arrive_time        = parse_arrive_time_from_log(LOG_FILE)        # 正则 'AutowareState: Driving => ArrivedGoal'
                                                                 # 兜底：kinematic_state 距 goal <=2m 首次时间
copy(LOG_FILE -> OUT_DIR/{NN}_traj.log)                          # 1:1 复制；NN 取自 LOG_FILE 数字编号

# 全 bag 提取
write control_cmd.csv      # 2 列，全部消息
write kinematic_state.csv  # 3 列 (header_stamp,x,y)

# zero_vel_time
window_csv = filter control_cmd by [arrive_time-10, arrive_time+10]
zero_vel_time = backward_scan(window_csv, EPS=1e-3)
zero_vel_sec  = floor(zero_vel_time)
sec0, sec1    = zero_vel_sec, zero_vel_sec + 2

# 窗口 csv & 三张图
write control_cmd_{sec0}_{sec1}.csv               # 3 列
plot+summary control_cmd_velocity_curve.png            (3.4)
plot+summary control_cmd_velocity_curve_{sec0}_{sec1}.png   (3.5)
plot+summary control_cmd_velocity_only_{sec0}_{sec1}.png    (3.6)

# velocity_smoother bundle
for topic in 9_topics:
    msgs = filter(topic, header.sec == zero_vel_sec)
    for i, m in enumerate(msgs):
        write {sub}/{i:03d}_{bag_ts_ns}.txt  (3.7)
        write {sub}/{i:03d}_{bag_ts_ns}.png  (3.8)
write velocity_smoother_sec_{sec}_plots/summary.txt   (3.9)

# stage overlay
control_jump_stamp = argmax(|dv/dt|) over window_csv
for each of 9 topics: pick last msg with header.stamp <= control_jump_stamp
write velocity_smoother_sec_{sec}_stage_overlay.txt  (3.10)
write velocity_smoother_sec_{sec}_stage_overlay.png  (3.11)
```

---

## 5. 注意事项

- 读 bag 一定加 `StorageFilter`，否则 `planning_control_bag_*` 数十万条会很慢
- 时间字段统一用 `header.stamp`，避免与 bag 接收时间 `t` 混用导致 5~50ms 错位
- `Control.longitudinal.velocity` / `TrajectoryPoint.longitudinal_velocity_mps` 字段名以当前 humble 消息为准，写代码前用 `ros2 interface show ...` 复核
- 若 bag 缺 `/vehicle/status/velocity_status`，速度统一用 `/localization/kinematic_state.twist.twist.linear.x`，并在 `summary.txt` 注明
- 数字精度（与 0424/05 对齐）：
  - `header_stamp` 9 位小数
  - `velocity` / `acceleration` 6 位小数
  - velocity_smoother txt 内的 `x` / `longitudinal_velocity_mps` / `max_adjacent_jump` / `jump_values` 9 位小数
- 输出路径绝不放进 `BAG_DIR` 内部，固定写到平级 `${BAG_DIR}_logs/`
- 路径风格：summary 内出现的路径与 0424/05 一致使用绝对路径（除 `velocity_smoother_sec_*_plots/summary.txt` 中可用相对路径，与模板保持一致）
- 文件存在时直接覆盖；脚本无 `--force` 选项
请帮我做一个plan，利用录制的rosbag包(.bag文件)和log文件(.log)来进行到达终点段的速度和轨迹分析，分析流程如下:
下面所说的时间都只精确到sec
1. 读取日志log文件，找到终点位置 goal_pose 和到达终点时刻的时间戳 arrive_time
2. 获取 arrive_time 前后 10S 的cmd控制指令话题
3. 找到实际停下来的那一刻时间戳 zero_vel_time，也就是 zero_vel_time 之后的速度指令全为0
4. 获取 zero_vel_time 时刻的 traj和kinematic_state话题
5. 获取 zero_vel_time 时刻的 velocity_smoother 的内部话题数据

---

## AI 执行 Plan（到达终点段速度/轨迹分析）

### 输入
- `BAG_DIR`: 待分析的 rosbag2 目录，例如 `log/0428/planning_control_bag_08/`（含 `metadata.yaml` + `*_0.db3`）
- `LOG_FILE`: 同次运行的 launch 日志，例如 `log/0428/autoware_08.log`
- `OUT_DIR`: 默认 `${BAG_DIR%/}_logs/`（与 0424 模板同名风格）

### 环境
- `source /opt/ros/${ROS_DISTRO:-humble}/setup.bash`
- 用 Python + `rosbag2_py.SequentialReader` + `rclpy.serialization.deserialize_message`；读 bag 时务必 `StorageFilter(topics=[...])` 只筛需要的 topic，避免全量解析

### 关键 topic（消息类型）
- 终点：`/planning/mission_planning/goal`（`geometry_msgs/PoseStamped`），缺失则用 `/planning/mission_planning/route` 的 `goal_pose` 兜底
- 控制指令：`/control/command/control_cmd`（`autoware_control_msgs/Control`，速度字段 `longitudinal.velocity`）
  - 备选：`/control/trajectory_follower/control_cmd`、`/control/command/gear_cmd`
- 实际车速：`/localization/kinematic_state`（`nav_msgs/Odometry`，`twist.twist.linear.x`），若 bag 录了 `/vehicle/status/velocity_status` 优先用它
- 规划轨迹：`/planning/scenario_planning/trajectory`（`autoware_planning_msgs/Trajectory`）
- velocity_smoother 内部 9 个 topic（按 0424 模板命名子目录）：

  | 子目录 | topic |
  | --- | --- |
  | `raw` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_raw` |
  | `ext_vel_limited` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_external_velocity_limited` |
  | `lat_acc` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered` |
  | `steer_rate_limited` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited` |
  | `time_resampled` | `/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled` |
  | `forward_filtered` | `/planning/scenario_planning/velocity_smoother/debug/forward_filtered_trajectory` |
  | `backward_filtered` | `/planning/scenario_planning/velocity_smoother/debug/backward_filtered_trajectory` |
  | `merged_filtered` | `/planning/scenario_planning/velocity_smoother/debug/merged_filtered_trajectory` |
  | `output` | `/planning/scenario_planning/velocity_smoother/trajectory` |

### 步骤

#### Step 1 —— 从 .log 提取 `goal_pose` 和 `arrive_time`（只精确到 sec）
- 在 `LOG_FILE` 中正则匹配 goal 行；若日志不直接打印 goal，则回退到从 bag 的 `/planning/mission_planning/goal` 取首条
- `arrive_time` 取自日志中"到达"/"reach goal"等行的 `[stamp]`；若日志无显式行，则用 bag 的 `/localization/kinematic_state` 首次满足 `hypot(pos - goal) <= 2.0 m` 的 `header.stamp`
- 输出：`OUT_DIR/goal_arrive.txt`：`goal_x, goal_y, arrive_time_sec`

#### Step 2 —— 导出 `[arrive_time-10s, arrive_time+10s]` 的 cmd 控制指令
- 读 `/control/command/control_cmd`，按 `header.stamp` 过滤
- 输出 `OUT_DIR/control_cmd_around_arrive.csv`，列：`header_stamp, velocity, acceleration, steering_tire_angle`
- 同时输出 `kinematic_state_around_arrive.csv`（`stamp, vx, ax, dist_to_goal`）便于对照

#### Step 3 —— 求 `zero_vel_time`
- 定义：`control_cmd.longitudinal.velocity` 在该时刻之后**所有**采样均 `|v| < EPS`（默认 `EPS=1e-3`）
- 算法：从 csv 末尾向前回溯，找到最后一个 `|v| >= EPS` 的样本，其下一条样本时间即 `zero_vel_time`；若整段都为 0，则取窗口起点
- 输出 `OUT_DIR/zero_vel_time.txt`：`zero_vel_time_sec`

#### Step 4 —— 导出 `zero_vel_time` 那一秒（`header.sec == floor(zero_vel_time)`）的 traj 与 kinematic_state
- 复用 0424 提取脚本 `log/0424/extract_topics.py` 的 `header_sec == target_sec` 过滤思路
- 导出：
  - `OUT_DIR/trajectory_sec_<sec>/NNN_<stamp_ns>.txt`：每条 `Trajectory` 全部点的 `(s, x, y, yaw, vx, ax)` 表
  - `OUT_DIR/kinematic_state_sec_<sec>.csv`：该秒所有 odometry 样本

#### Step 5 —— 导出 `zero_vel_time` 那一秒的 velocity_smoother 9 个 topic（标准模板）
- 模板：`log/0424/planning_control_localization_06.bag_logs/velocity_smoother_sec_<sec>_plots/`
- 目录结构（与 0424 一致，**子目录名固定**）：

  ```
  velocity_smoother_sec_<sec>_plots/
    summary.txt
    raw/                NNN_<stamp_ns>.txt   NNN_<stamp_ns>.png
    ext_vel_limited/    ...
    lat_acc/
    steer_rate_limited/
    time_resampled/
    forward_filtered/
    backward_filtered/
    merged_filtered/
    output/
  ```

- 每条消息一对 `txt` + `png`：
  - `txt`：表头 `idx, s, x, y, vx, ax`，每行一个轨迹点（`s` 为相邻欧氏弧长累加）
  - `png`：横轴 `s`，纵轴 `vx`（matplotlib，单图无样式特殊设置）
- `summary.txt`：照搬 0424 模板，记录 `bag/target_sec/filter` 与每个 topic 的 `msg_count` 和 `out_dir`

### 单脚本骨架（建议落到 `src/byd/near_goal_traj_velo_anly.py`）
```python
# 用 argparse 接收 BAG_DIR / LOG_FILE / OUT_DIR
# 1) parse_log()  -> goal_xy, arrive_time
# 2) dump_window(arrive_time, ±10) -> control_cmd_*.csv, kinematic_state_*.csv
# 3) find_zero_vel(csv) -> zero_vel_time
# 4) dump_sec(floor(zero_vel_time), ['/planning/scenario_planning/trajectory',
#                                    '/localization/kinematic_state'])
# 5) dump_velocity_smoother_bundle(floor(zero_vel_time))
```

### 注意事项
- 读 bag 一定加 `StorageFilter`，否则 `planning_control_bag_*` 数十万条会很慢
- 时间字段统一用 `header.stamp`（不是 bag 接收时间 `t`），避免 5~50ms 抖动错位
- `Control.longitudinal.velocity` 字段名以当前 humble msg 为准，写代码前用 `ros2 interface show autoware_control_msgs/msg/Control` 复核
- 该 bag 若无 `/vehicle/status/velocity_status`，速度统一退到 `/localization/kinematic_state.twist.twist.linear.x`，并在 `summary.txt` 注明
- 输出绝对不要塞进 `BAG_DIR` 内部，固定写到平级 `${BAG_DIR}_logs/`
