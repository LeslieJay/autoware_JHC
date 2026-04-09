# ElasticBandSmoother 工作流程与关键数据总结

## 节点概述

`ElasticBandSmoother` 是一个 ROS2 节点，订阅规划路径（`Path`），使用弹性带（Elastic Band）QP 优化对其进行平滑，并发布平滑后的轨迹（`Trajectory`）。

---

## 话题接口

| 方向 | 话题 | 类型 |
|------|------|------|
| 输入 | `~/input/path` | `autoware_planning_msgs/Path` |
| 输入 | `~/input/odometry` | `nav_msgs/Odometry` |
| 输出 | `~/output/traj` | `autoware_planning_msgs/Trajectory` |
| 输出 | `~/output/path` | `autoware_planning_msgs/Path` |

---

## 主要工作流程（`onPath` 回调）

```
Path 输入
  │
  ├─ [1] isDataReady：检查 ego_pose/twist、path 点数、左右边界
  │
  ├─ [2] isDrivingForward：检测行驶方向，不支持倒退（倒退时直接透传）
  │
  ├─ [3] 转换：Path → vector<TrajectoryPoint>（input_traj_points）
  │
  ├─ [4] 重规划判断（ReplanChecker）
  │       ├─ isResetRequired → 需要则 resetPreviousData()，触发重优化
  │       └─ isReplanRequired → 检查时间/路径形状变化
  │
  ├─ [5] EBPathSmoother::smoothTrajectory（弹性带 QP 优化）
  │         或直接复用 prev_optimized_traj_points_ptr_
  │
  ├─ [6] applyInputVelocity：将输入路径速度同步到优化轨迹，并插入精确停止点
  │
  ├─ [7] extendTrajectory：将优化轨迹末端与后续输入路径接续，重采样为完整轨迹
  │
  ├─ [8] setZeroVelocityAfterStopPoint：停止点之后速度全部置零
  │
  └─ [9] 发布 Trajectory / Path
```

---

## EBPathSmoother::smoothTrajectory 内部流程

```
输入 traj_points + ego_pose
  │
  ├─ [1] cropPoints：
  │       前向裁剪 = num_points × delta_arc_length
  │       后向裁剪 = output_backward_traj_length
  │
  ├─ [2] insertFixedPoint：
  │       用上次优化结果的头部替换当前轨迹头部（保证前后帧连续性）
  │
  ├─ [3] resampleTrajectoryPointsWithoutStopPoint：
  │       按 delta_arc_length 等间距重采样（等间距是 QP 稳定的前提）
  │
  ├─ [4] getPaddedTrajectoryPoints：
  │       不足 num_points 时用末尾点填充至固定长度
  │
  ├─ [5] updateConstraint（构建 QP 问题）：
  │       ┌─────────────────────────────────────────────────────┐
  │       │  点索引   约束宽度（lateral clearance）              │
  │       │  i=0      clearance_for_fix（几乎固定，连续性保障）  │
  │       │  目标点   clearance_for_fix（固定目标姿态）          │
  │       │  i<num_joint_points+1  clearance_for_joint（拼接段） │
  │       │  其余     clearance_for_smooth（自由平滑段）          │
  │       └─────────────────────────────────────────────────────┘
  │       P 矩阵 = smooth_weight × P_smooth + lat_error_weight × I
  │       P_smooth 是五对角矩阵（二阶差分，最小化曲率变化）
  │       优化变量 δ 为各点横向偏移量
  │
  ├─ [6] OSQP 求解（支持热启动 warm start）
  │
  ├─ [7] convertOptimizedPointsToTrajectory：
  │       将 δ 叠加回参考点，重建 TrajectoryPoint（验证误差 < max_validation_error）
  │
  └─ [8] 缓存结果到 prev_eb_traj_points_ptr_，发布调试话题
```

**优化目标**：

$$\min_{\delta} \frac{1}{2}\delta^T P \delta + q^T \delta \quad \text{s.t.} \quad lb_i \leq \delta_i \leq ub_i$$

---

## ReplanChecker 判断逻辑

### isResetRequired（触发完全重置 + 重优化）
- 无历史数据（首帧）
- 自车周围路径横向偏移变化 > `max_path_shape_around_ego_lat_dist_`
- 终点位置变化 > `max_goal_moving_dist_`
- 自车位置跳变 > `max_ego_moving_dist_`

### isReplanRequired（触发重新优化，不重置）
- 距上次优化时间 > `max_delta_time_sec_`
- 前方路径形状横向偏移 > `max_path_shape_forward_lat_dist_`（每隔 10m 检查一次）

---

## 关键数据结构

### `PlannerData`
```cpp
traj_points   // 当前输入轨迹点
ego_pose      // 自车位姿
ego_vel       // 自车纵向速度
```

### `EBParam`（弹性带 QP 参数）
```cpp
num_points              // 优化点总数（固定长度）
delta_arc_length        // 重采样弧长间距（m）
num_joint_points        // 拼接段点数
clearance_for_fix       // 固定点约束宽度（≈0，几乎不移动）
clearance_for_joint     // 拼接段约束宽度（小）
clearance_for_smooth    // 平滑段约束宽度（大）
smooth_weight           // 曲率平滑权重
lat_error_weight        // 横向误差惩罚权重
qp_param                // OSQP: max_iteration, eps_abs, eps_rel
enable_warm_start       // 是否使用热启动
enable_optimization_validation  // 是否验证优化结果误差
max_validation_error    // 最大允许优化误差（m）
```

### `CommonParam`
```cpp
output_delta_arc_length       // 最终输出轨迹重采样间距（m）
output_backward_traj_length   // 向后保留长度（m）
```

### `EgoNearestParam`
```cpp
dist_threshold   // 最近点搜索最大距离（m）
yaw_threshold    // 最近点搜索最大偏航角差（rad）
```

---

## 重要状态变量

| 变量 | 说明 |
|------|------|
| `prev_optimized_traj_points_ptr_` | 上帧完整优化结果，不需重规划时直接复用 |
| `prev_eb_traj_points_ptr_`（EBPathSmoother内） | 上帧 EB 优化结果，用于插入固定点保证连续性 |
| `prev_traj_points_ptr_`（ReplanChecker内） | 上帧输入轨迹，用于判断路径是否变化 |
| `prev_ego_pose_ptr_`（ReplanChecker内） | 上帧自车位姿，用于检测跳变 |
| `prev_replanned_time_ptr_`（ReplanChecker内） | 上次执行重优化的时刻 |
