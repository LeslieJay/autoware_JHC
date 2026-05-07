### 关键信息

1. 终点不允许修改（哪怕允许修改到终点附件，工程上最好也别用），所以必须使用 fixed_goal_planner

2. fixed_goal_planner 的触发条件：
- Route is set with `allow_goal_modification=false`. This is the default.
- The goal is set on `road` lanes.

3. fixed_goal_planner 的特点
没有"goal search", "collision check", "safety check", etc等功能.

### 整体流程

1. 初始化 goal_searcher
2. 生成 goal candidates
3. 判断是否触发规划线程
4. 如果模块未激活 → 退出
5. 重置路径
6. 安全检测（静态+动态障碍物）
7. 状态机更新（核心）
8. 同步子线程（lane / freespace）
9. 更新 context_data（核心数据）
10. 决策完成 → 处理速度
11. 如果已激活 → 推进路径执行

# 仿真结果

## no path candidate

1. 现象：接近终点时，显示 no path candidate 停止状态
![no path candidate](./images/no_path_candidate.png)

"no path candidate" 消息产生于 goal_planner_module.cpp:1554，条件是：

goal_candidates_ 不为空（已找到有效的 goal 候选点）
但 pull_over_path_candidates 为空（所有规划器都无法为这些 goal 候选点生成有效路径）
这意味着 三个路径规划器（SHIFT、ARC_FORWARD、ARC_BACKWARD）对所有 goal candidate 全部规划失败。

## 常见原因及排查方法


1. 没有 shoulder lane（路肩车道）— 最常见原因
getPullOverLanes() 会查找 goal 附近的 shoulder lane。如果地图中 goal 位置附近没有 shoulder lane，返回的是最外侧的 route lane。此时 goal planner 试图在行车道内做 pull over，shift 距离为 0 或非常小，路径规划很容易因为 lane departure check 失败而返回空。

排查：在 RViz 中检查 goal 附近是否有 shoulder lanelet。

2. parking_policy 方向不匹配
当前配置为 parking_policy: "right_side"。如果你的地图中 goal 靠左侧或 shoulder lane 在左侧，需要改为 "left_side"。

3. goal 搜索范围太小
当前配置：
forward_goal_search_length: 15.0   # 向前搜索 15m
backward_goal_search_length: 5.0   # 向后搜索 5m

如果车辆接近 goal 时距离较近，可能搜索范围不够。

4. Shift planner 的 lane departure check 过于严格
Shift planner 在 shift_pull_over.cpp:282 中会检查生成的路径是否超出 departure_check_lane。如果 goal 在道路边缘或 lane 较窄，shifted path 会因为超出车道边界而被拒绝。

5. margin_from_boundary 太大
margin_from_boundary: 0.75 意味着 goal 候选点必须距离车道边界至少 0.75m。对于窄道，这个值可能太大。

6. collision_check_soft_margins 太大

collision_check_soft_margins: [5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]

最大 5m 的碰撞检测余量，如果 goal 附近有任何障碍物，几乎所有路径都会被判定不安全。

推荐的实时调试命令
# 1. 查看 goal planner 的 debug 信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/goal_planner --once 2>/dev/null | head -100

# 2. 检查是否找到了 goal candidates
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/goal_planner --once 2>/dev/null | head -50

# 3. 查看 pull over path candidate 的数量
ros2 topic echo /planning/path_candidate/goal_planner --once 2>/dev/null | head -10

# 4. 检查当前参数
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.forward_goal_search_length
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.margin_from_boundary
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.pull_over.shift_parking.enable_shift_parking

推荐的参数调整（即时生效）
# 方案A: 增大搜索范围
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.forward_goal_search_length 30.0
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.backward_goal_search_length 15.0

# 方案B: 减小边界余量（适用于窄道）
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.margin_from_boundary 0.3

# 方案C: 放宽碰撞检测margins
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.object_recognition.collision_check_soft_margins [2.0,1.5,1.0,0.8,0.6]

# 方案D: 放宽lane departure检测
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.pull_over.lane_departure_check_expansion_margin 0.5

# 方案E: 增加lateral搜索范围
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.max_lateral_offset 2.0


如果是地图问题（最根本的解决方案）
如果你的 goal 设置在没有 shoulder lane 的普通车道终点，goal planner 的 pull over 逻辑本身就不太适用。这种情况下有两个方向：

1. 在地图中为 goal 附近添加 shoulder lanelet，让 pull over 有合法的目标车道

2. 考虑 goal 是否应该设在车道中心线上，而不是需要 pull over 的位置

---

## 2026-04-25 快速理解与精停建议（新增）

### 1. 代码在哪里看（建议阅读顺序）

1. `src/manager.cpp`
- 参数入口与装配：`GoalPlannerModuleManager::initGoalPlannerParameters()`
- 场景模块实例化：`createNewSceneModuleInstance()`

2. `src/goal_planner_module.cpp`
- 主流程：`updateData()`、`plan()`、`isExecutionRequested()`
- 线程同步：`syncWithThreads()`
- 目标判定：`isOnModifiedGoal()`

3. `src/decision_state.cpp`
- 状态机：`NOT_DECIDED -> DECIDING -> DECIDED`
- 稳定安全判定与迟滞：`keep_unsafe_time` + `check_collision_duration`

4. `src/goal_searcher.cpp`
- 候选目标生成：纵向/横向离散采样
- 候选排序：`minimum_weighted_distance` / `minimum_longitudinal_distance`

5. `src/pull_over_planner/shift_pull_over.cpp`
- Shift 几何生成：`generatePullOverPath()`
- 末段速度整形：`generateReferencePath()`

### 2. 组件运行逻辑（快速心智模型）

1. 模块是否触发
- `isExecutionRequested()` 先判断目标是否可达、是否在请求距离内。

2. 目标是否可修改
- 允许修改：走 `goal_searcher.search()`，在目标附近生成多个 `goal_candidates`。
- 不允许修改：只保留原始 goal（等价 fixed-goal 行为）。

3. 候选路径生成（异步）
- 主线程发送 request。
- Lane Parking 线程生成 Shift/Arc/Bezier 候选。
- Freespace 线程在需要时生成 freespace 候选。

4. 安全判定与状态机
- 先做动态/静态障碍物安全检查。
- 在 `DECIDING` 持续稳定安全一段时间后进入 `DECIDED`。

5. 执行与收敛
- 选定 pull-over path 后执行。
- 距离 `modified_goal` 小于 `th_arrived_distance` 即判定到达。

### 3. 与停车精度直接相关的关键参数（按影响链路）

1. 目标离散分辨率（规划上限）
- `goal_search.goal_search_interval`
- 近似决定纵向离散误差上限：`~ interval / 2`

2. 路径离散与几何拟合
- `center_line_path_interval`
- 太大时，shift 起终点附近会出现几何量化误差

3. 末段几何偏置
- `pull_over.shift_parking.after_shift_straight_distance`
- 在 Shift 模式下，这个值直接把 shift 结束点放在 goal 前方（沿切向）

4. 到达判定门限
- `th_arrived_distance`
- 决定系统“宣布到达”的距离阈值，不改变真实轨迹，但会改变停止判定时刻

5. 末段速度与制动可控性
- `pull_over.pull_over_velocity`
- `pull_over.pull_over_minimum_velocity`
- `pull_over.maximum_deceleration`
- `pull_over.shift_parking.deceleration_interval`

### 4. 你当前配置中的亮点与风险

#### 已经做得很对的部分

1. `goal_search_interval=0.02` + `th_arrived_distance=0.02`
- 规划离散分辨率与到达门限都对齐到 2 cm 级别。

2. `pull_over_velocity=0.5` / `pull_over_minimum_velocity=0.1`
- 低速末段更有利于减小控制超调。

3. `after_shift_straight_distance=0.3`
- 已显著减小“提前停车”偏置。

#### 仍建议关注的点

1. `max_lateral_offset=0.1` 但 `lateral_offset_interval=0.5`
- 这会导致横向采样几乎只有 `dy=0`（基本不搜索横向微调）。
- 若目标对齐误差经常是横向主导，建议把 `lateral_offset_interval` 降到 `0.02~0.05`。

2. Bezier 分支仍是 `bezier_parking.after_shift_straight_distance=1.5`
- 当进入 Bezier 分支（尤其 bus stop area 场景）时，可能重新引入明显纵向偏置。
- 若你会启用 Bezier，建议与 Shift 分支保持一致（例如 `0.3`）。

3. `mission_planner.enable_correct_goal_pose=false`
- 若上层给定 goal 的姿态/落点本身不理想，关闭纠偏会把误差原样下发给下游。
- 对精停场景，建议根据地图质量决定是否启用。

### 5. 一个务实的精停调参流程（建议按此执行）

1. 固定工况重复测试 20~50 次
- 相同速度、相同初始姿态、相同路段。

2. 记录两类误差
- 规划终点误差：最终轨迹终点 vs modified_goal。
- 实车停车误差：最终车辆 base_link vs modified_goal。

3. 分离“系统偏置”与“随机噪声”
- 均值偏差大：优先改几何参数（`after_shift_straight_distance`、采样间隔）。
- 标准差大：优先看定位噪声/控制稳定性，而不是继续压规划参数。

4. 参数迭代建议步长
- `after_shift_straight_distance` 每次改 `0.05m`。
- `goal_search_interval` / `center_line_path_interval` 每次改 `0.01m`。
- 速度参数每次改 `0.1m/s`。

5. 性能与精度平衡
- 搜索间隔过小会显著增加候选数量与规划耗时。
- 建议先用日志确认线程周期与 CPU 余量，再继续压间隔。

### 6. 现实边界（必须明确）

仅靠 goal planner 参数，通常无法稳定保证 ±2 cm 实车停车精度。常见瓶颈是：

1. 定位噪声（NDT/匹配误差通常在厘米级以上）
2. 底盘与低速控制跟踪误差
3. 感知时延与控制周期离散化

因此应采用“规划 + 控制 + 定位”三层联合优化：

1. 规划层：离散分辨率与末段几何偏置
2. 控制层：低速制动与爬行段参数
3. 定位层：高精度定位与时间同步

以上三层一起收敛，才有机会接近 2 cm 目标。
