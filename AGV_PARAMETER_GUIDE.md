# AGV静态障碍物绕避参数配置指南

## 适用场景
- **车辆类型**: 低速AGV
- **运行环境**: 封闭园区室外道路
- **最高速度**: ≤15 km/h (4.17 m/s)
- **特点**: 低速、封闭、可控环境

---

## 参数详解

### 一、基础采样参数

| 参数 | 默认值 | AGV建议 | 说明 |
|------|--------|---------|------|
| `resample_interval_for_planning` | 0.3m | **0.2m** | 规划时路径点间隔，越小越精细 |
| `resample_interval_for_output` | 4.0m | **2.0m** | 输出路径点间隔，低速需要更密集 |

**原理**: 低速场景下，车辆转向响应需要更精细的路径控制。

---

### 二、车道使用策略

| 选项 | 说明 | AGV适用性 |
|------|------|-----------|
| `current_lane` | 仅使用当前车道 | ❌ 限制太大 |
| `same_direction_lane` | 使用同向车道 | ✅ **推荐** |
| `opposite_direction_lane` | 使用双向车道 | ⚠️ 园区内通常不需要 |

**AGV建议**: `same_direction_lane` - 园区内道路通常单向或不需要逆行绕避

---

### 三、目标物体参数（按类型）

#### 3.1 通用参数结构

每种物体类型都包含以下参数：

```yaml
th_moving_speed: X.X        # 移动速度阈值 [m/s]
th_moving_time: X.X         # 持续移动时间 [s]
longitudinal_margin: X.X    # 纵向安全裕度 [m]
lateral_margin:
  soft_margin: X.X          # 软横向裕度（舒适距离）[m]
  hard_margin: X.X          # 硬横向裕度（最小安全距离）[m]
  hard_margin_for_parked_vehicle: X.X  # 停放车辆硬裕度 [m]
```

#### 3.2 各类型物体AGV建议值

| 物体类型 | 移动速度阈值 | 纵向裕度 | 软裕度 | 硬裕度 | 说明 |
|----------|--------------|----------|--------|--------|------|
| **car/truck/bus** | 0.5 m/s ↓ | 0.5m ↑ | 0.8m ↑ | 0.3m ↑ | 园区内车辆速度慢 |
| **unknown** | 0.3 m/s ↓ | 0.5m ↑ | 0.8m ↑ | 0.2m | 未知物体保守策略 |
| **bicycle** | 0.3 m/s ↓ | 0.8m ↑ | 1.0m ↑ | 0.6m ↑ | 自行车不稳定，需更大空间 |
| **pedestrian** | 0.2 m/s ↓ | 1.5m ↑↑ | 1.2m ↑↑ | 0.8m ↑↑ | **重点保护**，最大安全距离 |

**↑ 表示相对默认值增加，↓ 表示降低**

#### 3.3 关键参数说明

- **th_moving_speed**: 速度超过此值认为是移动物体
  - 默认值高（1.0 m/s）适合高速场景
  - AGV场景降低（0.3-0.5 m/s）以快速识别慢速移动物体
  
- **longitudinal_margin**: 前后安全距离
  - 行人设置最大（1.5m）
  - 车辆设置中等（0.5m）
  
- **soft_margin vs hard_margin**:
  - soft: 理想保持距离，用于规划
  - hard: 最小安全距离，不可突破
  - AGV建议增加soft_margin提高舒适度

---

### 四、目标过滤参数

#### 4.1 检测范围

| 参数 | 默认值 | AGV建议 | 说明 |
|------|--------|---------|------|
| `min_forward_distance` | 50.0m | **20.0m** | 最小前方检测距离 |
| `max_forward_distance` | 150.0m | **60.0m** | 最大前方检测距离 |
| `backward_distance` | 10.0m | **5.0m** | 后方检测距离 |
| `object_check_goal_distance` | 20.0m | **15.0m** | 目标点附近检测距离 |

**原因**: 低速AGV刹车距离短，不需要太远的检测范围

#### 4.2 停放车辆识别

```yaml
parked_vehicle:
  th_offset_from_centerline: 1.0    # 中心线偏移阈值 [m]
  th_shiftable_ratio: 0.8           # 可移动比率阈值 [-]
  min_road_shoulder_width: 0.5      # 最小路肩宽度 [m]
```

**AGV优化**:
- `th_offset_from_centerline: 0.8` - 园区道路较窄，降低阈值
- `th_shiftable_ratio: 0.7` - 更容易识别为停放车辆
- `min_road_shoulder_width: 0.3` - 适应窄路肩

**判断逻辑**:
1. 物体到中心线距离 > `th_offset_from_centerline`
2. 且 shiftable_ratio > `th_shiftable_ratio`
3. → 认定为停放车辆

#### 4.3 策略配置

| 策略 | 选项 | AGV推荐 | 说明 |
|------|------|---------|------|
| **模糊车辆** | auto/manual/ignore | **auto** | 自动处理，无需人工批准 |
| **违停车辆** | auto/manual/ignore | **auto** | 自动绕避 |
| **相邻车道停车** | auto/manual/ignore | **auto** | 自动处理 |

**auto模式优势**:
- 无需等待操作员批准
- 适合封闭可控环境
- 提高运行效率

---

### 五、安全检查参数

#### 5.1 预测参数（关键）

| 参数 | 默认值 | AGV建议 | 原因 |
|------|--------|---------|------|
| `min_velocity` | 1.38 m/s | **0.5 m/s** | AGV最低速度更低 |
| `max_velocity` | 50.0 m/s | **5.0 m/s** | 园区限速约15km/h |
| `time_resolution` | 0.5s | **0.3s** | 提高时间分辨率，更精细预测 |
| `time_horizon_for_front_object` | 3.0s | **4.0s** | 低速下需更长预测时间 |
| `time_horizon_for_rear_object` | 10.0s | **6.0s** | 后方预测可缩短 |

**公式**: 预测距离 = 速度 × 时间范围
- 低速需要更长时间才能覆盖足够距离
- 示例: 3 m/s × 4s = 12m 前方预测距离

#### 5.2 RSS安全参数

```yaml
expected_front_deceleration: -1.0    # 前车预期减速度 [m/ss]
expected_rear_deceleration: -1.0     # 后车预期减速度 [m/ss]
rear_vehicle_reaction_time: 2.0      # 后车反应时间 [s]
rear_vehicle_safety_time_margin: 1.0 # 安全时间裕度 [s]
```

**AGV优化**:
- 减速度降低到 **-0.8 m/ss** - AGV减速较缓慢
- 反应时间缩短到 **1.5s** - 园区车辆反应快
- 安全裕度降到 **0.8s** - 低速环境可适当降低

#### 5.3 碰撞检查

| 参数 | 作用 | AGV建议 |
|------|------|---------|
| `check_all_predicted_path` | 检查所有预测路径 | **true** - 更保守 |
| `check_unavoidable_object` | 检查无法避开的物体 | **true** - 提高安全性 |
| `safety_check_backward_distance` | 后向安全检查距离 | **30m** - 从100m缩短 |

---

### 六、绕避机动参数

#### 6.1 横向参数

| 参数 | 默认值 | AGV建议 | 说明 |
|------|--------|---------|------|
| `max_right_shift_length` | 5.0m | **2.0m** | 园区道路窄，限制最大偏移 |
| `max_left_shift_length` | 5.0m | **2.0m** | 同上 |
| `max_deviation_from_lane` | 0.2m | **0.15m** | 降低偏离量 |
| `soft_drivable_bound_margin` | 0.3m | **0.2m** | 减少边界裕度 |

#### 6.2 纵向参数（重要）

| 参数 | 默认值 | AGV建议 | 原因 |
|------|--------|---------|------|
| `min_prepare_time` | 1.0s | **1.5s** | AGV加速慢，需更多准备时间 |
| `max_prepare_time` | 3.0s | **4.0s** | 同上 |
| `min_prepare_distance` | 1.0m | **2.0m** | 增加准备距离 |
| `min_slow_down_speed` | 1.38 m/s | **0.5 m/s** | 降低最小减速速度 |
| `nominal_avoidance_speed` | 8.33 m/s | **3.0 m/s** | 大幅降低绕避速度 |

**关键**: 这些参数直接影响绕避是否顺畅！

---

### 七、约束参数（车辆动力学）

#### 7.1 横向约束

```yaml
lateral:
  velocity: [1.39, 4.17, 11.1]           # 速度档位 [m/s]
  max_accel_values: [0.5, 0.5, 0.5]     # 最大横向加速度 [m/ss]
  min_jerk_values:                       # 最小jerk [m/sss]
    avoid: [0.2, 0.2, 0.2]
    return: [0.2, 0.2, 0.2]
  max_jerk_values: [1.0, 1.0, 1.0]      # 最大jerk [m/sss]
```

**AGV优化**:
```yaml
velocity: [0.5, 2.0, 4.0]                # 降低速度档位
max_accel_values: [0.3, 0.3, 0.3]        # 降低加速度，更平滑
min_jerk_values:
  avoid: [0.1, 0.1, 0.1]                 # 降低jerk
  return: [0.1, 0.1, 0.1]
max_jerk_values: [0.5, 0.5, 0.5]         # 降低最大jerk
```

**Jerk说明**: 加速度的变化率，越小越舒适

#### 7.2 纵向约束

| 参数 | 默认值 | AGV建议 | 说明 |
|------|--------|---------|------|
| `nominal_deceleration` | -1.0 | **-0.6** | 标称减速度 |
| `max_deceleration` | -1.5 | **-1.0** | 最大减速度 |
| `max_acceleration` | 0.5 | **0.4** | AGV加速较慢 |
| `nominal_jerk` | 0.5 | **0.3** | 降低jerk提高舒适度 |
| `max_jerk` | 1.0 | **0.6** | 降低最大jerk |

---

### 八、策略配置

#### 8.1 detection_reliability

| 选项 | 说明 | AGV适用性 |
|------|------|-----------|
| `reliable` | 完全信任感知结果，自动决策 | ✅ **推荐** - 园区环境可控 |
| `not_enough` | 不完全信任，需要操作员判断 | ❌ 影响效率 |

#### 8.2 make_approval_request

| 选项 | 说明 | AGV适用性 |
|------|------|-----------|
| `per_shift_line` | 每条shift line需要批准 | ⚠️ 适合需要监督的场景 |
| `per_avoidance_maneuver` | 每次绕避机动需要批准 | ✅ 减少批准次数 |

**建议**: 如果设置 `avoidance_for_ambiguous_vehicle.policy: "auto"`，则批准策略影响较小

#### 8.3 deceleration

| 选项 | 说明 | AGV推荐 |
|------|------|---------|
| `best_effort` | 遵守减速度/jerk约束，可能停不住 | ❌ 不够安全 |
| `reliable` | 忽略约束确保停住，可能不舒适 | ✅ **推荐** - AGV低速可承受 |

---

## 关键参数组合建议

### 配置方案一：完全自动（推荐）

适合封闭园区，无需人工干预

```yaml
# 策略
avoidance_for_ambiguous_vehicle:
  policy: "auto"
avoidance_for_parking_violation_vehicle:
  policy: "auto"
detection_reliability: "reliable"
deceleration: "reliable"

# 安全检查
safety_check:
  enable: true
  check_unavoidable_object: true
  check_all_predicted_path: true
```

### 配置方案二：保守监督

需要操作员批准关键决策

```yaml
# 策略
avoidance_for_ambiguous_vehicle:
  policy: "manual"
avoidance_for_parking_violation_vehicle:
  policy: "manual"
detection_reliability: "not_enough"
make_approval_request: "per_avoidance_maneuver"
```

---

## 调试参数

开发阶段建议全部启用：

```yaml
debug:
  enable_other_objects_marker: true          # 显示其他物体
  enable_other_objects_info: true            # 显示物体信息
  enable_detection_area_marker: true         # 显示检测区域
  enable_drivable_bound_marker: true         # 显示可行驶边界
  enable_safety_check_marker: true           # 显示安全检查结果
  enable_shift_line_marker: true             # 显示shift line
  enable_lane_marker: true                   # 显示车道
  enable_misc_marker: true                   # 显示其他标记
```

生产环境可以关闭部分以节省计算资源。

---

## 参数调优流程

### 1. 基础设置
- 先按照AGV建议值设置所有参数
- 设置所有debug选项为true

### 2. 测试场景
- 静态障碍物绕避
- 慢速移动障碍物
- 行人横穿
- 停放车辆

### 3. 观察与调整

| 现象 | 可能原因 | 调整参数 |
|------|----------|----------|
| 绕避动作太早 | 检测距离太远 | 减小 `max_forward_distance` |
| 绕避动作太晚 | 准备距离不足 | 增加 `min_prepare_distance` |
| 无法完成绕避 | 横向偏移不够 | 增加 `max_shift_length` |
| 绕避不平滑 | Jerk限制太大 | 减小 `max_jerk_values` |
| 横向裕度不足 | 安全距离太小 | 增加 `soft_margin` |
| 识别不到停放车辆 | 阈值太高 | 降低 `th_shiftable_ratio` |
| 行人保护不足 | 安全距离太小 | 增加行人的 `longitudinal_margin` 和 `soft_margin` |

### 4. 性能优化
- 检测距离适当缩小
- 降低采样密度（在保证性能前提下）
- 关闭不必要的debug标记

---

## 常见问题

### Q1: AGV速度很慢但还是检测不到障碍物？
**A**: 检查 `min_velocity` 是否设置太高，建议降到 0.5 m/s

### Q2: 行人横穿时AGV反应太慢？
**A**: 
1. 降低 `pedestrian.th_moving_speed` 到 0.2 m/s
2. 缩短 `pedestrian.th_moving_time` 到 0.8s
3. 增加 `time_horizon_for_front_object` 到 4-5s

### Q3: 绕避过程不流畅，有顿挫感？
**A**: 
1. 降低 `max_jerk_values` (横向和纵向)
2. 增加 `min_prepare_time`
3. 检查 `nominal_avoidance_speed` 是否太高

### Q4: 识别停放车辆不准确？
**A**: 根据实际情况调整：
- 道路宽：增加 `th_offset_from_centerline`
- 道路窄：降低到 0.6-0.8m
- 调整 `th_shiftable_ratio` (0.6-0.8)

### Q5: 需要人工批准太频繁？
**A**: 
1. 将策略改为 `"auto"`
2. 设置 `detection_reliability: "reliable"`
3. 确保传感器标定准确

---

## 文件位置

- **默认配置**: `static_obstacle_avoidance.param.yaml`
- **AGV优化配置**: `agv_static_obstacle_avoidance.param.yaml`

使用AGV配置时，在launch文件中指定：
```xml
<param name="config_file" value="$(find-pkg-share ...)/config/agv_static_obstacle_avoidance.param.yaml"/>
```

---

## 总结

### AGV关键优化点
1. ✅ **降低速度相关阈值** (min_velocity, max_velocity)
2. ✅ **增加行人安全距离** (longitudinal_margin, soft_margin)
3. ✅ **缩短检测范围** (max_forward_distance)
4. ✅ **增加准备时间和距离** (min_prepare_time, min_prepare_distance)
5. ✅ **降低动力学约束** (加速度、jerk)
6. ✅ **启用自动策略** (policy: "auto")
7. ✅ **增加预测时间** (time_horizon)

### 安全第一原则
- 宁可保守（增加安全距离）
- 不可激进（减少安全距离）
- 行人 > 自行车 > 车辆（优先级）

### 性能平衡
在确保安全的前提下：
- 适当减小检测范围
- 适当降低采样密度
- 关闭不必要的可视化
