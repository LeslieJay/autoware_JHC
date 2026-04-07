# 静态障碍物避让模块 README 详细解释

## 目录
1. [模块概述](#模块概述)
2. [整体流程图详解](#整体流程图详解)
3. [目标过滤流程图详解](#目标过滤流程图详解)
4. [路径生成图表详解](#路径生成图表详解)
5. [安全检查流程图详解](#安全检查流程图详解)
6. [关键概念图解](#关键概念图解)

---

## 模块概述

静态障碍物避让模块是一个**基于规则（rule-based）**的行为层避让模块，专门处理**静止（stopped）目标**的避让。它工作在行为规划层，能够考虑车道结构和交通规则。

### 核心特点

- **运行模式**：支持 MANUAL（手动批准）和 AUTO（自动执行）两种模式
- **目标类型**：仅处理静态障碍物，动态障碍物由动态避障模块处理
- **工作层级**：行为规划层，可以处理跨车道、转向灯等交通规则相关行为

---

## 整体流程图详解

### 流程图结构

README 中的整体流程图（第 62-237 行）展示了模块的三个主要执行阶段：

#### 1. updateData() 阶段

```
┌─────────────────────────────────────┐
│   fillFundamentalData()             │
│   - 更新参考姿态和路径              │
│   - 更新当前车道信息                │
│   - 计算避让起点和返回点            │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│   fillAvoidanceTargetObjects()      │
│   - 检查目标对象类型                │
│   - 检查是否静止                    │
│   - 检查是否在自车车道附近          │
│   - 检查是否在车道边缘              │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│   updateRegisteredObject()          │
│   compensateDetectionLost()         │
└─────────────────────────────────────┘
```

**关键点解释**：
- **参考路径**：通过样条插值生成密集间隔的中心线路径
- **避让起点**：根据交通信号计算自车应该开始避让的位置
- **返回点**：根据交通信号和目标位置计算应该返回原车道的位置
- **检测丢失补偿**：即使目标暂时消失，也会保持一段时间，防止抖动

#### 2. fillShiftLine() 阶段

```
┌─────────────────────────────────────┐
│   生成 shift line（横向移线）       │
│   - 基于过滤后的目标对象            │
│   - 计算避让和返回的横向偏移        │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│   创建候选路径                      │
│   - 使用 Path Shifter 生成平滑路径  │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│   检查候选路径                      │
│   - 检查与周围移动车辆的距离        │
│   - 检查路径是否平滑（jerk）        │
│   - 检查路径是否在可行驶区域内      │
└─────────────────────────────────────┘
```

#### 3. fillEgoStatus() 阶段

```
┌─────────────────────────────────────┐
│   获取当前模块状态                  │
│   - RUNNING: 目标仍在，或未返回     │
│   - CANCEL: 目标消失且未开始避让    │
│   - SUCCEEDED: 完成避让并返回       │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│   判断是否可以执行让行              │
│   ↓                                 │
│   如果路径不安全 → 执行让行         │
│   （仅在未开始避让时）              │
└─────────────────────────────────────┘
```

---

## 目标过滤流程图详解

### 1. 检测区域（Detection Area）

<img src="./images/target_filter/detection_area.svg" width="800" height="400"/>
<img src="./images/target_filter/detection_area_rviz.png" width="800" height="400"/>

**概念解释**：
- 检测区域是一个**空间过滤区域**，用于初步筛选可能的目标对象
- 宽度计算：`检测区域宽度 = 自车宽度 + 最大横向裕度`
- 纵向距离：
  - 如果 `detection_area.static = true`：固定为 `max_forward_distance`
  - 如果 `detection_area.static = false`：动态计算，考虑：
    - 最大横向偏移长度
    - 横向 jerk 约束
    - 当前自车速度
    - 准备阶段所需距离

**公式**：
```cpp
动态距离 = 1.5 * 动态计算距离 + 标称准备距离
最终距离 = clamp(动态距离, min_forward_distance, max_forward_distance)
```

### 2. 车辆行为判断流程图

<img src="./images/target_filter/merging.png" width="800" height="400"/>
<img src="./images/target_filter/deviating.png" width="800" height="400"/>
<img src="./images/target_filter/none.png" width="800" height="400"/>

**三种行为类型**：

#### NONE（无特殊行为）
- **条件**：车辆相对车道的偏航角小于阈值 `yaw_deviation`
- **含义**：车辆基本平行于车道，没有明显的汇入或偏离行为

#### MERGING（汇入）
- **判断逻辑**：
  ```
  如果对象在路径右侧：
    - 如果角度 < 0 且 > -π/2 → DEVIATING
    - 如果角度 > π/2 → DEVIATING
    - 否则 → MERGING
  
  如果对象在路径左侧：
    - 如果角度 > 0 且 < π/2 → DEVIATING
    - 如果角度 < -π/2 → DEVIATING
    - 否则 → MERGING
  ```
- **含义**：车辆正在汇入自车车道

#### DEVIATING（偏离）
- **含义**：车辆正在从自车车道偏离到其他车道

### 3. 泊车车辆判断

<img src="./images/target_filter/parked_vehicle.svg" width="800" height="400"/>

**判断公式**：
```
可移动距离 Ld = (车道宽度 - 对象宽度) / 2
实际偏移距离 La = 从中心线到对象的距离
比例 ratio = La / Ld

如果 ratio > th_shiftable_ratio → 判定为泊车车辆
```

**图解**：
```
┌─────────────────────────────────────┐
│        车道边界                     │
│    ┌─────────────────────────┐     │
│    │  车道中心线              │     │
│    │        │                │     │
│    │        │  对象          │     │
│    │        │  (La)          │     │
│    └────────┴────────────────┘     │
│         (Ld)                        │
└─────────────────────────────────────┘
```

### 4. 目标过滤决策树

**流程图位置**：README 第 421-464 行

**决策流程**：

```
开始
  ↓
┌─────────────────────────────────────┐
│ isSatisfiedWithCommonCondition()   │
│ ✓ 是否在检测区域内？                │
│ ✓ 是否是避让目标类型？              │
│ ✓ 是否静止？                        │
│ ✓ 是否在前后距离阈值内？            │
└─────────────────────────────────────┘
  ↓ (通过)
┌─────────────────────────────────────┐
│ isNoNeedAvoidanceBehavior()         │
│ 检查是否真的需要避让                │
└─────────────────────────────────────┘
  ↓ (需要避让)
┌─────────────────────────────────────┐
│ 对象类型判断                        │
│ ├─ UNKNOWN → 特殊条件检查           │
│ ├─ 车辆类型 → isSatisfiedWithVehicleCondition()
│ └─ 非车辆类型 → isSatisfiedWithNonVehicleCondition()
└─────────────────────────────────────┘
```

**车辆类型过滤详细流程**：

```
isSatisfiedWithVehicleCondition()
  ↓
┌─────────────────────────────────────┐
│ isNeverAvoidanceTarget()            │
│ ✗ 在路口内且平行/汇入？             │
│ ✗ 汇入自车车道且悬垂距离大？        │
│ ✗ 偏离自车车道且悬垂距离大？        │
│ ✗ 在自车车道但不在边缘车道？        │
│ ✗ 靠近停止因素（红绿灯/人行横道）？  │
└─────────────────────────────────────┘
  ↓ (不是明确不避让)
┌─────────────────────────────────────┐
│ isObviousAvoidanceTarget()          │
│ ✓ 在自由空间内且不在自车车道？      │
│ ✓ 在路口外且是泊车车辆？            │
│ ✓ 平行于车道且不在自车车道？        │
└─────────────────────────────────────┘
  ↓ (不是明确避让目标)
┌─────────────────────────────────────┐
│ 模糊车辆判断                        │
│ - 停止时间 > 阈值？                 │
│ - 在路口内且偏离？                  │
│ - 汇入自车车道？                    │
│ - 偏离自车车道？                    │
│ - 平行于车道？                      │
└─────────────────────────────────────┘
```

---

## 路径生成图表详解

### 1. 包络多边形（Envelope Polygon）

<img src="./images/path_generation/envelope_polygon.png" width="800" height="400"/>
<img src="./images/path_generation/envelope_polygon_rviz.png" width="800" height="400"/>

**目的**：减少感知噪声对路径生成的影响

**生成过程**：

```
原始对象多边形
    ↓
添加缓冲区 (envelope_buffer_margin)
    ↓
生成矩形包络（始终平行于参考路径）
    ↓
根据姿态协方差更新
    ↓
最终包络多边形
```

**更新逻辑**：

<img src="./images/path_generation/polygon_update.png" width="800" height="400"/>

```
如果姿态协方差 < 阈值：
  - 如果新包络不在旧包络内 → 创建新包络
  - 否则 → 保持旧包络

如果姿态协方差 >= 阈值：
  - 如果 < 最大协方差 → 使用新包络
  - 否则 → 保持旧包络
```

**效果**：即使感知输出包含噪声，包络多边形的大小和位置也会收敛到稳定值。

### 2. 横向裕度（Lateral Margin）

<img src="./images/path_generation/margin.png" width="800" height="400"/>
<img src="./images/path_generation/soft_hard.png" width="800" height="400"/>
<img src="./images/path_generation/hard_margin.png" width="1200" height="400"/>

**两种裕度类型**：

#### Soft Margin（软约束）
- **作用**：理想情况下希望保持的距离
- **特点**：可以缩短，范围：0.0 ~ `soft_margin`
- **使用场景**：当空间不足时，可以压缩软约束以保持在可行驶区域内

#### Hard Margin（硬约束）
- **作用**：必须保持的最小距离
- **特点**：不可压缩，必须严格遵守
- **两种值**：
  - `hard_margin`：普通车辆
  - `hard_margin_for_parked_vehicle`：泊车车辆（通常更大，考虑车门突然打开）

**图解**：

```
┌─────────────────────────────────────┐
│  对象                                │
│  │                                  │
│  │ hard_margin (必须保持)           │
│  │                                  │
│  │ soft_margin (可以压缩)           │
│  │                                  │
│  └─ 自车路径                         │
└─────────────────────────────────────┘

总横向距离 = hard_margin + soft_margin (理想情况)
最小横向距离 = hard_margin (空间不足时)
```

**调整裕度场景**：

<img src="./images/path_generation/adjust_margin.png" width="800" height="400"/>

当避让会导致进入对向车道时（`use_lane_type = same_direction_lane`），模块会缩短软约束，确保不进入对向车道。

### 3. 纵向裕度（Longitudinal Margin）

<img src="./images/path_generation/margin.png" width="800" height="400"/>

**计算方式**：

```
前纵向缓冲区 = 车头悬长 + longitudinal_margin (如果 consider_front_overhang = true)
             或
            = longitudinal_margin (如果 consider_front_overhang = false)

后纵向缓冲区 = 车尾悬长 + longitudinal_margin (如果 consider_rear_overhang = true)
             或
            = longitudinal_margin (如果 consider_rear_overhang = false)
```

**图解**：

```
对象包络多边形
    ↓
┌─────────────────────────────────────┐
│  前纵向缓冲区                        │
│  (车头悬长 + longitudinal_margin)   │
│                                      │
│  ┌──────────────┐                   │
│  │  对象包络    │                   │
│  └──────────────┘                   │
│                                      │
│  后纵向缓冲区                        │
│  (车尾悬长 + longitudinal_margin)   │
└─────────────────────────────────────┘
    ↓
避让段终点和返回段起点
```

### 4. 必须避让 vs 可以通过

<img src="./images/path_generation/must_avoid.png" width="800" height="400"/>
<img src="./images/path_generation/pass_through.png" width="800" height="400"/>

#### 必须避让场景

**条件**：横向距离 < `hard_margin` / `hard_margin_for_parked_vehicle`

**行为**：
- 必须执行避让
- 如果等待批准，插入停车点
- 自车保持停止直到避让被批准

#### 可以通过场景（pass_through.png）

**条件**：横向距离 > `hard_margin` / `hard_margin_for_parked_vehicle`

**行为**：
- 可以不执行避让
- 即使等待批准，也不插入停车点
- 认为可以在不避让的情况下安全通过

### 5. 空间不足场景

<img src="./images/path_generation/do_nothing.png" width="800" height="400"/>
<img src="./images/path_generation/insufficient_drivable_space.png" width="800" height="400"/>

**条件**：无法保持 `hard_margin` / `hard_margin_for_parked_vehicle` 的距离

**行为**：
- **不生成避让路径**
- **不插入停车点**
- 由下游的 Obstacle Cruise Planner 负责处理

**原因**：如果硬约束都无法满足，说明空间严重不足，避让模块无法安全处理，需要其他模块介入。

### 6. 横向偏移长度计算

<img src="./images/path_generation/lateral.png" width="800" height="400"/>

**计算公式**：

```
基础偏移长度 = 悬垂距离 + 横向裕度 + 0.5 * 自车宽度

限制条件：
- 不能接近可行驶边界 < soft_drivable_bound_margin
- 在狭窄道路上可以放宽到 hard_drivable_bound_margin
```

**可使用的车道类型**  
<img src="images/path_generation/opposite_direction.png" width="800" height="400"/>
<img src="images/path_generation/same_direction.png" width="800" height="400"/>

- `current_lane`：仅使用当前车道
- `same_direction_lane`：可以使用同向车道
- `opposite_direction_lane`：可以使用对向车道

### 7. Shift Line 生成

**图表位置**：  
<img src="images/path_generation/shift_line.png" width="800" height="400"/>

**固定点**：
- 避让段终点：基于包络多边形计算
- 返回段起点：基于包络多边形计算

**动态点**：
- 避让段起点：根据偏移长度、当前速度、横向 jerk 约束计算
- 返回段终点：根据偏移长度、当前速度、横向 jerk 约束计算

**纵向距离计算公式**：

```cpp
double PathShifter::calcLongitudinalDistFromJerk(
  const double lateral,    // 横向偏移
  const double jerk,        // 横向 jerk
  const double velocity)   // 速度
{
  return 4.0 * pow(0.5 * lateral / jerk, 1.0/3.0) * velocity;
}
```

**准备距离约束**：

```
准备长度 = max(自车速度 * max_prepare_time, min_prepare_distance)

目的：确保转向灯开启足够时间后才开始避让
```

---

## 安全检查流程图详解

### 安全检查流程

**图表位置**：  
<img src="images/safety_check/safety_check_flow.png" width="800" height="400"/>

**流程**：

```
生成避让路径
    ↓
检查路径安全性
    ↓
┌─────────────────────────────────────┐
│ 路径是否安全？                       │
│ ├─ 是 → 执行避让                    │
│ └─ 否 → 执行让行                    │
└─────────────────────────────────────┘
```

**检查内容**：
- 避让目标对象本身
- 避让路径附近的非目标对象
- 周围移动车辆

### 让行行为（Yield Maneuver）

**图表位置**：  
<img src="images/safety_check/stop.png" width="800" height="400"/>
<img src="images/safety_check/not_stop.png" width="800" height="400"/>

#### 需要停车让行  
<img src="images/safety_check/stop.png" width="800" height="400"/>

**条件**：
- 路径不安全
- 横向距离 < `hard_margin` / `hard_margin_for_parked_vehicle`

**行为**：
- 回退避让路径
- 在目标前方插入停车点
- 等待直到可以安全避让

#### 不需要停车  
<img src="images/safety_check/not_stop.png" width="800" height="400"/>

**条件**：
- 横向距离 > `hard_margin` / `hard_margin_for_parked_vehicle`

**行为**：
- 不插入停车点
- 认为可以在不避让的情况下安全通过

### 安全检查车道选择

**参数配置**：

```yaml
safety_check:
  check_current_lane: false      # 检查当前车道
  check_shift_side_lane: true     # 检查避让侧车道（推荐）
  check_other_side_lane: false    # 检查另一侧车道
```

**建议配置**：仅检查避让侧车道，减少误触发。

---

## 红绿灯场景处理

### 图表位置
- <img src="images/traffic_light/traffic_light.png" width="800" height="400"/>：整体场景
- <img src="images/traffic_light/limit_shift_length.png" width="800" height="400"/>：限制偏移长度
- <img src="images/traffic_light/shift_from_current_pos.png" width="800" height="400"/>
  <img src="images/traffic_light/shift_from_stop_line.png" width="800" height="400"/>：避让起点控制
- <img src="images/traffic_light/return_after_stop_line.png" width="800" height="400"/>
  <img src="images/traffic_light/return_before_stop_line.png" width="800" height="400"/>：返回终点控制

### 控制策略

#### 1. 控制偏移长度

**场景**：红灯且未开始避让

**行为**：
- 限制最大偏移长度
- **仅使用当前车道**
- 防止阻塞其他车辆

#### 2. 控制避让起点

**场景**：目标对象在停止线之后

**行为**：
- 将避让起点设置在停止线上
- 防止在避让过程中停在红灯前

#### 3. 控制返回终点

**场景**：已开始避让

**行为**：
- 尝试将返回终点设置在停止线上
- 确保在停止线前完成返回

---

## 路径形状决定流程

### 单障碍物场景

**图表位置**：  
<img src="images/how_to_decide_path_shape_one_object.drawio.svg"/>

**流程**：

```
计算横向偏移距离
    ↓
根据速度和横向 jerk 生成 shift point
    ↓
基于 shift point 生成平滑避让路径
```

**特殊情况处理**：

1. **横向 jerk 放宽条件**：
   - 自车接近避让目标
   - 返回时距离目标不足

2. **最小速度放宽条件**：
   - 自车速度 < 标称最小速度 → 使用最小速度计算
   - 自车速度 < 急转最小速度且标称 jerk 不足 → 使用急转最小速度

### 多障碍物场景（同向）

**图表位置**：  
<img src="images/how_to_decide_path_shape_multi_object_one_direction.drawio.svg"/>

**流程**：

```
为每个障碍物生成 shift point
    ↓
合并重叠的 shift point（选择最大偏移值）
    ↓
过滤 shift point
    ↓
生成最终避让路径
```

**合并规则**：
- 检查每个路径点上的偏移长度
- 如果 shift point 重叠，选择同方向的最大偏移值

### 多障碍物场景（双向）

**图表位置**：  
<img src="images/how_to_decide_path_shape_multi_object_both_direction.drawio.svg"/>

**流程**：

```
为每个障碍物生成 shift point
    ↓
合并所有 shift point
    ↓
处理冲突区域（不同方向的偏移）
    ↓
使用最大偏移值的总和作为最终偏移
    ↓
过滤 shift point
    ↓
生成最终避让路径
```

**冲突处理**：
- 如果存在不同方向的偏移冲突
- 使用这些区域最大偏移值的总和作为最终偏移

### Shift Point 过滤

**过滤步骤**：

1. **量化（Quantization）**：
   - 量化避让宽度，忽略小幅偏移

2. **小幅偏移移除（Small shifts removal）**：
   - 移除相对于前一个 shift point 变化很小的偏移
   - 统一到前一个偏移宽度

3. **相似梯度移除（Similar gradient removal）**：
   - 用直线连接两个 shift point
   - 移除中间接近直线的 shift point

4. **瞬时返回移除（Remove momentary returns）**：
   - 对于减少避让宽度的 shift point（返回中心线）
   - 如果纵向距离足够长，移除它们

---

## 关键概念图解

### 1. 取消避让

**图表位置**：  
<img src="images/cancel/cancel.png" width="800" height="400"/>

**条件**：
- 所有目标对象已消失
- 自车尚未开始避让

**行为**：回退避让路径，返回原始路径

### 2. 可行驶区域扩展

**图表位置**：  
*位于* `images/advanced/` *目录下的各种场景图*

**支持的区域**：
- **路口区域**  
  <img src="images/advanced/avoidance_intersection.png" width="800" height="400"/>：HD Map 中定义的路口区域
- **斑马线区域**  
  <img src="images/advanced/avoidance_zebra.png" width="800" height="400"/>：HD Map 中定义的斑马线标记区域
- **自由空间区域**  
  <img src="images/advanced/avoidance_freespace.png" width="800" height="400"/>：HD Map 中定义的自由空间

**配置**：

```yaml
use_intersection_areas: true
use_hatched_road_markings: true
use_freespace_areas: true
```

### 3. 不同车道类型的使用

- **同向车道**  
  <img src="images/advanced/avoidance_same_direction.png" width="800" height="400"/>：仅使用同向车道避让
- **对向车道**  
  <img src="images/advanced/avoidance_opposite_direction.png" width="800" height="400"/>：可以使用对向车道避让

---

## 图表详细解读

### 1. 目标过滤场景图解读

#### 明确不避让场景

**never_avoid_intersection.png** - 路口内车辆  
<img src="images/target_filter/never_avoid_intersection.png" width="800" height="400"/>
- **场景**：车辆在路口内，且平行于车道或汇入自车车道
- **判断**：路口内不应有泊车车辆，这些车辆可能是正常行驶
- **行为**：永远不避让

**never_avoid_not_edge.png** - 不在边缘车道的车辆  
<img src="images/target_filter/never_avoid_not_edge.png" width="800" height="400"/>
- **场景**：车辆在自车车道上，但两侧都有相邻车道
- **判断**：如果车辆在中间车道，说明不是靠边停车
- **行为**：永远不避让

**never_avoid_deviating.png** - 正在偏离的车辆  
<img src="images/target_filter/never_avoid_deviating.png" width="800" height="400"/>
- **场景**：车辆正在从自车车道偏离到其他车道，且大部分在自车车道上
- **判断**：车辆正在离开，不是障碍
- **行为**：永远不避让

**never_avoid_merging.png** - 正在汇入的车辆  
<img src="images/target_filter/never_avoid_merging.png" width="800" height="400"/>
- **场景**：车辆正在汇入自车车道，且大部分在自车车道上
- **判断**：车辆正在进入，但大部分还在自车车道，可能是误判
- **行为**：永远不避让

**never_avoid_stop_factor.png** - 靠近停止因素的车辆  
<img src="images/target_filter/never_avoid_stop_factor.png" width="800" height="400"/>
- **场景**：车辆停在红绿灯或人行横道前，且不像是泊车车辆
- **判断**：车辆可能在等待信号，不是障碍
- **行为**：永远不避让

#### 明确避让场景

**avoid_on_ego_lane.png** - 自车车道上的泊车车辆  
<img src="images/target_filter/avoid_on_ego_lane.png" width="800" height="400"/>
- **场景**：车辆停在自车车道上，且靠路边停车
- **判断**：明确需要避让的障碍物
- **行为**：立即避让

**avoid_not_on_ego_lane.png** - 相邻车道上的车辆  
<img src="images/target_filter/avoid_not_on_ego_lane.png" width="800" height="400"/>
- **场景**：车辆停在相邻车道上
- **判断**：虽然不在自车车道，但可能影响通行
- **行为**：立即避让

#### 模糊场景

**ambiguous_parallel.png** - 平行于车道的车辆  
<img src="images/target_filter/ambiguous_parallel.png" width="800" height="400"/>
- **场景**：车辆停在自车车道上，但没有明显靠边
- **判断**：可能是故障车辆，也可能是临时停车
- **行为**：如果 `avoidance_for_ambiguous_vehicle.enable = true`，则避让

**ambiguous_merging.png** - 正在汇入的车辆  
<img src="images/target_filter/ambiguous_merging.png" width="800" height="400"/>
- **场景**：车辆正在汇入自车车道
- **判断**：可能是正常汇入，也可能是障碍
- **行为**：如果启用模糊车辆避让，则避让

**ambiguous_deviating.png** - 正在偏离的车辆  
<img src="images/target_filter/ambiguous_deviating.png" width="800" height="400"/>
- **场景**：车辆正在从自车车道偏离
- **判断**：可能是正常变道，也可能是障碍
- **行为**：如果启用模糊车辆避让，则避让

### 2. 路径生成场景图解读

#### 包络多边形更新机制

**polygon_update.png** 展示了包络多边形的更新逻辑：  
<img src="images/polygon_update.png" width="800" height="400"/>

```
时间 T1:
  原始对象位置 → 生成包络多边形 A

时间 T2:
  感知到新位置（可能有噪声）
    ↓
  生成一次性包络多边形 B
    ↓
  比较 B 和 A：
    - 如果 B 在 A 内 → 保持 A（过滤噪声）
    - 如果 B 不在 A 内 → 更新为 B（真实移动）
```

**效果**：即使感知有噪声，包络多边形也会稳定收敛。

#### 软硬裕度调整

**adjust_margin.png** 展示了软裕度的动态调整：  
<img src="images/adjust_margin.png" width="800" height="400"/>

```
场景：避让会导致进入对向车道

初始计算：
  硬裕度 = 0.2m
  软裕度 = 0.3m
  总裕度 = 0.5m

调整后：
  硬裕度 = 0.2m（不变）
  软裕度 = 0.1m（压缩）
  总裕度 = 0.3m（但仍满足硬约束）
```

**目的**：在保持安全的前提下，尽可能不进入对向车道。

#### 必须避让 vs 可以通过

**must_avoid.png** 和 **pass_through.png** 的对比：  
<img src="images/must_avoid.png" width="800" height="400"/>
<img src="images/pass_through.png" width="800" height="400"/>

```
场景 A（must_avoid）：
  横向距离 = 0.15m < hard_margin(0.2m)
  → 必须避让
  → 插入停车点等待批准

场景 B（pass_through）：
  横向距离 = 0.25m > hard_margin(0.2m)
  → 可以不避让
  → 不插入停车点
  → 认为可以安全通过
```

### 3. 安全检查场景图解读

**safety_check_flow.png** 展示了安全检查的完整流程：  
<img src="images/safety_check/safety_check_flow.png" width="800" height="400"/>

```
生成避让路径
    ↓
检查周围对象
    ├─ 当前车道对象
    ├─ 避让侧车道对象
    └─ 另一侧车道对象（可选）
    ↓
评估碰撞风险
    ├─ 使用 RSS 模型
    ├─ 考虑预测路径
    └─ 计算安全距离
    ↓
┌─────────────────────┐
│ 路径是否安全？       │
├─ 是 → 执行避让       │
└─ 否 → 执行让行       │
└─────────────────────┘
```

**stop.png** vs **not_stop.png**：  
<img src="images/safety_check/stop.png" width="800" height="400"/>
<img src="images/safety_check/not_stop.png" width="800" height="400"/>

- **stop.png**：横向距离不足，必须停车等待
- **not_stop.png**：横向距离足够，可以不避让通过

### 4. 红绿灯场景图解读

**traffic_light.png** 展示了整体场景：  
<img src="images/traffic_light/traffic_light.png" width="800" height="400"/>
- 自车在红灯前
- 目标对象在停止线后
- 需要避免在避让过程中停在红灯前

**limit_shift_length.png**：  
<img src="images/traffic_light/limit_shift_length.png" width="800" height="400"/>
- 未开始避让时，限制最大偏移长度
- 仅使用当前车道，避免阻塞其他车辆

**shift_from_stop_line.png**：  
<img src="images/traffic_light/shift_from_stop_line.png" width="800" height="400"/>
- 将避让起点设置在停止线上
- 确保在停止线前完成避让

**return_after_stop_line.png**：  
<img src="images/traffic_light/return_after_stop_line.png" width="800" height="400"/>
- 将返回终点设置在停止线后
- 确保在停止线前完成返回

### 5. 多障碍物场景图解读

#### 同向多障碍物

**how_to_decide_path_shape_multi_object_one_direction.drawio.svg**：  
<img src="images/how_to_decide_path_shape_multi_object_one_direction.drawio.svg" width="800" height="400"/>

```
障碍物1 → 生成 shift point 1
障碍物2 → 生成 shift point 2
障碍物3 → 生成 shift point 3
    ↓
合并重叠的 shift point
    ↓
过滤优化
    ↓
生成最终路径
```

**关键点**：如果多个障碍物在同一方向，选择最大偏移值。

#### 双向多障碍物

**how_to_decide_path_shape_multi_object_both_direction.drawio.svg**：  
<img src="images/how_to_decide_path_shape_multi_object_both_direction.drawio.svg" width="800" height="400"/>

```
左侧障碍物 → 生成左偏移 shift point
右侧障碍物 → 生成右偏移 shift point
    ↓
检测冲突区域
    ↓
使用最大偏移值的总和
    ↓
生成最终路径
```

**关键点**：如果障碍物在不同方向，需要处理冲突，使用偏移值的总和。

## 实现细节补充

### Shift Line 生成流程

根据源代码 `shift_line_generator.cpp`，生成流程如下：

```
1. updateRegisteredRawShiftLines()
   - 更新已注册的原始移位线

2. applyPreProcess()
   - applyCombineProcess(): 组合当前和已注册的移位线
   - addReturnShiftLine(): 添加返回中心线的移位
   - applyFillGapProcess(): 填充移位线之间的间隙
   - applyMergeProcess(): 合并正向和负向移位线

3. generateCandidateShiftLine()
   - generateTotalShiftLine(): 生成连续的移位轮廓
   - extractShiftLinesFromLine(): 提取离散的移位线
   - applyTrimProcess(): 修剪和优化
     - applyQuantizeProcess(): 量化处理
     - applySmallShiftFilter(): 过滤小幅移位
     - applySimilarGradFilter(): 合并相似梯度
```

### 包络多边形更新逻辑

根据 README 描述，更新逻辑如下：

```cpp
// 伪代码
if (pose_covariance < threshold) {
    if (one_shot_polygon NOT within previous_polygon) {
        envelope_polygon = one_shot_polygon;  // 创建新包络
    } else {
        envelope_polygon = previous_polygon;  // 保持旧包络
    }
} else {
    if (pose_covariance < max_covariance) {
        envelope_polygon = one_shot_polygon;  // 使用新包络
    } else {
        envelope_polygon = previous_polygon;  // 保持旧包络
    }
}
```

### 检测区域动态计算

根据 README 中的公式：

```cpp
const auto max_shift_length = std::max(
    std::abs(parameters_->max_right_shift_length), 
    std::abs(parameters_->max_left_shift_length));
    
const auto dynamic_distance =
    PathShifter::calcLongitudinalDistFromJerk(
        max_shift_length, 
        getLateralMinJerkLimit(), 
        speed);

return std::clamp(
    1.5 * dynamic_distance + getNominalPrepareDistance(),
    parameters_->object_check_min_forward_distance,
    parameters_->object_check_max_forward_distance);
```

**解释**：
- 根据最大偏移长度和横向 jerk 约束计算动态距离
- 乘以 1.5 倍安全系数
- 加上标称准备距离
- 限制在最小和最大前向距离之间

## 总结

静态障碍物避让模块通过以下关键机制实现安全避让：

1. **目标过滤**：多层条件筛选，区分明确避让、明确不避让和模糊目标
2. **包络多边形**：减少感知噪声影响，通过更新逻辑稳定收敛
3. **软硬裕度**：在安全性和灵活性之间平衡，硬约束必须满足，软约束可调整
4. **Shift Line 生成**：基于横向 jerk 约束生成平滑路径，支持多障碍物场景
5. **安全检查**：检查非目标对象，必要时执行让行，支持多车道检查
6. **特殊场景处理**：红绿灯、路口、多障碍物等，都有专门的处理逻辑

模块设计追求"易理解、易调参"，在简单场景下表现良好，复杂场景需要结合下游模块和人工审核。

### 关键设计理念

1. **规则驱动**：使用明确的规则判断，易于理解和调试
2. **噪声抑制**：通过包络多边形和检测丢失补偿减少感知噪声影响
3. **安全优先**：硬约束必须满足，软约束可调整
4. **渐进式处理**：从目标过滤到路径生成，逐步细化
5. **状态管理**：通过状态机管理避让生命周期

### 适用场景

- ✅ 简单场景：单障碍物、清晰的道路结构
- ✅ 泊车车辆避让：路边停放的车辆
- ✅ 静态障碍物：静止的行人、自行车等
- ⚠️ 复杂场景：需要结合其他模块
- ❌ 动态障碍物：由动态避障模块处理
