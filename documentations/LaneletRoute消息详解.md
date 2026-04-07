# LaneletRoute 消息详解

## 📋 消息定义

**文件位置**: `autoware_planning_msgs/msg/LaneletRoute.msg`

```msg
std_msgs/Header header
geometry_msgs/Pose start_pose
geometry_msgs/Pose goal_pose
autoware_planning_msgs/LaneletSegment[] segments
unique_identifier_msgs/UUID uuid
bool allow_modification
```

---

## 🎯 作用和功能

### 核心作用

`LaneletRoute` 是 **Autoware 全局路径规划的核心消息**，它定义了车辆从起点到终点应该经过的 **Lanelet（车道单元）序列**。

### 在 Planning 流程中的位置

```
Mission Planning (全局规划)
    ↓
 【LaneletRoute】 ← 本消息
    ↓
Behavior Planning (行为规划)
    ↓
Motion Planning (运动规划)
    ↓
Control (控制)
```

---

## 📊 字段详解

### 1. `std_msgs/Header header`

**标准 ROS 消息头**

```cpp
struct Header {
  builtin_interfaces/Time stamp;  // 时间戳
  string frame_id;                 // 坐标系 (通常是 "map")
}
```

**用途**:
- 记录路径生成的时间
- 指定坐标系（通常是全局地图坐标系 "map"）

**示例**:
```yaml
header:
  stamp:
    sec: 1698765432
    nanosec: 123456789
  frame_id: "map"
```

---

### 2. `geometry_msgs/Pose start_pose`

**路径起点位姿**

```cpp
struct Pose {
  Point position;      // x, y, z 坐标
  Quaternion orientation; // 四元数表示的朝向
}
```

**用途**:
- 记录路径规划的起点位置
- 通常是车辆当前位置或用户指定的起点

**示例**:
```yaml
start_pose:
  position:
    x: 100.0
    y: 200.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0  # 朝向正东
```

**在 AGV 中的应用**:
- 室内 AGV 通常从当前充电桩位置开始
- 或从上一个任务的结束点开始

---

### 3. `geometry_msgs/Pose goal_pose`

**路径终点位姿**

**用途**:
- 记录路径规划的目标位置
- 用户通过 RViz "2D Goal Pose" 设置的目标点

**示例**:
```yaml
goal_pose:
  position:
    x: 500.0
    y: 300.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707  # 朝向东北 45°
```

**重要性**:
- Goal Planner 会使用这个位姿进行停车规划
- 车辆最终需要尽可能接近这个位姿（位置和朝向）

---

### 4. `autoware_planning_msgs/LaneletSegment[] segments` ⭐⭐⭐

**最重要的字段：Lanelet 路径段序列**

这是一个数组，包含从起点到终点经过的所有 Lanelet 段。

#### LaneletSegment 的定义

```msg
# LaneletSegment.msg
autoware_planning_msgs/LaneletPrimitive preferred_primitive
autoware_planning_msgs/LaneletPrimitive[] primitives
```

#### LaneletPrimitive 的定义

```msg
# LaneletPrimitive.msg
int64 id                    # Lanelet ID
string primitive_type       # "lane", "lane_change_left", "lane_change_right" 等
```

**用途**:
- **定义车辆应该行驶的车道序列**
- 每个 segment 包含一个或多个可选的 lanelet
- `preferred_primitive` 是推荐的 lanelet
- `primitives[]` 是所有可选的 lanelet（用于换道等场景）

**示例**:
```yaml
segments:
  - preferred_primitive:
      id: 1001
      primitive_type: "lane"
    primitives:
      - id: 1001
        primitive_type: "lane"
      - id: 1002  # 左侧车道（可选）
        primitive_type: "lane"
  
  - preferred_primitive:
      id: 1003
      primitive_type: "lane"
    primitives:
      - id: 1003
        primitive_type: "lane"
```

**关键作用**:
1. ✅ **速度限制来源**: 每个 lanelet 有速度限制属性
2. ✅ **路径中心线**: Lanelet 的中心线是路径规划的基础
3. ✅ **车道约束**: 限制车辆在特定车道内行驶
4. ✅ **换道决策**: 提供可选的相邻车道信息

---

### 5. `unique_identifier_msgs/UUID uuid`

**路径的唯一标识符**

```cpp
struct UUID {
  uint8[16] uuid;  // 128位 UUID
}
```

**用途**:
- 唯一标识每一条路径
- 用于追踪和调试
- 检测路径是否更新

**示例**:
```yaml
uuid:
  uuid: [0x12, 0x34, 0x56, 0x78, ...]  # 16字节
```

**实际应用**:
```cpp
// 检查路径是否更新
if (new_route.uuid != current_route.uuid) {
  // 路径已更新，重新规划
  updatePath();
}
```

---

### 6. `bool allow_modification`

**是否允许路径修改标志**

**用途**:
- 指示 Behavior Planner 是否可以修改这条路径
- `true`: 允许避障、换道等修改
- `false`: 严格按照路径行驶，不允许偏离

**应用场景**:

| 场景 | allow_modification | 原因 |
|------|-------------------|------|
| 普通道路行驶 | `true` | 允许动态避障、换道 |
| 窄通道 | `false` | 不允许偏离，必须沿中心线 |
| 停车入库 | `false` | 精确路径，不允许修改 |
| 紧急路径 | `false` | 不允许任何偏离 |

**AGV 应用**:
```cpp
// 室内 AGV 在货架区域
route.allow_modification = false;  // 禁止偏离，避免碰撞货架

// 室内 AGV 在开阔区域
route.allow_modification = true;   // 允许动态避障
```

---

## 🔄 消息流转过程

### 1. 路径请求

用户通过 RViz 设置目标点，或通过 API 发送路径请求：

```bash
# RViz 中点击 "2D Goal Pose"
# 或通过命令行
ros2 topic pub /planning/mission_planning/goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {...}}"
```

### 2. Mission Planning 生成 Route

**节点**: `mission_planner`

```cpp
// 伪代码
LaneletRoute generateRoute(
  Pose start_pose,      // 当前位置
  Pose goal_pose,       // 目标位置
  LaneletMap map        // Lanelet2 地图
) {
  // 1. 在地图上查找起点和终点对应的 lanelet
  auto start_lanelet = map.findNearestLanelet(start_pose);
  auto goal_lanelet = map.findNearestLanelet(goal_pose);
  
  // 2. 使用 A* 或 Dijkstra 算法搜索路径
  auto lanelet_sequence = routeSearch(start_lanelet, goal_lanelet);
  
  // 3. 构建 LaneletRoute 消息
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose = start_pose;
  route.goal_pose = goal_pose;
  
  for (auto lanelet : lanelet_sequence) {
    LaneletSegment segment;
    segment.preferred_primitive.id = lanelet.id();
    route.segments.push_back(segment);
  }
  
  route.allow_modification = true;
  return route;
}
```

### 3. 发布到话题

```bash
# 话题名称
/planning/mission_planning/route

# 消息类型
autoware_planning_msgs/msg/LaneletRoute
```

### 4. Behavior Path Planner 订阅

**节点**: `behavior_path_planner`

```cpp
// behavior_path_planner_node.cpp
route_subscriber_ = create_subscription<LaneletRoute>(
  "~/input/route",
  rclcpp::QoS{1}.transient_local(),
  [this](const LaneletRoute::SharedPtr msg) {
    route_ptr_ = msg;
    has_received_route_ = true;
    
    // 基于 route 生成详细路径
    generatePath();
  }
);
```

---

## 📍 在 Planning 模块中的使用

### 1. 生成参考路径

```cpp
// Behavior Path Planner 使用 route
PathWithLaneId generateReferencePath(const LaneletRoute & route) {
  PathWithLaneId path;
  
  for (const auto & segment : route.segments) {
    // 获取 lanelet 的中心线
    auto lanelet = map_->laneletLayer.get(segment.preferred_primitive.id);
    auto centerline = lanelet.centerline();
    
    // 添加到路径
    for (const auto & point : centerline) {
      path.points.push_back(convertToPathPoint(point));
    }
  }
  
  return path;
}
```

### 2. 获取速度限制

```cpp
// 从 route 获取速度限制
double getSpeedLimit(const LaneletRoute & route, size_t segment_idx) {
  auto lanelet_id = route.segments[segment_idx].preferred_primitive.id;
  auto lanelet = map_->laneletLayer.get(lanelet_id);
  
  // 读取 lanelet 的速度限制属性
  auto speed_limit_attr = lanelet.attribute("speed_limit");
  if (speed_limit_attr) {
    return std::stod(speed_limit_attr.value());
  }
  
  return default_speed_limit_;  // 默认速度
}
```

### 3. 检查路径有效性

```cpp
bool isRouteValid(const LaneletRoute & route) {
  if (route.segments.empty()) {
    RCLCPP_ERROR(logger_, "Route has no segments!");
    return false;
  }
  
  // 检查 lanelet 连通性
  for (size_t i = 0; i < route.segments.size() - 1; ++i) {
    auto current = route.segments[i].preferred_primitive.id;
    auto next = route.segments[i + 1].preferred_primitive.id;
    
    if (!isConnected(current, next)) {
      RCLCPP_ERROR(logger_, "Lanelet %ld and %ld are not connected!", current, next);
      return false;
    }
  }
  
  return true;
}
```

---

## 🛠️ 调试和查看

### 1. 查看当前路径

```bash
# 查看完整消息
ros2 topic echo /planning/mission_planning/route

# 仅查看 segments
ros2 topic echo /planning/mission_planning/route/segments

# 查看起点和终点
ros2 topic echo /planning/mission_planning/route | grep -A5 "start_pose\|goal_pose"
```

### 2. 可视化路径

在 RViz 中：
1. Add → By topic → `/planning/mission_planning/route` → `LaneletRoute`
2. 路径会显示为一系列连接的车道

### 3. 检查路径更新

```bash
# 监控路径变化
ros2 topic hz /planning/mission_planning/route

# 查看路径 UUID 变化
watch -n 1 'ros2 topic echo --once /planning/mission_planning/route | grep -A1 uuid'
```

---

## 🎯 AGV 应用示例

### 室内 AGV 的 LaneletRoute

```yaml
header:
  stamp: {sec: 1698765432, nanosec: 0}
  frame_id: "map"

start_pose:
  position: {x: 10.0, y: 5.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

goal_pose:
  position: {x: 50.0, y: 20.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}

segments:
  # 主通道
  - preferred_primitive:
      id: 100
      primitive_type: "lane"
    primitives:
      - {id: 100, primitive_type: "lane"}
  
  # 转弯区域
  - preferred_primitive:
      id: 101
      primitive_type: "lane"
    primitives:
      - {id: 101, primitive_type: "lane"}
  
  # 货架区走廊（窄通道）
  - preferred_primitive:
      id: 102
      primitive_type: "lane"
    primitives:
      - {id: 102, primitive_type: "lane"}
  
  # 目标区域
  - preferred_primitive:
      id: 103
      primitive_type: "lane"
    primitives:
      - {id: 103, primitive_type: "lane"}

uuid:
  uuid: [0xAB, 0xCD, 0xEF, ...]

allow_modification: true  # 允许在开阔区域避障
```

---

## 🔍 与速度的关系

### Route 如何影响速度

1. **Lanelet 速度限制**:
   ```cpp
   // 每个 lanelet 有速度属性
   <tag k="speed_limit" v="1.5"/>  // 1.5 m/s
   ```

2. **在 Behavior Planner 中应用**:
   ```cpp
   for (size_t i = 0; i < path.points.size(); ++i) {
     // 找到路径点所在的 lanelet
     auto lanelet_id = findLaneletId(path.points[i]);
     
     // 获取该 lanelet 的速度限制
     auto speed_limit = getSpeedLimit(lanelet_id);
     
     // 应用到路径点
     path.points[i].longitudinal_velocity_mps = 
       std::min(path.points[i].longitudinal_velocity_mps, speed_limit);
   }
   ```

3. **速度为零的可能原因**:
   - ✅ Route 中的 lanelet 速度限制为 0
   - ✅ Route 无效或为空
   - ✅ 起点/终点不在有效 lanelet 上

---

## 🐛 常见问题

### 1. Route 为空

**症状**: `route.segments` 是空数组

**原因**:
- 起点或终点不在地图的 lanelet 上
- 起点和终点之间没有连通路径
- 地图未正确加载

**解决方案**:
```bash
# 检查地图是否加载
ros2 topic echo /map/vector_map

# 检查起点和终点是否在 lanelet 上
ros2 run autoware_launch check_lanelet_pose \
  --pose "x y z roll pitch yaw"
```

### 2. Route 速度限制不生效

**症状**: 车辆速度不受 lanelet 速度限制影响

**原因**:
- Lanelet 地图中未设置 `speed_limit` 属性
- Behavior Planner 未正确读取速度限制

**解决方案**:
参考 `lanelet地图速度限制参数说明.md`，确保地图中有：
```xml
<tag k="speed_limit" v="1.5"/>
<tag k="speed_limit_mandatory" v="true"/>
```

### 3. allow_modification 不起作用

**症状**: 即使设为 false，车辆仍然偏离路径

**原因**:
- Behavior Planner 可能不支持这个标志
- 避障模块优先级更高

**调试**:
```bash
# 查看当前 route 的 allow_modification
ros2 topic echo /planning/mission_planning/route | grep allow_modification

# 检查 Behavior Planner 是否使用了这个标志
ros2 topic echo /rosout | grep "allow_modification"
```

---

## 📚 相关文档

- **Lanelet2 速度限制**: `lanelet地图速度限制参数说明.md`
- **Planning 架构**: `PLANNING_ARCHITECTURE_DIAGRAM.md`
- **BPP 工作原理**: `behavior_path_planner场景模块工作原理详解.md`
- **速度调试**: `速度调试日志已添加总结.md`

---

## 🎯 总结

### 关键要点

1. ✅ **LaneletRoute 是全局路径的核心**
   - 定义了从起点到终点的 lanelet 序列
   - 是 Behavior Planning 的输入

2. ✅ **segments 是最重要的字段**
   - 包含路径的所有 lanelet
   - 每个 lanelet 有速度限制和几何信息

3. ✅ **影响车辆速度的方式**
   - Lanelet 的 speed_limit 属性
   - 路径的连通性和有效性
   - allow_modification 标志

4. ✅ **调试技巧**
   - 使用 `ros2 topic echo` 查看路径
   - 在 RViz 中可视化
   - 检查 UUID 确认路径更新

### 在速度调试中的作用

当您调试速度为零问题时，应该：
1. 检查 Route 是否有效（segments 非空）
2. 查看每个 lanelet 的速度限制
3. 确认起点和终点在有效 lanelet 上
4. 验证路径连通性

**查看命令**:
```bash
# 快速检查 route
ros2 topic echo --once /planning/mission_planning/route | head -30
```

祝调试顺利！🚀

