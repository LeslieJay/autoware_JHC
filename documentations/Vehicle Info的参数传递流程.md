---

# 📋 **Planning模块中Vehicle Info的参数传递流程**

## **1. 参数定义源头**

### **配置文件位置**
```bash
# vehicle info参数定义在各车型的描述包中
src/launcher/autoware_launch/vehicle/{vehicle_model}_description/config/vehicle_info.param.yaml
```

**示例车型:**
- `sample_vehicle_description/config/vehicle_info.param.yaml`
- `byd_vehicle_description/config/vehicle_info.param.yaml`
- `awsim_labs_vehicle_description/config/vehicle_info.param.yaml`

**参数内容示例:**
```yaml
/**:
  ros__parameters:
    wheel_radius: 0.383
    wheel_width: 0.235
    wheel_base: 2.79
    wheel_tread: 1.64
    front_overhang: 1.0
    rear_overhang: 1.1
    left_overhang: 0.128
    right_overhang: 0.128
    vehicle_height: 2.5
    max_steer_angle: 0.70
```

---

## **2. Launch文件参数传递链**

### **Level 1: 顶层Launch（autoware.launch.xml）**

```xml
<code_block_to_apply_changes_from>
```

### **Level 2: Planning组件Launch**

```xml
<!-- tier4_planning_component.launch.xml -->
<!-- 第10行：根据vehicle_model变量构造参数文件路径 -->
<arg name="vehicle_param_file" value="$(find-pkg-share $(var vehicle_model)_description)/config/vehicle_info.param.yaml"/>

<!-- 传递给tier4_planning_launch -->
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <arg name="vehicle_param_file" value="$(var vehicle_param_file)"/>
  ...
</include>
```

### **Level 3: Scenario Planning Launch**

```xml
<!-- tier4_planning_launch/launch/scenario_planning/scenario_planning.launch.xml -->
<!-- 传递给各子模块 -->
<include file="...behavior_planning.launch.xml">
  <arg name="vehicle_param_file" value="$(var vehicle_param_file)"/>
</include>
<include file="...motion_planning.launch.xml">
  <arg name="vehicle_param_file" value="$(var vehicle_param_file)"/>
</include>
```

### **Level 4: 具体模块Launch**

**Behavior Planning中的应用:**
```xml
<!-- behavior_planning.launch.xml -->

<!-- Behavior Path Planner -->
<node pkg="autoware_behavior_path_planner" exec="behavior_path_planner" name="behavior_path_planner">
  <param from="$(var common_param_path)"/>
  <param from="$(var vehicle_param_file)"/>  <!-- 这里加载vehicle info -->
  <param from="$(var nearest_search_param_path)"/>
  ...
</node>

<!-- Behavior Velocity Planner -->
<node pkg="autoware_behavior_velocity_planner" exec="behavior_velocity_planner" name="behavior_velocity_planner">
  <param from="$(var common_param_path)"/>
  <param from="$(var vehicle_param_file)"/>  <!-- 这里加载vehicle info -->
  ...
</node>
```

**Motion Planning中的应用:**
```xml
<!-- motion_planning.launch.xml -->

<!-- Obstacle Cruise Planner -->
<node pkg="autoware_obstacle_cruise_planner" exec="obstacle_cruise_planner" name="obstacle_cruise_planner">
  <param from="$(var common_param_path)"/>
  <param from="$(var vehicle_param_file)"/>  <!-- 这里加载vehicle info -->
  ...
</node>

<!-- Path Sampler -->
<node pkg="autoware_path_sampler" exec="path_sampler" name="path_sampler">
  <param from="$(var common_param_path)"/>
  <param from="$(var vehicle_param_file)"/>  <!-- 这里加载vehicle info -->
  ...
</node>

<!-- Velocity Smoother -->
<node pkg="autoware_velocity_smoother" exec="velocity_smoother" name="velocity_smoother">
  <param from="$(var common_param_path)"/>
  <param from="$(var vehicle_param_file)"/>  <!-- 这里加载vehicle info -->
  ...
</node>
```

---

## **3. C++代码中的参数读取**

### **VehicleInfoUtils类的使用**

**在各planning节点的构造函数中:**

```cpp
// 例如：PathSampler节点
PathSampler::PathSampler(const rclcpp::NodeOptions & node_options)
: Node("path_sampler", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  // vehicle_info_现在包含所有车辆参数
}
```

**VehicleInfoUtils的实现:**

```cpp
// vehicle_info_utils.cpp
VehicleInfoUtils::VehicleInfoUtils(rclcpp::Node & node)
{
  // 从ROS参数服务器读取参数
  const auto wheel_radius_m = getParameter<double>(node, "wheel_radius");
  const auto wheel_base_m = getParameter<double>(node, "wheel_base");
  const auto wheel_tread_m = getParameter<double>(node, "wheel_tread");
  // ... 读取所有基础参数
  
  // 创建VehicleInfo对象并计算派生参数
  vehicle_info_ = createVehicleInfo(
    wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m,
    front_overhang_m, rear_overhang_m, left_overhang_m, right_overhang_m,
    vehicle_height_m, max_steer_angle_rad);
}
```

**参数读取机制:**
```cpp
template <class T>
T getParameter(rclcpp::Node & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  
  try {
    return node.declare_parameter<T>(name);
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(node.get_logger(), 
      "Failed to get parameter `%s`, please set it when you launch the node.",
      name.c_str());
    throw;
  }
}
```

---

## **4. 参数传递完整流程图**

```
autoware.launch.xml
    ↓ (vehicle_model="sample_vehicle")
tier4_planning_component.launch.xml
    ↓ (vehicle_param_file="...sample_vehicle_description/config/vehicle_info.param.yaml")
tier4_planning_launch/planning.launch.xml
    ↓ (vehicle_param_file)
scenario_planning.launch.xml
    ├─→ behavior_planning.launch.xml
    │      ├─→ behavior_path_planner node (<param from="vehicle_param_file"/>)
    │      └─→ behavior_velocity_planner node (<param from="vehicle_param_file"/>)
    └─→ motion_planning.launch.xml
           ├─→ obstacle_cruise_planner node (<param from="vehicle_param_file"/>)
           ├─→ path_sampler node (<param from="vehicle_param_file"/>)
           └─→ velocity_smoother node (<param from="vehicle_param_file"/>)

每个节点在构造函数中:
    VehicleInfoUtils(*this).getVehicleInfo()
        ↓
    从ROS参数服务器读取 wheel_base, wheel_tread, 等参数
        ↓
    创建 VehicleInfo 对象（包含基础参数和派生参数）
        ↓
    存储在节点成员变量 vehicle_info_ 中供后续使用
```

---

## **5. 如何为AGV自定义Vehicle Info**

### **步骤1: 创建自定义车型包**

```bash
cd src/launcher/autoware_launch/vehicle/
cp -r sample_vehicle_launch my_agv_launch
cd my_agv_launch
mv sample_vehicle_description my_agv_description
```

### **步骤2: 修改配置文件**

```bash
# 编辑vehicle info
nano my_agv_description/config/vehicle_info.param.yaml
```

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.15        # AGV轮子半径
    wheel_width: 0.08
    wheel_base: 0.6           # AGV轴距（小型AGV）
    wheel_tread: 0.5
    front_overhang: 0.2
    rear_overhang: 0.2
    left_overhang: 0.05
    right_overhang: 0.05
    vehicle_height: 0.8       # AGV高度
    max_steer_angle: 1.57     # 差速驱动可以设大角度
```

### **步骤3: 启动时指定车型**

```bash
# 方法1: 命令行参数
ros2 launch autoware_launch autoware.launch.xml \
    vehicle_model:=my_agv \
    map_path:=...

# 方法2: 修改launch文件默认值
# 在autoware.launch.xml中:
<arg name="vehicle_model" default="my_agv"/>
```

---

## **6. 调试和验证**

### **检查参数是否正确加载**

```bash
# 查看behavior_path_planner的参数
ros2 param list /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner | grep wheel

# 获取特定参数值
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner wheel_base

# 查看所有vehicle相关参数
ros2 param dump /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner | grep -A 10 "wheel\|vehicle"
```

### **验证参数传递**

```bash
# 在节点启动时添加日志输出
# 或使用计算最小转弯半径的工具
ros2 run autoware_vehicle_info_utils min_turning_radius_calculator.py \
    -y src/launcher/autoware_launch/vehicle/my_agv_launch/my_agv_description/config/vehicle_info.param.yaml
```

---

## **7. 关键要点总结**

| 项目 | 说明 |
|------|------|
| **参数来源** | `{vehicle_model}_description/config/vehicle_info.param.yaml` |
| **传递方式** | Launch文件通过 `<param from="$(var vehicle_param_file)"/>` 加载 |
| **读取机制** | C++代码中通过 `VehicleInfoUtils(*this).getVehicleInfo()` 读取 |
| **参数范围** | 所有planning节点都会加载这些参数（作为ROS参数） |
| **修改方式** | 创建自定义vehicle包或直接修改配置文件 |
| **生效时机** | 节点启动时从参数服务器读取 |

---

**对于AGV应用的建议:**
1. ✅ 创建独立的AGV车型包，避免修改示例配置
2. ✅ 精确测量并配置所有尺寸参数
3. ✅ 差速驱动AGV需要特别注意 `max_steer_angle` 和运动学模型
4. ✅ 使用 `ros2 param` 命令验证参数是否正确加载
5. ✅ 在仿真环境中先测试参数的合理性

需要我详细说明某个特定环节吗？
