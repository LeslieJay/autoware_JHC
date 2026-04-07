# Autoware 关闭MRM（最小风险机动）的方法

## 概述

MRM（Minimum Risk Maneuver，最小风险机动）是Autoware的安全机制，用于在系统故障时执行安全停止。如果需要关闭MRM，有以下几种方法：

## 方法1：通过配置参数关闭（推荐）

### 1.1 修改 vehicle_cmd_gate 参数文件

编辑配置文件：
- `src/launcher/autoware_launch/autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml`
- 或 `src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/config/vehicle_cmd_gate.param.yaml`

确保以下参数设置为 `false`：

```yaml
/**:
  ros__parameters:
    use_emergency_handling: false  # 设置为false关闭MRM
```

### 1.2 验证参数

配置文件中的 `use_emergency_handling` 参数默认已经是 `false`，所以MRM功能默认是关闭的。

### 1.3 代码修改

**重要**：不要硬编码参数值，应该从配置文件读取。代码已经恢复为正确的参数读取方式：

```cpp
use_emergency_handling_ = declare_parameter<bool>("use_emergency_handling");
```

## 方法2：完全禁用MRM Handler节点（不推荐）

如果希望完全禁用MRM Handler节点本身，可以修改启动文件：

### 2.1 修改 system.launch.xml

编辑文件：
`src/universe/autoware_universe/launch/tier4_system_launch/launch/system.launch.xml`

在MRM Handler部分添加条件判断：

```xml
<!-- MRM Handler -->
<group if="$(var launch_mrm_handler)">
  <include file="$(find-pkg-share autoware_mrm_handler)/launch/mrm_handler.launch.xml">
    <arg name="config_file" value="$(var mrm_handler_param_path)"/>
  </include>
</group>
```

然后在参数定义中添加：
```xml
<arg name="launch_mrm_handler" default="false" description="launch MRM handler"/>
```

### 2.2 传递参数

在启动Autoware时传递参数：
```bash
ros2 launch autoware_launch autoware.launch.xml launch_mrm_handler:=false
```

**注意**：完全禁用MRM Handler可能会导致其他系统组件出现问题，因为某些组件可能依赖MRM Handler的状态。建议使用方法1。

## 方法3：通过MRM Handler参数禁用功能

### 3.1 修改 mrm_handler.param.yaml

编辑文件：
`src/launcher/autoware_launch/autoware_launch/config/system/mrm_handler/mrm_handler.param.yaml`

禁用所有MRM行为：

```yaml
/**:
  ros__parameters:
    use_emergency_holding: false
    use_parking_after_stopped: false
    use_pull_over: false
    use_comfortable_stop: false
```

这样可以禁用所有MRM行为，但节点仍然运行。

## 验证MRM是否已关闭

### 检查日志

启动Autoware后，检查日志中是否有以下信息：

```
[INFO] [vehicle_cmd_gate]: use_emergency_handling_: false
```

### 检查话题

确认以下话题不会发布紧急命令：
- `/system/emergency/control_cmd`
- `/system/emergency/hazard_lights_cmd`
- `/system/emergency/gear_cmd`
- `/system/fail_safe/mrm_state`

### 检查节点

使用以下命令检查MRM Handler节点是否运行：
```bash
ros2 node list | grep mrm_handler
```

如果使用了方法2，该节点应该不存在。

## 总结

**推荐方法**：使用方法1，通过配置参数 `use_emergency_handling: false` 关闭MRM功能。这是最简单且安全的方法。

**重要提示**：
1. 不要硬编码参数值，应该从配置文件读取
2. 关闭MRM会降低系统安全性，请谨慎使用
3. 在生产环境中，建议保持MRM功能开启

## 相关文件

- 配置文件：
  - `src/launcher/autoware_launch/autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml`
  - `src/launcher/autoware_launch/autoware_launch/config/system/mrm_handler/mrm_handler.param.yaml`
  
- 启动文件：
  - `src/universe/autoware_universe/launch/tier4_system_launch/launch/system.launch.xml`
  - `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml`

- 代码文件：
  - `src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/src/vehicle_cmd_gate.cpp`

