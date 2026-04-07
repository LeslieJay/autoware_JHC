### DDS settings for ROS 2 and Autoware 
1.export ROS_LOCALHOST_ONLY=1 禁止多机通信，只允许本地的节点之间通信
2.Autoware 最推荐的DDS通信方式为   CycloneDDS
3.如果不调整DDS设置，可能导致autoware无法接受到较大的数据（点云）
### system-wide network settings
1. 在启动launch文件之前，加大linux内核的buffer size
sudo sysctl -w net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB
2. IP fragmentation settings
sudo sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB和

### Autoware-Documentation
#### creating sensing models
1. sensor_description:描述 base_link -> sensor_kit_link 以及 sensor_kit_link -> <ALL-SENSORS> 的静态转换关系
<YOUR-VEHICLE-NAME>_sensor_kit_description/
   ├─ config/
   │     ├─ sensor_kit_calibration.yaml
   │     └─ sensors_calibration.yaml
   └─ urdf/
         ├─ sensor_kit.xacro
         └─ sensors.xacro

2. sensor_launch:通过 sensing.launch.xml 来启动各个传感器（lidar、imu、camera等）
<YOUR-VEHICLE-NAME>_sensor_kit_launch/
      ├─ config/
      ├─ data/
      └─ launch/
           ├─ camera.launch.xml
           ├─ gnss.launch.xml
           ├─ imu.launch.xml
           ├─ lidar.launch.xml
           ├─ pointcloud_preprocessor.launch.py
           └─ sensing.launch.xml

#### creating vehicle models
1. 车辆3D模型   mesh/.dae
2. 车辆参数信息  vehicle_info.param.yaml mirror.param.yaml

<YOUR-OWN-AUTOWARE-DIR>/
  └─ src/
       └─ vehicle/
            └─ <YOUR-VEHICLE-NAME>_vehicle_launch/
                 ├─ <YOUR-VEHICLE-NAME>_vehicle_description/
                 └─ <YOUR-VEHICLE-NAME>_vehicle_launch/

#### Calibrating your sensors
1. 使用非线性优化方法来对雷达-IMU进行联合标定 ([LIDAR_IMU_CALIBRATION](https://github.com/autocore-ai/calibration_tools/tree/main/li_calib))

#### Creating vehicle interface
1. autoware -> vehicle
- */control/command/control_cmd*:  获取纵向和横向的控制
- */control/command/actuation_cmd*: 获取刹车/油门的信息
- */control/command/* *: etc.

2. vehicle -> autoware
- */vehicle/status/battery_charge*:  获取纵向和横向的控制
- */vehicle/status/velocity_status*: 获取刹车/油门的信息
- */vehicle/status/* *: etc.

``` cpp
#! QoS策略中队列深度都是1

// from autoware
control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 1, std::bind(&<YOUR-OWN-VEHICLE-INTERFACE>::callback_control_cmd, this, _1));
...
// to autoware
gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
  "/vehicle/status/gear_status", rclcpp::QoS{1});
```
{vehicle_info.param.yaml}:保存了车体模型的相关参数，车体尺寸会对autoware规划模块计算出的路径产生影响

#### Creating Maps
1. pointcloud map
2. vector map
- [bag2lanelet](https://autowarefoundation.github.io/autoware_tools/main/bag2lanelet/)

#### Evaluating the controller performance
1. *control_performance_analysis*：可视化分析车辆控制性能的package

#### Evaluating by a rosbag-based simulator
ros2 launch autoware_launch logging_simulator.launch.xml ... planning:=false control:=false

#### ament_cmake_auto
ament_cmake_auto 是 ament_cmake 的简化版本，适合没有复杂需求的包，而 ament_cmake 提供了更高的灵活性和控制权，适合需要自定义构建过程的包

#### colcon 
1.编译指定包    colcon build --packages-select <package_name1> <package_name2> ...
2.编译指定包及其依赖包    colcon build --packages-up-to <package_name1> <package_name2> ...
3.编译除了指定包外的所有包    colcon build --packages-skip <package_name1>
#### autoware官方论坛
https://github.com/orgs/autowarefoundation/discussions/categories/show-and-tell
实车测试过程中的问题
1. planning模块规划的轨迹中速度为 0
可能是因为某些原因车辆停止，查看话题  /planning/status/stop_reasons 
查看轨迹话题的前一部分的输入  ros2 topic echo /planning/mission_planning/route --field longitudinal_velocity_mps

### 定位速度信息产生的位置
1. Lanelet2地图 (包含speed_limit属性)
   ↓
2. RouteHandler::getCenterLinePath()
   ├─ 读取 lanelet.speed_limit
   └─ 赋值: p.point.longitudinal_velocity_mps = speed_limit
   ↓
3. Behavior Path Planner (路径规划)
   └─ 使用getCenterLinePath()获取带初始速度的路径
   ↓
4. Behavior Velocity Planner (速度修改)
   └─ 根据交通规则、障碍物等修改速度
   ↓
5. Motion Velocity Planner (速度优化)
   └─ 进一步优化速度曲线
   ↓
6. Path Smoother (路径平滑)
   └─ 平滑速度曲线
   ↓
7. Planning Validator (验证发布)
   └─ 最终发布到 /planning/scenario/trajectory
2.控制模块输出的速度值为0,无法切换
### planning模块输出正常，但是无法切换到 auto 模式，导致 control 输出的速度全为 0 
**vehicle_command_gate** 节点：
1. 过滤 trajectory follower传递的控制指令
2. 在多种控制模式之间切换，包括MRM和remot等
3.速度指令无法达到限速值，只能低速行驶
### 影响速度的模块有：
#### 规划部分
1 behaviour path planner: 决定行驶路径形状，几何
output:path_with_lane_id
如果触发避障，生成偏移曲线的话，也会修改速度 // void StaticObstacleAvoidanceModule::insertPrepareVelocity

2 behaviour velocity planner: 根据语义交通规则（人行道/十字路口等），决定插入停止点/减速点
output:path

3 motion velocity planner: 考虑前方障碍物/超出车道线/车辆相撞等情况，修改轨迹上的速度
output:trajectory

4 path smoother： 平滑速度点，保证行驶平稳

#### 控制部分
1 纵向的PID控制器:(v,acc,jerk)
2 轴向的PP控制器:(steering_tire_angle, steering_tire_angle_rate) 
3 同时打开障碍物停止（obstacle stop）和障碍物避让（obstacles avoidance）情况下，停而不绕

### 关键功能模块总结 
#### operation mode
1.查看当前模式和其他模式的可用性
ros2 topic echo /api/operation_mode/state
2.调用服务，切换模式
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
3.查看autoware状态
ros2 topic echo /autoware/state
