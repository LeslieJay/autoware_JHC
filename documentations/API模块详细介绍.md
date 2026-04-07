# AD API 

## Localization

1. /api/localization/initialization_state
话题，发布初始化状态

2. /api/localization/initialize
服务，请求为 PoseWithCovarianceStamped初始化位姿， 响应为 当前状态status

## operation mode

1. /api/operation_mode/state

2. /api/operation_mode/change_to_autonomous

3. /api/operation_mode/enable_autoware_control

### routing

1. /api/routing/state 话题，状态值

2. /api/routing/route 话题，lanelet形式的路径

3. /api/routing/clear_route 服务，无请求值，返回status

4. /api/routing/set_route_points 服务，请求包括终点和途径点，返回status

5. /api/routing/set_route
6. /api/routing/change_route_points
7. /api/routing/change_route

### vehicle status

1. /api/vehicle/kinematics 地理位姿+位姿+速度+加速度

2. /api/vehicle/status 转向灯/方向盘的信息

3. /api/vehicle/metrics 电量或者油量

4. /api/vehicle/dimensions 了解当前车辆与障碍物之间的实际距离，车辆几何尺寸

5. /api/vehicle/specs 车辆规格（最大转向角度）

### planning factors

1. /api/planning/velocity_factors  velocity factors 导致车辆停止或者减速的原因 

| Behavior | Description |
|---|---|
| surrounding-obstacle | There are obstacles immediately around the vehicle. |
| route-obstacle | There are obstacles along the route ahead. |
| intersection | There are obstacles in other lanes in the path. |
| crosswalk | There are obstacles on the crosswalk. |
| rear-check | There are obstacles behind that would be in a human driver's blind spot. |
| user-defined-attention-area | There are obstacles in the predefined attention area. |
| no-stopping-area | There is not enough space beyond the no stopping area. |
| stop-sign | A stop by a stop sign. |
| traffic-signal | A stop by a traffic signal. |
| v2x-gate-area | A stop by a gate area. It has enter and leave as sequences and v2x type as details. |
| merge | A stop before merging lanes. |
| sidewalk | A stop before crossing the sidewalk. |
| lane-change | A lane change. |
| avoidance | A path change to avoid an obstacle in the current lane. |
| emergency-operation | A stop by emergency instruction from the operator. |

2. /api/planning/steering_factors  steering factor 需要使用转向灯的操作 

Behavior 	Description
intersection 	A turning left or right at an intersection.
lane-change 	A lane change.
avoidance 	A path change to avoid an obstacle. It has a sequence of change and return.
start-planner 	T.B.D.
goal-planner 	T.B.D.
emergency-operation 	A path change by emergency instruction from the operator.

3. /api/perception/objects  perception 感知到物体的信息 

objects.id 	unique_identifier_msgs/msg/UUID 	The UUID of each object
objects.existence_probability 	float64 	The probability of the object exits
objects.classification 	autoware_adapi_v1_msgs/msg/ObjectClassification[] 	The type of the object recognized and the confidence level
objects.kinematics 	autoware_adapi_v1_msgs/msg/DynamicObjectKinematics 	Consist of the object pose, twist, acceleration and the predicted_paths
objects.shape 	shape_msgs/msg/SolidPrimitive 	escribe the shape of the object with dimension, and polygon

### Vehicle operation

#### Request to intervene 当 Minimal Risk Maneuver (MRM) 状态不正常的时候，会要求人工接管控制

- **fail sfae** [官方文档](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/)

1. /api/fail_safe/rti_state 话题，rti是否发出请求

2. /api/fail_safe/list_mrm_description 服务，MRM模块的详细信息

3. /api/fail_safe/mrm_state 服务，MRM是否在执行/当前正在执行的行为

4. /api/fail_safe/mrm_request/send  向MRM发送请求和策略
5. /api/fail_safe/mrm_request/list  列出MRM的请求者及策略

#### Request to cooperate 与RTI不同，RTC仅覆盖决策而无需更改操作模式，车辆可以继续自动驾驶

- planning模块给的轨迹是按场景分段规划出的， 执行完scene A，再执行scene B，最后执行scene C。

- **最终执行的轨迹是由 planning 模块和 operator 共同决定的**

`The planning component manages each situation that requires decision as a scene. Each scene has an ID that doesn't change until the scene is completed or canceled. The operator can override the decision of the target scene using this ID.`

1. /api/planning/velocity_factors
2. /api/planning/steering_factors

Name 	Type 	Description
factors.pose 	geometry_msgs/msg/Pose 	The base link pose related to the velocity factor.
factors.distance 	float32 	The distance from the base link to the above pose.
factors.status 	uint16 	The status of the velocity factor.
factors.behavior 	string 	The behavior type of the velocity factor.
factors.sequence 	string 	The sequence type of the velocity factor.
factors.detail 	string 	The additional information of the velocity factor.
factors.cooperation 	*autoware_adapi_v1_msgs/msg/CooperationStatus[<=1]* 	The cooperation status if the module supports.

- autoware_adapi_v1_msgs/msg/CooperationStatus

unique_identifier_msgs/UUID uuid
autoware_adapi_v1_msgs/CooperationDecision autonomous
autoware_adapi_v1_msgs/CooperationDecision cooperator
bool cancellable

3. /api/planning/cooperation/set_commands
4. /api/planning/cooperation/set_policies
5. /api/planning/cooperation/get_policies

### System monitoring

1. /api/system/heartbeat 话题，检查APP和autoware之间的通信是否正常，包括延迟/丢失等

2. Diagnostics 包含autoware每个功能单元的error level

**输出每个功能单元的诊断图，用查找认造成异常的原因**

- /api/system/diagnostics/struct
- /api/system/diagnostics/status
- /api/system/diagnostics/reset

### Manual control

1. remote 模式
/api/manual/remote/control_mode/list
/api/manual/remote/control_mode/select
/api/manual/remote/control_mode/status
...

2. local 模式
/api/manual/local/control_mode/list
/api/manual/local/control_mode/select
/api/manual/local/control_mode/status
...













