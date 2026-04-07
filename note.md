<!--
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2026-03-24 16:45:57
 * @LastEditors: wei.canming
 * @LastEditTime: 2026-03-25 08:55:46
 * @Description: 
-->
colcon build \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS="-Wno-error -Wno-unused-parameter"


### 清除路径 
service /api/routing/clear_route

## planning
1. 关闭那些节点，用不上
2. 必须使用的节点，调参数，改逻辑

### behavior path planner 
1. side shift（关闭）  用于接受远程（remote control）发送的路径偏移指令
2. avoidance by lane change （必须） 换道避障，实际继承于 normal_lane_change
3. bidirectional_traffic_module （关闭）专门处理双向都可行驶通过的单车道情况，保证车行驶在右侧
4. dynamic_obstacle_avoidance_module (暂时关闭) 避让动态物体，减小复杂度
5. externalRequestLaneChangeRightModuleManager （关闭）接受外部发送的变道指令
6. goal planner （必须） 接近终点时触发，保证终点精确位姿，
7. lane change （必须）正常的换道功能，
8. sampling planner （关闭）
9. start planner （不确定）不开的话，如果从路边起步，好像靠control也可以走到路中间，代测试
10. static obstacle avoidance （必须开）同车道绕障 

### behavior velocity planner
1. boundary_departure_prevention_module (关闭)
2. dynamic_obstacle_stop_module (必须) 规避动态障碍物
3. obstacle_cruise_module (必须) 在障碍物后方巡游
4. obstacle_slow_down (必须) 前方有障碍物时降低速度
5. velocity_limit (必须) 在转弯时降低速度
6. out_of_lane (必须) 在存在跑出道路的情况下，降低速度或者停止
7. road_user_stop(必须) 检测到行人或者车辆时，降低速度
8. run_out (必须) 添加减速点和停止点，以防止与朝自车行驶路径方向运动的目标物体发生碰撞