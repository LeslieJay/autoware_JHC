# 测试记录
测试过程中出现的问题及解决方法

### 速度在正常和全0之间反复跳变

- 根本原因：replan_vel_deviation 参数过小触发正反馈振荡

1.第一次规划，轨迹起点速度engage_vel=0.8
2.第二次规划，此时desired_velocity~=0.98，但此时的vehicle_speed=0.0，两者之间的差值大于replan_vel_deviation，触发LARGE_DEVIATION_REPLAN，初始速度被强制用当前车速（0）重新规划，输出全0轨迹
3.上一帧轨迹全0，desired_vel=0，重新进入ENGAGING，轨迹又被engage_velocity拉起，ego点速度又大于0.8