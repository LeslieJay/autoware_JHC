### updateControlState() 函数详细解释

- 根据当前车速和停止距离，判断当前状态
1. 从stopping状态进入driving，距离stop点的长度大于drive_state_stop_dist+drive_state_offset_stop_dist(安全裕度)
2. 从stopped状态进入driving，距离stop点的长度大于drive_state_stop_dist，设置两种不同的起步状态是为了防止在stop点附近抖动，一会儿刹车一会儿加速
3. 根据车速判断是否running/stopped，根据stop_dist判断是否在is_stopping
4. 进入emergency_condition：1 车辆超过了stop点，2 车辆速度小于epl，3 m_enable_overshoot_emergency
5. 判断当前是否处于 AUTONOMOUS 模式
6. 如果当前状态不是stopped，则m_prev_keep_stopped_condition设置为空指针
7. 进入四种状态(DRIVE, STOPPING, STOPPED, EMERGENCY)的判断逻辑，是否需要转换状态
8. 当前为DRIVE，判断emergency_condition和!is_under_control，如果进入stopping_condition，平滑减速，如果是停止状态且距离stop点小于drive_state_stop_dist，则进入stopped状态，否则直接return
9. 当前为STOPPING，判断emergency_condition，如果当前是stopped_condition，返回STOPPED状态，如果当前departure_condition_from_stopping，开始起步，进入DRIVE
10. 当前为STOPPED，判断departure_condition_from_stopped，