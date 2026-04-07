<!--
 * @Author: leslie leslie@byd.com
 * @Date: 2026-03-03 15:28:38
 * @LastEditors: leslie leslie@byd.com
 * @LastEditTime: 2026-03-28 17:10:17
 * @FilePath: /autoware/src/universe/autoware_universe/control/autoware_pid_longitudinal_controller/note.md
 * @Description: Do not edit
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
-->
### 调试信息话题 /control/trajectory_follower/longitudinal/diagnostic

stamp:
  sec: 1774688645
  nanosec: 545873807
layout:
  dim: []
  data_offset: 0
data:
- 0.029984045773744583
- 3.617964744567871
- 3.614506721496582
- 0.7056638598442078
- 3.5446832180023193
- 0.7378581166267395
- 0.0
- -0.06304756551980972
- -0.06308101862668991
- -3.6123595237731934
- -3.6142759323120117
- -0.07328152656555176
- -0.006190768908709288
- 0.0
- 0.7050396203994751
- 0.7050396203994751
- 0.7050396203994751
- 0.08695264160633087
- 0.08695264160633087
- -0.006190768908709288
- 0.0048913960345089436
- 0.0
- 0.0
- 0.0
- 3.684462785720825
- 0.7534335851669312
- -0.06304756551980972
- -3.6123595237731934
- 20.37175941467285
- 1.000956654548645
- 0.7063390016555786
- 0.04839398339390755
- 0.0011268336093053222
- 0.7050396203994751
- -0.0632372722029686
- -3.6232285499572754
- 0.0
---


DT = 0,
CURRENT_VEL = 1,
TARGET_VEL = 2,
TARGET_ACC = 3,
NEAREST_VEL = 4,
NEAREST_ACC = 5,
SHIFT = 6,
PITCH_USING_RAD = 7,
PITCH_RAW_RAD = 8,
PITCH_USING_DEG = 9,
PITCH_RAW_DEG = 10,
ERROR_VEL = 11,
ERROR_VEL_FILTERED = 12,
CONTROL_STATE = 13,
ACC_CMD_PID_APPLIED = 14,
ACC_CMD_ACC_LIMITED = 15,
ACC_CMD_JERK_LIMITED = 16,
ACC_CMD_SLOPE_APPLIED = 17,
ACC_CMD_PUBLISHED = 18,
ACC_CMD_FB_P_CONTRIBUTION = 19,
ACC_CMD_FB_I_CONTRIBUTION = 20,
ACC_CMD_FB_D_CONTRIBUTION = 21,
FLAG_STOPPING = 22,
FLAG_EMERGENCY_STOP = 23,
PREDICTED_VEL = 24,
CALCULATED_ACC = 25,
PITCH_RAW_TRAJ_RAD = 26,
PITCH_RAW_TRAJ_DEG = 27,
STOP_DIST = 28,
FF_SCALE = 29,
ACC_CMD_FF = 30,
ERROR_ACC = 31,
ERROR_ACC_FILTERED = 32,
ACC_CMD_ACC_FB_APPLIED = 33,
PITCH_LPF_RAD = 34,
PITCH_LPF_DEG = 35,
SMOOTH_STOP_MODE = 36,
SIZE  // this is the number of enum elements

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
10. 当前为STOPPED，判断departure_condition_from_stopped，如果当前状态为静止且方向盘角度未收敛，则 current_keep_stopped_condition 为true，判断是否需要保持stopped