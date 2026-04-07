# PID纵向控制器速度和角度计算详解

## 概述

本文档详细解释 `autoware_pid_longitudinal_controller` 包中所有与速度计算和角度计算相关的函数。该控制器使用PID控制算法来实现纵向速度跟踪，并通过坡度补偿来应对坡道行驶。

---

## 一、速度计算相关函数

### 1.1 PID控制器核心函数：`PIDController::calculate()`

**位置：** `src/pid.cpp:34-74`

**函数签名：**
```cpp
double PIDController::calculate(
  const double error,           // 速度误差
  const double dt,              // 时间步长
  const bool enable_integration, // 是否启用积分项
  std::vector<double> & pid_contributions  // PID各项贡献值
)
```

**功能说明：**
这是PID控制器的核心计算函数，根据速度误差计算反馈加速度。

**计算流程：**

1. **比例项（P项）计算：**
   ```cpp
   double ret_p = p.kp * error;  // 比例项 = Kp × 误差
   ret_p = std::min(std::max(ret_p, p.min_ret_p), p.max_ret_p);  // 限制在[min_ret_p, max_ret_p]
   ```

2. **积分项（I项）计算：**
   ```cpp
   if (enable_integration) {
     m_error_integral += error * dt;  // 累加误差积分
     // 限制积分项范围，防止积分饱和
     m_error_integral = std::min(std::max(m_error_integral, p.min_ret_i / p.ki), 
                                 p.max_ret_i / p.ki);
   }
   const double ret_i = p.ki * m_error_integral;  // 积分项 = Ki × 积分误差
   ```

3. **微分项（D项）计算：**
   ```cpp
   if (m_is_first_time) {
     error_differential = 0;  // 第一次调用，微分项为0
   } else {
     error_differential = (error - m_prev_error) / dt;  // 误差变化率
   }
   double ret_d = p.kd * error_differential;  // 微分项 = Kd × 误差变化率
   ret_d = std::min(std::max(ret_d, p.min_ret_d), p.max_ret_d);  // 限制范围
   ```

4. **PID输出计算：**
   ```cpp
   double ret = ret_p + ret_i + ret_d;  // PID输出 = P + I + D
   ret = std::min(std::max(ret, p.min_ret), p.max_ret);  // 限制总输出范围
   ```

**输入参数：**
- `error`: 速度误差 = 目标速度 - 当前速度
- `dt`: 控制周期时间（秒）
- `enable_integration`: 是否启用积分项（低速或车辆卡住时启用）

**输出：**
- 返回PID计算的加速度命令（m/s²）
- `pid_contributions`: [P项贡献, I项贡献, D项贡献]

**关键特性：**
- 每个PID项都有独立的上下限限制，防止某一项过大
- 积分项可以禁用，防止车辆静止时积分累积
- 第一次调用时微分项为0，避免初始跳跃

---

### 1.2 速度反馈控制函数：`applyVelocityFeedback()`

**位置：** `src/pid_longitudinal_controller.cpp:1119-1171`

**函数签名：**
```cpp
double PidLongitudinalController::applyVelocityFeedback(const ControlData & control_data)
```

**功能说明：**
结合前馈（FeedForward）和反馈（FeedBack）控制，计算最终的加速度命令。

**计算流程：**

1. **计算速度误差：**
   ```cpp
   const double vel_sign = (control_data.shift == Shift::Forward) ? 1.0 : -1.0;
   const double current_vel = control_data.current_motion.vel;
   const double target_vel = control_data.interpolated_traj.points.at(target_idx).longitudinal_velocity_mps;
   const double diff_vel = (target_vel - current_vel) * vel_sign;  // 速度误差
   ```

2. **判断是否启用积分：**
   ```cpp
   const bool vehicle_is_moving = std::abs(current_vel) > m_current_vel_threshold_pid_integrate;
   const bool vehicle_is_stuck = !vehicle_is_moving && 
                                  time_under_control > m_time_threshold_before_pid_integrate;
   const bool enable_integration = (vehicle_is_moving || 
                                    (m_enable_integration_at_low_speed && vehicle_is_stuck)) &&
                                   is_under_control;
   ```
   - 车辆移动时：启用积分
   - 车辆静止但卡住时：如果启用 `enable_integration_at_low_speed`，则启用积分

3. **速度误差低通滤波：**
   ```cpp
   const double error_vel_filtered = m_lpf_vel_error->filter(diff_vel);
   ```
   使用低通滤波器平滑速度误差，减少噪声影响

4. **PID反馈计算：**
   ```cpp
   std::vector<double> pid_contributions(3);
   const double pid_acc = m_pid_vel.calculate(error_vel_filtered, control_data.dt, 
                                               enable_integration, pid_contributions);
   ```

5. **前馈加速度计算（带缩放）：**
   ```cpp
   // 前馈缩放：从时间坐标系转换到弧长坐标系
   const double ff_scale = std::clamp(
     std::abs(current_vel) / std::max(std::abs(target_vel), 0.1), 
     ff_scale_min, ff_scale_max);
   const double ff_acc = target_acceleration * ff_scale;
   ```
   **缩放原因：** 前馈加速度应该基于弧长坐标系计算，而不是时间坐标系。如果直接使用轨迹中的加速度，在速度偏差较大时会导致控制不准确。

6. **最终加速度命令：**
   ```cpp
   const double feedback_acc = ff_acc + pid_acc;  // 前馈 + 反馈
   ```

**输入参数：**
- `control_data`: 包含当前速度、目标速度、轨迹信息等

**输出：**
- 返回反馈加速度命令（m/s²）

**关键特性：**
- 前馈项提供快速响应，反馈项消除误差
- 前馈缩放考虑了速度偏差，提高控制精度
- 积分项仅在车辆移动或卡住时启用，防止静止时积分饱和

---

### 1.3 延迟补偿预测函数：`predictedStateAfterDelay()`

**位置：** `src/pid_longitudinal_controller.cpp:1066-1117`

**函数签名：**
```cpp
StateAfterDelay PidLongitudinalController::predictedStateAfterDelay(
  const Motion current_motion,           // 当前运动状态
  const double delay_compensation_time   // 延迟补偿时间
) const
```

**功能说明：**
预测在延迟时间后（通常是执行器延迟，约0.17秒）车辆的速度、加速度和行驶距离。

**计算流程：**

1. **如果没有历史控制命令（首次运行或非自主模式）：**
   ```cpp
   // 计算停车时间
   const double time_to_stop = -current_vel / current_acc;
   const double delay_time_calculation = 
     (time_to_stop > 0.0 && time_to_stop < delay_compensation_time) 
       ? time_to_stop 
       : delay_compensation_time;
   
   // 简单线性预测
   pred_vel = current_vel + current_acc * delay_time_calculation;
   running_distance = std::abs(
     delay_time_calculation * current_vel + 
     0.5 * current_acc * delay_time_calculation * delay_time_calculation
   );
   ```

2. **如果有历史控制命令（正常情况）：**
   ```cpp
   for (std::size_t i = 0; i < m_ctrl_cmd_vec.size(); ++i) {
     if ((clock_->now() - m_ctrl_cmd_vec.at(i).stamp).seconds() < delay_compensation_time) {
       // 计算到下一个加速度命令的时间
       const double time_to_next_acc = ...;
       const double acc = m_ctrl_cmd_vec.at(i).acceleration;
       
       // 更新预测速度
       pred_vel += pred_vel < 0.0 ? (-acc * time_to_next_acc) : (acc * time_to_next_acc);
       
       // 更新行驶距离
       running_distance += std::abs(
         std::abs(pred_vel) * time_to_next_acc + 
         0.5 * acc * time_to_next_acc * time_to_next_acc
       );
       
       // 如果速度符号改变，停止预测
       if (pred_vel / current_vel < 0.0) {
         pred_vel = 0.0;
         break;
       }
     }
   }
   ```

**输入参数：**
- `current_motion`: 当前速度(m/s)和加速度(m/s²)
- `delay_compensation_time`: 延迟补偿时间，默认0.17秒

**输出：**
- `StateAfterDelay` 结构体，包含：
  - `vel`: 预测速度（m/s）
  - `acc`: 预测加速度（m/s²）
  - `running_distance`: 预测行驶距离（m）

**关键特性：**
- 考虑执行器延迟，提高控制精度
- 使用历史控制命令进行更准确的预测
- 处理速度符号变化的情况（前进/后退切换）

---

### 1.4 平滑停止加速度计算：`SmoothStop::calculate()`

**位置：** `src/smooth_stop.cpp:117-168`

**函数签名：**
```cpp
double SmoothStop::calculate(
  const double stop_dist,              // 到停车线的距离
  const double current_vel,            // 当前速度
  const double current_acc,            // 当前加速度
  const std::vector<std::pair<rclcpp::Time, double>> & vel_hist,  // 速度历史
  const double delay_time,             // 延迟时间
  DebugValues & debug_values           // 调试值
)
```

**功能说明：**
在停止状态（STOPPING）下，计算平滑停止的加速度命令。使用多阶段减速策略，确保车辆平稳、准确地停在目标位置。

**计算流程：**

1. **预测停车时间：**
   ```cpp
   const auto time_to_stop = calcTimeToStop(vel_hist);
   ```
   使用速度历史拟合线性函数 `v = at + b`，预测停车时间

2. **判断车辆状态：**
   ```cpp
   const bool is_fast_vel = std::abs(current_vel) > m_params.min_fast_vel;  // 是否高速
   const bool is_running = std::abs(current_vel) > m_params.min_running_vel || 
                           std::abs(current_acc) > m_params.min_running_acc;  // 是否在运行
   ```

3. **多阶段减速策略：**

   **阶段1：超过停车线（紧急情况）**
   ```cpp
   if (stop_dist < m_params.strong_stop_dist) {
     // 大幅超过停车线，使用强减速
     return m_params.strong_stop_acc;  // 例如：-3.4 m/s²
   } else if (stop_dist < m_params.weak_stop_dist) {
     // 略微超过停车线，使用弱减速
     return m_params.weak_stop_acc;    // 例如：-0.8 m/s²
   }
   ```

   **阶段2：车辆运行中**
   ```cpp
   if (is_running) {
     if ((time_to_stop && *time_to_stop > m_params.weak_stop_time + delay_time) || 
         (!time_to_stop && is_fast_vel)) {
       // 预测停车时间较长或高速行驶，使用强减速
       return m_strong_acc;  // 初始计算的强减速加速度
     }
     // 否则使用弱减速，平滑停车
     return m_params.weak_acc;  // 例如：-0.3 m/s²
   }
   ```

   **阶段3：车辆已停止**
   ```cpp
   // 车辆停止后0.5秒内，继续使用弱减速
   if ((now - m_weak_acc_time).seconds() < 0.5) {
     return m_params.weak_acc;
   }
   // 0.5秒后，使用强减速保持停车
   return m_params.strong_stop_acc;
   ```

**输入参数：**
- `stop_dist`: 到停车线的距离（m），负值表示超过停车线
- `current_vel`: 当前速度（m/s）
- `current_acc`: 当前加速度（m/s²）
- `vel_hist`: 速度历史记录，用于预测停车时间
- `delay_time`: 延迟时间（s）

**输出：**
- 返回平滑停止加速度命令（m/s²）

**关键特性：**
- 多阶段减速策略：强减速 → 弱减速 → 强减速保持
- 根据预测停车时间动态调整减速强度
- 考虑延迟时间，提前开始减速

---

### 1.5 强减速初始化：`SmoothStop::init()`

**位置：** `src/smooth_stop.cpp:27-39`

**函数签名：**
```cpp
void SmoothStop::init(const double pred_vel_in_target, const double pred_stop_dist)
```

**功能说明：**
初始化平滑停止，计算强减速阶段的加速度。

**计算流程：**
```cpp
// 如果距离停车线很近
if (pred_stop_dist < epsilon) {
  m_strong_acc = m_params.min_strong_acc;  // 使用最小强减速
  return;
}

// 根据运动学公式计算强减速：v² = 2as
// 解得：a = -v² / (2s)
m_strong_acc = -std::pow(pred_vel_in_target, 2) / (2 * pred_stop_dist);
m_strong_acc = std::max(std::min(m_strong_acc, m_params.max_strong_acc), 
                        m_params.min_strong_acc);
```

**物理原理：**
使用运动学公式 `v² = v₀² + 2as`，假设最终速度为0，得到：
```
a = -v₀² / (2s)
```
其中：
- `v₀`: 预测速度（延迟后）
- `s`: 预测到停车线的距离
- `a`: 需要的减速度

**输入参数：**
- `pred_vel_in_target`: 延迟后的预测速度（m/s）
- `pred_stop_dist`: 延迟后的预测到停车线距离（m）

**输出：**
- 设置 `m_strong_acc` 成员变量

---

## 二、角度计算相关函数

### 2.1 从姿态计算俯仰角：`getPitchByPose()`

**位置：** `src/longitudinal_controller_utils.cpp:79-87`

**函数签名：**
```cpp
double getPitchByPose(const Quaternion & quaternion_msg)
```

**功能说明：**
从车辆姿态四元数中提取俯仰角（pitch angle）。

**计算流程：**
```cpp
double roll, pitch, yaw;
tf2::Quaternion quaternion;
tf2::fromMsg(quaternion_msg, quaternion);  // 将ROS消息转换为tf2四元数
tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);  // 提取欧拉角
return pitch;  // 返回俯仰角
```

**输入参数：**
- `quaternion_msg`: 车辆姿态四元数（geometry_msgs::msg::Quaternion）

**输出：**
- 返回俯仰角（弧度），范围：[-π/2, π/2]
- **注意：** 俯仰角定义为车辆向上倾斜时为负值（与坡度定义相反）

**物理意义：**
- Pitch = 0：车辆水平
- Pitch > 0：车辆前倾（下坡）
- Pitch < 0：车辆后倾（上坡）

---

### 2.2 从轨迹计算俯仰角：`getPitchByTraj()`

**位置：** `src/longitudinal_controller_utils.cpp:89-113`

**函数签名：**
```cpp
double getPitchByTraj(
  const Trajectory & trajectory,  // 参考轨迹
  const size_t start_idx,         // 起始索引（最近点）
  const double wheel_base         // 轴距
)
```

**功能说明：**
根据轨迹上的前后两点计算道路俯仰角（坡度角）。

**计算流程：**

1. **查找合适的计算点：**
   ```cpp
   for (size_t i = start_idx + 1; i < trajectory.points.size(); ++i) {
     const double dist = calc_distance3d(
       trajectory.points.at(start_idx), 
       trajectory.points.at(i)
     );
     if (dist > wheel_base) {
       // 找到距离超过轴距的点，使用这两点计算俯仰角
       return std::make_pair(start_idx, i);
     }
   }
   // 如果轨迹点太密集，使用最后两个点
   return std::make_pair(start_idx, trajectory.points.size() - 1);
   ```

2. **计算俯仰角：**
   ```cpp
   return calc_elevation_angle(
     trajectory.points.at(prev_idx).pose.position,  // 后轮位置（起始点）
     trajectory.points.at(next_idx).pose.position   // 前轮位置（目标点）
   );
   ```

**输入参数：**
- `trajectory`: 参考轨迹
- `start_idx`: 最近轨迹点索引（对应车辆后轮位置）
- `wheel_base`: 车辆轴距（m）

**输出：**
- 返回道路俯仰角（弧度）
- **注意：** 俯仰角定义为上坡为正（与pitch定义相反）

**物理意义：**
- 使用前后轮位置的高度差计算道路坡度
- 更准确地反映实际道路坡度，不受车辆振动影响

---

### 2.3 坡度补偿函数：`applySlopeCompensation()`

**位置：** `src/pid_longitudinal_controller.cpp:1005-1017`

**函数签名：**
```cpp
double PidLongitudinalController::applySlopeCompensation(
  const double input_acc,    // 输入加速度（未补偿）
  const double pitch,        // 俯仰角（弧度）
  const Shift shift          // 档位（前进/后退）
) const
```

**功能说明：**
根据道路坡度调整加速度命令，补偿重力影响。

**计算流程：**
```cpp
if (!m_enable_slope_compensation) {
  return input_acc;  // 如果未启用坡度补偿，直接返回
}

// 限制俯仰角范围
const double pitch_limited = std::clamp(pitch, m_min_pitch_rad, m_max_pitch_rad);

// 根据档位确定符号
double sign = (shift == Shift::Forward) ? 1.0 : 
              (shift == Shift::Reverse ? -1.0 : 0);

// 坡度补偿：重力加速度分量
// 上坡时（pitch > 0）：需要额外加速度克服重力
// 下坡时（pitch < 0）：重力帮助加速，需要减少加速度
double compensated_acc = input_acc + sign * 9.81 * std::sin(pitch_limited);
return compensated_acc;
```

**物理原理：**

1. **重力分解：**
   - 车辆在坡道上受到重力 `mg`，沿坡道方向的分量为 `mg·sin(θ)`
   - 这个分量会影响车辆的加速度

2. **补偿计算：**
   - **上坡（pitch > 0）：** 重力阻碍前进，需要额外加速度
     ```
     compensated_acc = input_acc + g·sin(pitch)
     ```
   - **下坡（pitch < 0）：** 重力帮助加速，需要减少加速度
     ```
     compensated_acc = input_acc - g·sin(|pitch|)
     ```

3. **档位影响：**
   - **前进（Forward）：** sign = 1.0，正常补偿
   - **后退（Reverse）：** sign = -1.0，补偿方向相反

**输入参数：**
- `input_acc`: 未补偿的加速度命令（m/s²）
- `pitch`: 道路俯仰角（弧度）
- `shift`: 车辆档位（Forward/Reverse）

**输出：**
- 返回补偿后的加速度命令（m/s²）

**关键特性：**
- 使用重力加速度 g = 9.81 m/s²
- 考虑前进/后退方向
- 限制俯仰角范围，防止异常值

---

### 2.4 坡度角选择逻辑（在 `getControlData()` 中）

**位置：** `src/pid_longitudinal_controller.cpp:530-572`

**功能说明：**
根据配置的坡度源（slope_source）选择使用哪种俯仰角计算方法。

**坡度源类型：**

1. **RAW_PITCH（原始俯仰角）：**
   ```cpp
   if (m_slope_source == SlopeSource::RAW_PITCH) {
     control_data.slope_angle = m_lpf_pitch->getValue();  // 使用低通滤波后的姿态俯仰角
   }
   ```
   - 来源：车辆姿态（IMU/定位）
   - 优点：实时性好，易于获取
   - 缺点：受车辆振动影响，精度较低

2. **TRAJECTORY_PITCH（轨迹俯仰角）：**
   ```cpp
   else if (m_slope_source == SlopeSource::TRAJECTORY_PITCH) {
     control_data.slope_angle = traj_pitch;  // 使用轨迹计算的俯仰角
   }
   ```
   - 来源：轨迹点高度差
   - 优点：精度高，不受振动影响
   - 缺点：需要高精度地图，不支持自由空间规划

3. **TRAJECTORY_ADAPTIVE（自适应轨迹俯仰角）：**
   ```cpp
   else if (m_slope_source == SlopeSource::TRAJECTORY_ADAPTIVE) {
     const bool is_vel_slow = current_vel < m_adaptive_trajectory_velocity_th;
     // 低速时使用姿态俯仰角，高速时使用轨迹俯仰角
     control_data.slope_angle = is_vel_slow ? m_lpf_pitch->getValue() : traj_pitch;
   }
   ```
   - 根据速度自适应选择
   - 低速：使用姿态俯仰角（实时性好）
   - 高速：使用轨迹俯仰角（精度高）

4. **TRAJECTORY_GOAL_ADAPTIVE（目标自适应轨迹俯仰角）：**
   ```cpp
   else if (m_slope_source == SlopeSource::TRAJECTORY_GOAL_ADAPTIVE) {
     const double goal_dist = calcSignedArcLength(...);
     const bool is_close_to_trajectory_end = goal_dist < m_wheel_base;
     // 接近轨迹终点时使用姿态俯仰角，否则使用轨迹俯仰角
     control_data.slope_angle = is_close_to_trajectory_end 
                                  ? m_lpf_pitch->getValue() 
                                  : traj_pitch;
   }
   ```
   - 根据到轨迹终点的距离自适应选择
   - 接近终点：使用姿态俯仰角
   - 远离终点：使用轨迹俯仰角

**俯仰角平滑处理：**
```cpp
if (m_previous_slope_angle.has_value()) {
  constexpr double gravity_const = 9.8;
  // 限制俯仰角变化率，避免突变
  control_data.slope_angle = std::clamp(
    control_data.slope_angle,
    m_previous_slope_angle.value() + m_min_jerk * control_data.dt / gravity_const,
    m_previous_slope_angle.value() + m_max_jerk * control_data.dt / gravity_const
  );
}
m_previous_slope_angle = control_data.slope_angle;
```

**关键特性：**
- 多种坡度源可选，适应不同场景
- 自适应策略平衡实时性和精度
- 俯仰角变化率限制，避免突变

---

## 三、辅助计算函数

### 3.1 时间步长计算：`getDt()`

**位置：** `src/pid_longitudinal_controller.cpp:949-962`

**功能说明：**
计算当前控制周期的时间步长。

**计算流程：**
```cpp
double dt;
if (!m_prev_control_time) {
  dt = m_longitudinal_ctrl_period;  // 首次调用，使用配置的控制周期
  m_prev_control_time = std::make_shared<rclcpp::Time>(clock_->now());
} else {
  dt = (clock_->now() - *m_prev_control_time).seconds();  // 计算实际时间差
  *m_prev_control_time = clock_->now();
}

// 限制时间步长范围，防止异常值
const double max_dt = m_longitudinal_ctrl_period * 2.0;
const double min_dt = m_longitudinal_ctrl_period * 0.5;
return std::max(std::min(dt, max_dt), min_dt);
```

**关键特性：**
- 使用实际时间差，而非固定周期
- 限制时间步长范围，防止系统异常时产生过大或过小的dt

---

### 3.2 差分限制滤波器：`applyDiffLimitFilter()`

**位置：** `src/longitudinal_controller_utils.cpp:147-163`

**功能说明：**
限制变量的变化率，防止加速度或速度命令突变。

**函数重载1：指定最大最小变化率**
```cpp
double applyDiffLimitFilter(
  const double input_val,    // 输入值
  const double prev_val,     // 前一次值
  const double dt,           // 时间步长
  const double max_val,      // 最大变化率
  const double min_val       // 最小变化率
)
```

**计算流程：**
```cpp
const double diff_raw = (input_val - prev_val) / dt;  // 原始变化率
const double diff = std::clamp(diff_raw, min_val, max_val);  // 限制变化率
const double filtered_val = prev_val + diff * dt;  // 计算滤波后的值
return filtered_val;
```

**函数重载2：对称限制**
```cpp
double applyDiffLimitFilter(
  const double input_val,
  const double prev_val,
  const double dt,
  const double lim_val  // 对称限制值（正负相同）
)
```

**应用场景：**
- 限制加速度命令的变化率（jerk限制）
- 限制速度命令的变化率
- 平滑控制命令，提高乘坐舒适性

---

### 3.3 轨迹点插值：`lerpTrajectoryPoint()`

**位置：** `src/longitudinal_controller_utils.cpp:85-122` (头文件)

**功能说明：**
在轨迹上找到最接近车辆位置的轨迹点，并进行线性插值。

**计算流程：**

1. **找到最近轨迹段：**
   ```cpp
   const size_t seg_idx = findFirstNearestSegmentIndexWithSoftConstraints(
     points, pose, max_dist, max_yaw
   );
   ```

2. **计算插值比例：**
   ```cpp
   const double len_to_interpolated = calcLongitudinalOffsetToSegment(points, seg_idx, pose.position);
   const double len_segment = calcSignedArcLength(points, seg_idx, seg_idx + 1);
   const double interpolate_ratio = std::clamp(len_to_interpolated / len_segment, 0.0, 1.0);
   ```

3. **线性插值所有属性：**
   ```cpp
   interpolated_point.pose.position.x = lerp(points[i].x, points[i+1].x, ratio);
   interpolated_point.pose.position.y = lerp(points[i].y, points[i+1].y, ratio);
   interpolated_point.pose.position.z = lerp(points[i].z, points[i+1].z, ratio);
   interpolated_point.longitudinal_velocity_mps = lerp(points[i].vel, points[i+1].vel, ratio);
   interpolated_point.acceleration_mps2 = lerp(points[i].acc, points[i+1].acc, ratio);
   // ... 其他属性
   ```

**关键特性：**
- 支持位置、速度、加速度等所有轨迹属性的插值
- 使用软约束（max_dist, max_yaw）提高鲁棒性

---

## 四、完整控制流程

### 4.1 主控制循环：`run()`

**位置：** `src/pid_longitudinal_controller.cpp:409-445`

**执行流程：**

1. **获取输入数据：**
   ```cpp
   setTrajectory(input_data.current_trajectory);
   setKinematicState(input_data.current_odometry);
   setCurrentAcceleration(input_data.current_accel);
   setCurrentOperationMode(input_data.current_operation_mode);
   ```

2. **计算控制数据：**
   ```cpp
   const auto control_data = getControlData(current_pose);
   ```
   包括：
   - 当前速度和加速度
   - 目标速度和加速度
   - 俯仰角（坡度）
   - 延迟补偿后的状态
   - 到停车线的距离

3. **更新控制状态：**
   ```cpp
   updateControlState(control_data);
   ```
   根据距离、速度等条件切换状态（DRIVE/STOPPING/STOPPED/EMERGENCY）

4. **计算控制命令：**
   ```cpp
   const Motion ctrl_cmd = calcCtrlCmd(control_data);
   ```
   根据当前状态计算加速度命令

5. **发布控制命令和调试信息：**
   ```cpp
   const auto cmd_msg = createCtrlCmdMsg(ctrl_cmd, control_data.current_motion.vel);
   publishDebugData(ctrl_cmd, control_data);
   ```

---

### 4.2 控制命令计算：`calcCtrlCmd()`

**位置：** `src/pid_longitudinal_controller.cpp:799-898`

**执行流程：**

1. **根据控制状态选择计算方式：**

   **STOPPED状态：**
   ```cpp
   if (m_control_state == ControlState::STOPPED) {
     ctrl_cmd.vel = m_stopped_state_params.vel;  // 0.0
     ctrl_cmd.acc = m_stopped_state_params.acc;  // -3.4 m/s²（保持刹车）
   }
   ```

   **EMERGENCY状态：**
   ```cpp
   else if (m_control_state == ControlState::EMERGENCY) {
     raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);  // 紧急减速
   }
   ```

   **DRIVE状态：**
   ```cpp
   else if (m_control_state == ControlState::DRIVE) {
     raw_ctrl_cmd.vel = target_velocity;
     raw_ctrl_cmd.acc = applyVelocityFeedback(control_data);  // PID反馈控制
     raw_ctrl_cmd = keepBrakeBeforeStop(control_data, raw_ctrl_cmd, target_idx);  // 保持刹车
   }
   ```

   **STOPPING状态：**
   ```cpp
   else if (m_control_state == ControlState::STOPPING) {
     raw_ctrl_cmd.acc = m_smooth_stop.calculate(...);  // 平滑停止
     raw_ctrl_cmd.vel = m_stopped_state_params.vel;    // 0.0
   }
   ```

2. **加速度限制和滤波：**
   ```cpp
   raw_ctrl_cmd.acc = std::clamp(raw_ctrl_cmd.acc, m_min_acc, m_max_acc);  // 限制加速度范围
   raw_ctrl_cmd.acc = applyDiffLimitFilter(..., m_max_jerk, m_min_jerk);   // 限制jerk
   ```

3. **加速度反馈：**
   ```cpp
   const double acc_err = current_acc - raw_ctrl_cmd.acc;
   m_lpf_acc_error->filter(acc_err);
   const double acc_cmd = raw_ctrl_cmd.acc - m_lpf_acc_error->getValue() * m_acc_feedback_gain;
   ```

4. **坡度补偿：**
   ```cpp
   ctrl_cmd.acc = applySlopeCompensation(acc_cmd, control_data.slope_angle, control_data.shift);
   ```

5. **最终差分限制：**
   ```cpp
   ctrl_cmd.acc = applyDiffLimitFilter(ctrl_cmd.acc, m_prev_ctrl_cmd.acc, dt, m_max_acc_cmd_diff);
   ```

---

## 五、关键参数说明

### 5.1 PID参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `kp` | 比例增益 | 1.0 |
| `ki` | 积分增益 | 0.1 |
| `kd` | 微分增益 | 0.0 |
| `max_out` | PID输出最大值 | 1.0 m/s² |
| `min_out` | PID输出最小值 | -1.0 m/s² |
| `max_p_effort` | P项最大值 | 1.0 m/s² |
| `max_i_effort` | I项最大值 | 0.3 m/s² |
| `max_d_effort` | D项最大值 | 0.0 m/s² |

### 5.2 坡度补偿参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `enable_slope_compensation` | 是否启用坡度补偿 | true |
| `slope_source` | 坡度源类型 | "trajectory_goal_adaptive" |
| `lpf_pitch_gain` | 俯仰角低通滤波增益 | 0.95 |
| `max_pitch_rad` | 最大俯仰角 | 0.1 rad |
| `min_pitch_rad` | 最小俯仰角 | -0.1 rad |
| `adaptive_trajectory_velocity_th` | 自适应速度阈值 | 1.0 m/s |

### 5.3 延迟补偿参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `delay_compensation_time` | 延迟补偿时间 | 0.17 s |

### 5.4 平滑停止参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `smooth_stop_max_strong_acc` | 最大强减速 | -0.5 m/s² |
| `smooth_stop_min_strong_acc` | 最小强减速 | -0.8 m/s² |
| `smooth_stop_weak_acc` | 弱减速 | -0.3 m/s² |
| `smooth_stop_weak_stop_acc` | 弱停车减速 | -0.8 m/s² |
| `smooth_stop_strong_stop_acc` | 强停车减速 | -3.4 m/s² |
| `smooth_stop_weak_stop_time` | 弱减速最大时间 | 0.8 s |

---

## 六、总结

### 6.1 速度计算总结

1. **PID反馈控制：** 使用比例、积分、微分三项消除速度误差
2. **前馈控制：** 使用轨迹加速度提供快速响应
3. **延迟补偿：** 预测延迟后的状态，提高控制精度
4. **平滑停止：** 多阶段减速策略，确保平稳停车

### 6.2 角度计算总结

1. **俯仰角来源：** 姿态（实时性好）或轨迹（精度高）
2. **自适应选择：** 根据速度和距离自适应选择俯仰角来源
3. **坡度补偿：** 根据俯仰角补偿重力影响
4. **平滑处理：** 低通滤波和变化率限制，避免突变

### 6.3 关键特性

- **鲁棒性：** 多种保护机制（限制、滤波、状态机）
- **精度：** 延迟补偿、前馈缩放、坡度补偿
- **舒适性：** 平滑停止、jerk限制、积分项控制
- **适应性：** 自适应坡度源、低速积分启用

---

## 七、参考资料

1. **README.md：** 控制器总体设计说明
2. **LongitudinalControllerDiagram.drawio.svg：** 控制框图
3. **LongitudinalControllerStateTransition.drawio.svg：** 状态转换图
4. **BrakeKeeping.drawio.svg：** 刹车保持示意图
5. **slope_definition.drawio.svg：** 坡度定义示意图

---

**文档版本：** 1.0  
**最后更新：** 2025-01-XX  
**作者：** AI Assistant

