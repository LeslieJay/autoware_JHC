# Pure Pursuit 横向控制器角度计算详解

## 概述

本文档详细解释 `autoware_pure_pursuit` 包中所有与角度计算相关的函数。Pure Pursuit 算法是一种几何路径跟踪算法，通过计算目标点的曲率来确定转向角。

---

## 一、核心角度计算函数

### 1.1 曲率转转向角：`convertCurvatureToSteeringAngle()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:91-94`

**函数签名：**
```cpp
double convertCurvatureToSteeringAngle(double wheel_base, double kappa)
```

**功能说明：**
将曲率（curvature）转换为前轮转向角（steering angle）。这是 Pure Pursuit 算法的核心输出函数。

**计算流程：**
```cpp
return atan(wheel_base * kappa);
```

**物理原理：**

1. **自行车模型：**
   - 假设车辆为自行车模型（bicycle model）
   - 前轮转向角 δ 与转弯半径 R 的关系：
     ```
     tan(δ) = L / R
     ```
     其中：
     - `L`: 轴距（wheel_base）
     - `R`: 转弯半径
     - `δ`: 前轮转向角

2. **曲率与半径的关系：**
   ```
   κ = 1 / R  (曲率 = 1 / 半径)
   ```

3. **最终公式推导：**
   ```
   tan(δ) = L / R = L × κ
   δ = arctan(L × κ)
   ```

**输入参数：**
- `wheel_base`: 车辆轴距（m）
- `kappa`: 曲率（1/m），正值表示左转，负值表示右转

**输出：**
- 返回前轮转向角（弧度），范围：[-π/2, π/2]
- 正值：左转
- 负值：右转

**关键特性：**
- 使用 `atan()` 函数，自动限制输出范围在 [-π/2, π/2]
- 适用于低速场景（忽略轮胎侧滑）
- 基于几何关系，计算简单高效

**使用示例：**
```cpp
// 在 generateCtrlCmdMsg() 中调用
const double tmp_steering = 
  planning_utils::convertCurvatureToSteeringAngle(param_.wheel_base, target_curvature);
cmd.steering_tire_angle = std::clamp(tmp_steering, -max_angle, max_angle);
```

---

### 1.2 计算转弯半径：`calcRadius()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:77-89`

**函数签名：**
```cpp
double calcRadius(
  const geometry_msgs::msg::Point & target,  // 目标点
  const geometry_msgs::msg::Pose & current_pose  // 当前车辆姿态
)
```

**功能说明：**
根据当前车辆位置和目标点，计算通过这两点的圆弧半径。这是 Pure Pursuit 算法的几何基础。

**计算流程：**

1. **坐标变换：**
   ```cpp
   const double denominator = 2 * transformToRelativeCoordinate2D(target, current_pose).y;
   ```
   将目标点转换到车辆相对坐标系，取 y 坐标（横向距离）

2. **计算距离平方：**
   ```cpp
   const double numerator = calcDistSquared2D(target, current_pose.position);
   ```
   计算目标点与车辆位置的欧氏距离平方

3. **计算半径：**
   ```cpp
   if (fabs(denominator) > 0) {
     return numerator / denominator;
   } else {
     return RADIUS_MAX;  // 1e9，表示直线行驶
   }
   ```

**几何原理：**

Pure Pursuit 算法的几何原理基于以下假设：
- 车辆沿圆弧行驶
- 圆弧通过当前车辆位置和目标点
- 车辆当前朝向与圆弧相切

**半径计算公式推导：**

考虑一个圆，其圆心在车辆前方，半径为 R：
- 车辆位置：原点 (0, 0)
- 目标点：在相对坐标系中为 (x, y)
- 圆心：在车辆前方，距离为 R

根据几何关系：
```
R² = x² + (R - y)²
R² = x² + R² - 2Ry + y²
0 = x² - 2Ry + y²
2Ry = x² + y²
R = (x² + y²) / (2y)
```

其中：
- `x² + y²` = `numerator`（距离平方）
- `y` = `denominator / 2`（横向距离）

**输入参数：**
- `target`: 目标点（全局坐标）
- `current_pose`: 当前车辆姿态（包含位置和朝向）

**输出：**
- 返回转弯半径（m）
- 正值：左转
- 负值：右转
- `RADIUS_MAX` (1e9)：直线行驶（y ≈ 0）

**关键特性：**
- 当 `y ≈ 0` 时（目标点在车辆正前方），半径接近无穷大，表示直线行驶
- 使用相对坐标系简化计算
- 处理除零情况，返回最大半径

---

### 1.3 计算曲率：`calcCurvature()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:40-51`

**函数签名：**
```cpp
double calcCurvature(
  const geometry_msgs::msg::Point & target,      // 目标点
  const geometry_msgs::msg::Pose & current_pose // 当前车辆姿态
)
```

**功能说明：**
根据目标点和当前姿态计算曲率。曲率是半径的倒数。

**计算流程：**
```cpp
constexpr double KAPPA_MAX = 1e9;
const double radius = calcRadius(target, current_pose);

if (fabs(radius) > 0) {
  return 1 / radius;  // 曲率 = 1 / 半径
} else {
  return KAPPA_MAX;   // 半径接近0，曲率极大（急转弯）
}
```

**物理意义：**
- **曲率 κ = 1/R**
- 曲率越大，转弯越急
- 曲率为 0：直线行驶
- 曲率为正：左转
- 曲率为负：右转

**输入参数：**
- `target`: 目标点（全局坐标）
- `current_pose`: 当前车辆姿态

**输出：**
- 返回曲率（1/m）
- 范围：[-KAPPA_MAX, KAPPA_MAX]
- 正值：左转
- 负值：右转

**关键特性：**
- 处理半径接近 0 的情况（急转弯）
- 曲率与半径互为倒数关系

---

### 1.4 从轨迹点计算曲率：`calcCurvature()` (重载版本)

**位置：** `src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp:177-218`

**函数签名：**
```cpp
double PurePursuitLateralController::calcCurvature(const size_t closest_idx)
```

**功能说明：**
从轨迹上的三个点（前一点、当前点、后一点）计算当前轨迹的曲率。使用三点法计算曲率，比单点法更准确。

**计算流程：**

1. **确定计算距离：**
   ```cpp
   const size_t idx_dist = static_cast<size_t>(
     std::max(static_cast<int>((param_.curvature_calculation_distance) / param_.resampling_ds), 1));
   ```
   根据配置的曲率计算距离（默认 4.0 m）和重采样间隔（默认 0.1 m）确定索引距离

2. **查找计算点：**
   ```cpp
   if (closest_idx >= idx_dist) {
     prev_idx = closest_idx - idx_dist;  // 前一点
   } else {
     return 0.0;  // 轨迹点不足，返回0曲率
   }
   
   if (trajectory_resampled_->points.size() - 1 >= closest_idx + idx_dist) {
     next_idx = closest_idx + idx_dist;  // 后一点
   } else {
     return 0.0;  // 轨迹点不足，返回0曲率
   }
   ```

3. **三点法计算曲率：**
   ```cpp
   current_curvature = autoware_utils::calc_curvature(
     get_point(trajectory_resampled_->points.at(prev_idx)),   // 前一点
     get_point(trajectory_resampled_->points.at(closest_idx)), // 当前点
     get_point(trajectory_resampled_->points.at(next_idx))    // 后一点
   );
   ```

**三点法曲率计算原理：**

给定三个点 P₁(x₁, y₁), P₂(x₂, y₂), P₃(x₃, y₃)，计算通过这三点的圆的曲率：

1. **计算向量：**
   ```
   v₁ = P₂ - P₁
   v₂ = P₃ - P₂
   ```

2. **计算面积（用于判断方向）：**
   ```
   area = (x₂ - x₁)(y₃ - y₂) - (y₂ - y₁)(x₃ - x₂)
   ```

3. **计算曲率：**
   ```
   κ = 4 × area / (|v₁| × |v₂| × |v₁ + v₂|)
   ```

**输入参数：**
- `closest_idx`: 最近轨迹点的索引

**输出：**
- 返回轨迹曲率（1/m）
- 如果轨迹点不足，返回 0.0

**关键特性：**
- 使用三点法，比单点法更稳定
- 需要足够的轨迹点才能计算
- 默认使用前后 4.0 m 的点进行计算

---

## 二、角度辅助计算函数

### 2.1 归一化欧拉角：`normalizeEulerAngle()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:167-178`

**函数签名：**
```cpp
double normalizeEulerAngle(const double euler)
```

**功能说明：**
将欧拉角归一化到 [-π, π] 范围，用于角度差计算。

**计算流程：**
```cpp
double res = euler;
while (res > M_PI) {
  res -= (2 * M_PI);  // 如果大于π，减去2π
}
while (res < -M_PI) {
  res += 2 * M_PI;    // 如果小于-π，加上2π
}
return res;
```

**输入参数：**
- `euler`: 原始欧拉角（弧度）

**输出：**
- 返回归一化后的角度（弧度），范围：[-π, π]

**应用场景：**
- 计算两个姿态之间的角度差
- 在 `findClosestIdxWithDistAngThr()` 中使用：
  ```cpp
  const double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps);
  if (fabs(yaw_diff) > th_yaw) {
    continue;  // 角度差过大，跳过该点
  }
  ```

**关键特性：**
- 使用循环而非取模，避免浮点数精度问题
- 确保角度差在合理范围内

---

### 2.2 从 Yaw 角创建四元数：`getQuaternionFromYaw()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:223-228`

**函数签名：**
```cpp
geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double _yaw)
```

**功能说明：**
从 yaw 角（航向角）创建四元数，用于表示 2D 平面上的朝向。

**计算流程：**
```cpp
tf2::Quaternion q;
q.setRPY(0, 0, _yaw);  // Roll=0, Pitch=0, Yaw=_yaw
return tf2::toMsg(q);
```

**输入参数：**
- `_yaw`: 航向角（弧度），范围：[-π, π]

**输出：**
- 返回四元数（geometry_msgs::msg::Quaternion）

**应用场景：**
- 在 `calcNextPose()` 中创建旋转：
  ```cpp
  transform.rotation = getQuaternionFromYaw(
    (tan(cmd.steering_tire_angle) * ds) / param_.wheel_base
  );
  ```
- 在 `averageFilterTrajectory()` 中平滑轨迹的朝向

**关键特性：**
- 仅考虑 yaw 角（2D 平面）
- Roll 和 Pitch 设为 0

---

### 2.3 计算下一个姿态：`calcNextPose()`

**位置：** `src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp:143-158`

**函数签名：**
```cpp
TrajectoryPoint PurePursuitLateralController::calcNextPose(
  const double ds,           // 前进距离
  TrajectoryPoint & point,   // 当前轨迹点
  Lateral cmd               // 控制命令（包含转向角）
) const
```

**功能说明：**
根据当前姿态、前进距离和转向角，计算下一个姿态。用于预测轨迹生成。

**计算流程：**

1. **计算旋转角度：**
   ```cpp
   const double yaw_change = (tan(cmd.steering_tire_angle) * ds) / param_.wheel_base;
   ```
   **物理原理：**
   - 根据自行车模型，yaw 角变化率：
     ```
     dψ/dt = (v × tan(δ)) / L
     ```
   - 在距离 ds 上的 yaw 变化：
     ```
     Δψ = (ds × tan(δ)) / L
     ```
     其中：
     - `ds`: 前进距离
     - `δ`: 转向角
     - `L`: 轴距

2. **创建变换：**
   ```cpp
   transform.translation = create_translation(ds, 0.0, 0.0);  // 前进ds距离
   transform.rotation = getQuaternionFromYaw(yaw_change);      // 旋转yaw_change角度
   ```

3. **应用变换：**
   ```cpp
   tf2::Transform tf_pose, tf_offset;
   tf2::fromMsg(point.pose, tf_pose);
   tf2::fromMsg(transform, tf_offset);
   tf2::toMsg(tf_pose * tf_offset, output_p.pose);  // 组合变换
   ```

**输入参数：**
- `ds`: 前进距离（m），默认 0.3 m
- `point`: 当前轨迹点
- `cmd`: 控制命令，包含 `steering_tire_angle`

**输出：**
- 返回下一个轨迹点（TrajectoryPoint）

**应用场景：**
- 在 `generatePredictedTrajectory()` 中迭代预测未来轨迹
- 用于可视化预测路径

**关键特性：**
- 基于自行车模型
- 使用齐次变换矩阵组合平移和旋转

---

## 三、坐标变换函数（涉及角度）

### 3.1 转换到相对坐标系：`transformToRelativeCoordinate2D()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:204-221`

**函数签名：**
```cpp
geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const geometry_msgs::msg::Point & point,      // 全局坐标点
  const geometry_msgs::msg::Pose & origin       // 参考姿态（车辆姿态）
)
```

**功能说明：**
将全局坐标系中的点转换到以车辆为原点的相对坐标系。相对坐标系的 x 轴指向车辆前方，y 轴指向车辆左侧。

**计算流程：**

1. **平移：**
   ```cpp
   trans_p.x = point.x - origin.position.x;
   trans_p.y = point.y - origin.position.y;
   ```

2. **旋转（逆旋转矩阵）：**
   ```cpp
   double yaw = tf2::getYaw(origin.orientation);  // 获取车辆朝向角
   
   // 逆旋转矩阵：[cos(θ)  sin(θ)]  [x]
   //            [-sin(θ) cos(θ)]  [y]
   res.x = cos(yaw) * trans_p.x + sin(yaw) * trans_p.y;      // 前方距离
   res.y = -sin(yaw) * trans_p.x + cos(yaw) * trans_p.y;    // 左侧距离
   ```

**坐标变换原理：**

从全局坐标系 (X, Y) 到相对坐标系 (x, y)：
```
[x]   [cos(θ)  sin(θ)]  [X - X₀]
[y] = [-sin(θ) cos(θ)]  [Y - Y₀]
```

其中：
- `(X₀, Y₀)`: 车辆位置
- `θ`: 车辆朝向角（yaw）

**输入参数：**
- `point`: 全局坐标点
- `origin`: 参考姿态（车辆当前姿态）

**输出：**
- 返回相对坐标点
- `x`: 前方距离（正值在前，负值在后）
- `y`: 左侧距离（正值在左，负值在右）

**应用场景：**
- 在 `calcRadius()` 中计算目标点的相对位置
- 在 `findNextPointIdx()` 中判断点是否在车辆前方
- 在 `lerpNextTarget()` 中计算横向误差

---

### 3.2 转换到绝对坐标系：`transformToAbsoluteCoordinate2D()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:183-199`

**函数签名：**
```cpp
geometry_msgs::msg::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::msg::Point & point,      // 相对坐标点
  const geometry_msgs::msg::Pose & origin      // 参考姿态（车辆姿态）
)
```

**功能说明：**
将相对坐标系中的点转换到全局坐标系。这是 `transformToRelativeCoordinate2D()` 的逆变换。

**计算流程：**

1. **旋转：**
   ```cpp
   double yaw = tf2::getYaw(origin.orientation);
   rot_p.x = cos(yaw) * point.x + (-sin(yaw)) * point.y;
   rot_p.y = sin(yaw) * point.x + cos(yaw) * point.y;
   ```

2. **平移：**
   ```cpp
   res.x = rot_p.x + origin.position.x;
   res.y = rot_p.y + origin.position.y;
   ```

**坐标变换原理：**

从相对坐标系 (x, y) 到全局坐标系 (X, Y)：
```
[X]   [cos(θ) -sin(θ)]  [x]   [X₀]
[Y] = [sin(θ)  cos(θ)]  [y] + [Y₀]
```

**输入参数：**
- `point`: 相对坐标点
- `origin`: 参考姿态（车辆当前姿态）

**输出：**
- 返回全局坐标点

**应用场景：**
- 在可视化中绘制轨迹圆
- 将预测轨迹从相对坐标转换回全局坐标

---

## 四、角度相关的辅助函数

### 4.1 计算横向误差：`calcLateralError2D()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:66-75`

**函数签名：**
```cpp
double calcLateralError2D(
  const geometry_msgs::msg::Point & line_s,  // 线段起点
  const geometry_msgs::msg::Point & line_e,  // 线段终点
  const geometry_msgs::msg::Point & point    // 目标点
)
```

**功能说明：**
计算点到线段的垂直距离（横向误差）。使用向量叉积计算。

**计算流程：**

1. **构建向量：**
   ```cpp
   tf2::Vector3 a_vec((line_e.x - line_s.x), (line_e.y - line_s.y), 0.0);  // 线段向量
   tf2::Vector3 b_vec((point.x - line_s.x), (point.y - line_s.y), 0.0);    // 起点到目标点向量
   ```

2. **计算叉积：**
   ```cpp
   double lat_err = (a_vec.length() > 0) 
     ? a_vec.cross(b_vec).z() / a_vec.length() 
     : 0.0;
   ```

**数学原理：**

向量叉积的几何意义：
```
a_vec × b_vec = |a_vec| × |b_vec| × sin(θ)
```

其中 `θ` 是两向量夹角。

横向误差（点到直线的垂直距离）：
```
lateral_error = |b_vec| × sin(θ) = (a_vec × b_vec) / |a_vec|
```

在 2D 平面中，叉积的 z 分量：
```
(a_vec × b_vec).z = a_x × b_y - a_y × b_x
```

**输入参数：**
- `line_s`: 线段起点
- `line_e`: 线段终点
- `point`: 目标点

**输出：**
- 返回横向误差（m）
- 正值：点在线段左侧
- 负值：点在线段右侧
- 0：点在直线上

**应用场景：**
- 在 `lerpNextTarget()` 中计算车辆到轨迹段的横向误差
- 用于判断车辆偏离轨迹的程度

---

### 4.2 查找最近点（带角度阈值）：`findClosestIdxWithDistAngThr()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:109-136`

**函数签名：**
```cpp
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(
  const std::vector<geometry_msgs::msg::Pose> & poses,  // 轨迹点列表
  const geometry_msgs::msg::Pose & current_pose,        // 当前车辆姿态
  double th_dist,                                       // 距离阈值
  double th_yaw                                         // 角度阈值
)
```

**功能说明：**
在轨迹中查找最接近车辆的点，同时考虑距离和角度约束。

**计算流程：**

1. **遍历所有轨迹点：**
   ```cpp
   for (size_t i = 0; i < poses.size(); ++i) {
     // 计算距离
     const double ds = calcDistSquared2D(poses.at(i).position, current_pose.position);
     if (ds > th_dist * th_dist) {
       continue;  // 距离过远，跳过
     }
   ```

2. **计算角度差：**
   ```cpp
     const double yaw_pose = tf2::getYaw(current_pose.orientation);  // 车辆朝向
     const double yaw_ps = tf2::getYaw(poses.at(i).orientation);     // 轨迹点朝向
     const double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps); // 角度差
     if (fabs(yaw_diff) > th_yaw) {
       continue;  // 角度差过大，跳过
     }
   ```

3. **选择最近点：**
   ```cpp
     if (ds < dist_squared_min) {
       dist_squared_min = ds;
       idx_min = i;
     }
   }
   ```

**输入参数：**
- `poses`: 轨迹点列表
- `current_pose`: 当前车辆姿态
- `th_dist`: 距离阈值（m），默认 3.0 m
- `th_yaw`: 角度阈值（弧度），默认 π/2

**输出：**
- 返回 `std::pair<bool, int32_t>`
  - `first`: 是否找到有效点
  - `second`: 最近点的索引（如果找到）

**应用场景：**
- 在 `PurePursuit::run()` 中查找最近轨迹点
- 在 `calcTargetCurvature()` 中查找最近点

**关键特性：**
- 同时考虑距离和角度约束，提高鲁棒性
- 使用角度阈值避免选择方向相反的轨迹点

---

### 4.3 获取车道方向：`getLaneDirection()`

**位置：** `src/autoware_pure_pursuit_core/planning_utils.cpp:138-165`

**函数签名：**
```cpp
int8_t getLaneDirection(const std::vector<geometry_msgs::msg::Pose> & poses, double th_dist)
```

**功能说明：**
判断轨迹的方向（前进或后退）。通过分析轨迹点的相对位置确定。

**计算流程：**

1. **查找距离足够大的相邻点：**
   ```cpp
   for (uint32_t i = 0; i < poses.size(); i++) {
     // 获取相邻点
     if (i == poses.size() - 1) {
       prev = poses.at(i - 1);
       next = poses.at(i);
     } else {
       prev = poses.at(i);
       next = poses.at(i + 1);
     }
     
     // 检查距离
     if (calcDistSquared2D(prev.position, next.position) > th_dist * th_dist) {
       // 找到距离足够大的点对
   ```

2. **判断方向：**
   ```cpp
       const auto rel_p = transformToRelativeCoordinate2D(next.position, prev);
       return (rel_p.x > 0.0) ? 0 : 1;  // 0=前进, 1=后退
     }
   }
   ```

**输入参数：**
- `poses`: 轨迹点列表
- `th_dist`: 距离阈值（m），默认 0.5 m

**输出：**
- 返回方向标识
  - `0`: 前进方向
  - `1`: 后退方向
  - `2`: 错误（轨迹点不足或异常）

**应用场景：**
- 在 `findNextPointIdx()` 中判断轨迹方向
- 确保查找的点在车辆前方（前进）或后方（后退）

---

## 五、Pure Pursuit 核心算法流程

### 5.1 Pure Pursuit 主函数：`PurePursuit::run()`

**位置：** `src/autoware_pure_pursuit_core/autoware_pure_pursuit.cpp:52-96`

**功能说明：**
Pure Pursuit 算法的核心执行函数，计算目标曲率。

**执行流程：**

1. **查找最近轨迹点：**
   ```cpp
   auto closest_pair = findClosestIdxWithDistAngThr(
     *curr_wps_ptr_, *curr_pose_ptr_, closest_thr_dist_, closest_thr_ang_);
   ```

2. **查找前瞻点索引：**
   ```cpp
   int32_t next_wp_idx = findNextPointIdx(closest_pair.second);
   ```
   查找距离车辆超过前瞻距离的第一个轨迹点

3. **线性插值目标点：**
   ```cpp
   if (next_wp_idx == 0) {
     next_tgt_pos = curr_wps_ptr_->at(next_wp_idx).position;
   } else {
     next_tgt_pos = lerpNextTarget(next_wp_idx).second;  // 插值计算
   }
   ```

4. **计算曲率：**
   ```cpp
   double kappa = calcCurvature(next_tgt_pos, *curr_pose_ptr_);
   ```

**输出：**
- 返回 `std::pair<bool, double>`
  - `first`: 是否成功计算
  - `second`: 目标曲率（如果成功）

---

### 5.2 线性插值目标点：`lerpNextTarget()`

**位置：** `src/autoware_pure_pursuit_core/autoware_pure_pursuit.cpp:99-149`

**函数签名：**
```cpp
std::pair<bool, geometry_msgs::msg::Point> PurePursuit::lerpNextTarget(int32_t next_wp_idx)
```

**功能说明：**
在前瞻距离为半径的圆与轨迹段的交点中，找到车辆前方的目标点。

**计算流程：**

1. **计算横向误差：**
   ```cpp
   const double lateral_error = calcLateralError2D(
     vec_start, vec_end, curr_pose.position);
   ```
   计算车辆到轨迹段的垂直距离

2. **检查有效性：**
   ```cpp
   if (fabs(lateral_error) > lookahead_distance_) {
     return false;  // 横向误差过大，无法找到交点
   }
   ```

3. **计算垂足：**
   ```cpp
   // 计算轨迹段方向的单位向量
   Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
   uva2d.normalize();
   
   // 旋转90度得到垂直方向
   Eigen::Rotation2Dd rot = (lateral_error > 0) 
     ? Eigen::Rotation2Dd(-M_PI / 2.0) 
     : Eigen::Rotation2Dd(M_PI / 2.0);
   Eigen::Vector2d uva2d_rot = rot * uva2d;
   
   // 计算垂足位置
   h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
   h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
   ```

4. **计算交点：**
   ```cpp
   if (fabs(fabs(lateral_error) - lookahead_distance_) < ERROR2) {
     // 只有一个交点（相切）
     return h;
   } else {
     // 有两个交点，选择前方的交点
     const double s = sqrt(pow(lookahead_distance_, 2) - pow(lateral_error, 2));
     res.x = h.x + s * uva2d.x();  // 沿轨迹方向前进
     res.y = h.y + s * uva2d.y();
     return res;
   }
   ```

**几何原理：**

考虑以车辆为圆心、前瞻距离为半径的圆与轨迹段的交点：

1. **情况1：相切（一个交点）**
   - 横向误差 = 前瞻距离
   - 目标点 = 垂足

2. **情况2：相交（两个交点）**
   - 使用勾股定理计算沿轨迹方向的距离：
     ```
     s = √(ld² - lateral_error²)
     ```
   - 选择前方的交点（沿轨迹方向）

**输入参数：**
- `next_wp_idx`: 前瞻点索引

**输出：**
- 返回 `std::pair<bool, Point>`
  - `first`: 是否成功计算
  - `second`: 目标点位置（如果成功）

---

### 5.3 查找前瞻点索引：`findNextPointIdx()`

**位置：** `src/autoware_pure_pursuit_core/autoware_pure_pursuit.cpp:151-198`

**函数签名：**
```cpp
int32_t PurePursuit::findNextPointIdx(int32_t search_start_idx)
```

**功能说明：**
在轨迹中查找距离车辆超过前瞻距离的第一个点。

**计算流程：**

1. **判断轨迹方向：**
   ```cpp
   const auto gld = getLaneDirection(*curr_wps_ptr_, 0.05);
   ```

2. **根据方向过滤点：**
   ```cpp
   if (gld == 0) {  // 前进
     auto ret = transformToRelativeCoordinate2D(
       curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
     if (ret.x < 0) {
       continue;  // 点在车辆后方，跳过
     }
   } else if (gld == 1) {  // 后退
     auto ret = transformToRelativeCoordinate2D(...);
     if (ret.x > 0) {
       continue;  // 点在车辆前方，跳过
     }
   }
   ```

3. **检查距离：**
   ```cpp
   const double ds = calcDistSquared2D(curr_motion_point, curr_pose_point);
   if (ds > std::pow(lookahead_distance_, 2)) {
     return i;  // 找到超过前瞻距离的点
   }
   ```

**输入参数：**
- `search_start_idx`: 搜索起始索引（通常是最近点索引）

**输出：**
- 返回前瞻点索引
- `-1`: 未找到（轨迹点不足）

---

## 六、前瞻距离计算

### 6.1 计算前瞻距离：`calcLookaheadDistance()`

**位置：** `src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp:105-141`

**函数签名：**
```cpp
double PurePursuitLateralController::calcLookaheadDistance(
  const double lateral_error,    // 横向误差
  const double curvature,       // 当前曲率
  const double velocity,        // 当前速度
  const double min_ld,         // 最小前瞻距离
  const bool is_control_cmd    // 是否为控制命令计算
)
```

**功能说明：**
根据速度、曲率和横向误差动态计算前瞻距离。前瞻距离越大，车辆跟踪越平滑但响应越慢。

**计算流程：**

1. **速度项：**
   ```cpp
   const double vel_ld = abs(param_.ld_velocity_ratio * velocity);
   ```
   - 速度越快，前瞻距离越大
   - 默认比例：2.4

2. **曲率项：**
   ```cpp
   const double curvature_ld = -abs(param_.ld_curvature_ratio * curvature);
   ```
   - 曲率越大（转弯越急），前瞻距离越小（负值）
   - 默认比例：120.0
   - **注意：** 这里是负值，用于减小总前瞻距离

3. **横向误差项：**
   ```cpp
   double lateral_error_ld = 0.0;
   if (abs(lateral_error) >= param_.long_ld_lateral_error_threshold) {
     lateral_error_ld = abs(param_.ld_lateral_error_ratio * lateral_error);
   }
   ```
   - 仅当横向误差超过阈值（默认 0.5 m）时启用
   - 横向误差越大，前瞻距离越大
   - 默认比例：3.6
   - **目的：** 防止车辆以高航向误差进入道路

4. **总前瞻距离：**
   ```cpp
   const double total_ld = std::clamp(
     vel_ld + curvature_ld + lateral_error_ld, 
     min_ld, 
     param_.max_lookahead_distance
   );
   ```

**公式：**
```
LD = clamp(
  ld_velocity_ratio × |velocity| - 
  ld_curvature_ratio × |curvature| + 
  (if |lateral_error| > threshold: ld_lateral_error_ratio × |lateral_error| else 0),
  min_lookahead_distance,
  max_lookahead_distance
)
```

**输入参数：**
- `lateral_error`: 横向误差（m）
- `curvature`: 当前曲率（1/m）
- `velocity`: 当前速度（m/s）
- `min_ld`: 最小前瞻距离（m），前进默认 4.35 m，后退默认 7.0 m
- `is_control_cmd`: 是否为控制命令计算（影响调试输出）

**输出：**
- 返回前瞻距离（m），范围：[min_ld, max_lookahead_distance]

**关键特性：**
- 自适应调整，平衡跟踪精度和稳定性
- 高速时增大前瞻距离，提高稳定性
- 急转弯时减小前瞻距离，提高响应性
- 大横向误差时增大前瞻距离，避免高航向误差

---

## 七、完整控制流程

### 7.1 主控制循环：`run()`

**位置：** `src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp:343-369`

**执行流程：**

1. **接收输入数据：**
   ```cpp
   current_pose_ = input_data.current_odometry.pose.pose;
   trajectory_ = input_data.current_trajectory;
   current_odometry_ = input_data.current_odometry;
   ```

2. **重采样轨迹：**
   ```cpp
   setResampledTrajectory();  // 等间隔重采样
   ```

3. **可选：轨迹平滑：**
   ```cpp
   if (param_.enable_path_smoothing) {
     averageFilterTrajectory(*trajectory_resampled_);  // 移动平均滤波
   }
   ```

4. **计算控制命令：**
   ```cpp
   const auto cmd_msg = generateOutputControlCmd();
   ```

5. **检查转向收敛：**
   ```cpp
   output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg);
   ```

6. **生成预测轨迹：**
   ```cpp
   const auto predicted_trajectory = generatePredictedTrajectory();
   ```

---

### 7.2 计算目标曲率：`calcTargetCurvature()`

**位置：** `src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp:421-490`

**执行流程：**

1. **查找最近轨迹点：**
   ```cpp
   const auto closest_idx_result = 
     findNearestIndex(output_tp_array_, pose, 3.0, M_PI_4);
   ```

2. **计算横向误差：**
   ```cpp
   const double lateral_error = 
     calcLateralOffset(trajectory_resampled_->points, pose.position);
   ```

3. **计算当前曲率：**
   ```cpp
   const double current_curvature = calcCurvature(*closest_idx_result);
   ```

4. **计算前瞻距离：**
   ```cpp
   double lookahead_distance = calcLookaheadDistance(
     lateral_error, current_curvature, velocity, min_ld, is_control_output);
   ```

5. **运行 Pure Pursuit：**
   ```cpp
   pure_pursuit_->setCurrentPose(pose);
   pure_pursuit_->setWaypoints(extractPoses(*trajectory_resampled_));
   pure_pursuit_->setLookaheadDistance(lookahead_distance);
   const auto pure_pursuit_result = pure_pursuit_->run();
   ```

6. **返回结果：**
   ```cpp
   PpOutput output{};
   output.curvature = kappa;  // 目标曲率
   output.velocity = target_vel;
   ```

---

### 7.3 生成控制命令：`generateCtrlCmdMsg()`

**位置：** `src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp:399-410`

**执行流程：**

1. **曲率转转向角：**
   ```cpp
   const double tmp_steering = 
     convertCurvatureToSteeringAngle(param_.wheel_base, target_curvature);
   ```

2. **限制转向角范围：**
   ```cpp
   cmd.steering_tire_angle = std::clamp(
     tmp_steering, 
     -param_.max_steering_angle, 
     param_.max_steering_angle
   );
   ```

**完整公式链：**
```
目标点 → 半径 → 曲率 → 转向角
target → R → κ = 1/R → δ = arctan(L × κ)
```

---

## 八、关键参数说明

### 8.1 前瞻距离参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `ld_velocity_ratio` | 速度前瞻距离比例 | 2.4 |
| `ld_curvature_ratio` | 曲率前瞻距离比例 | 120.0 |
| `ld_lateral_error_ratio` | 横向误差前瞻距离比例 | 3.6 |
| `min_lookahead_distance` | 最小前瞻距离（前进） | 4.35 m |
| `max_lookahead_distance` | 最大前瞻距离 | 15.0 m |
| `reverse_min_lookahead_distance` | 最小前瞻距离（后退） | 7.0 m |
| `long_ld_lateral_error_threshold` | 横向误差阈值 | 0.5 m |

### 8.2 曲率计算参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `curvature_calculation_distance` | 曲率计算距离 | 4.0 m |
| `resampling_ds` | 轨迹重采样间隔 | 0.1 m |

### 8.3 车辆参数

| 参数 | 说明 | 来源 |
|------|------|------|
| `wheel_base` | 轴距 | 车辆信息 |
| `max_steering_angle` | 最大转向角 | 车辆信息 |

### 8.4 其他参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `converged_steer_rad` | 转向收敛阈值 | 0.1 rad |
| `prediction_ds` | 预测步长 | 0.3 m |
| `prediction_distance_length` | 预测总距离 | 21.0 m |
| `enable_path_smoothing` | 是否启用路径平滑 | false |
| `path_filter_moving_ave_num` | 移动平均窗口大小 | 25 |

---

## 九、角度计算流程图

### 9.1 完整角度计算流程

```
输入：当前姿态、轨迹、速度
  ↓
查找最近轨迹点
  ↓
计算横向误差
  ↓
计算当前曲率（三点法）
  ↓
计算前瞻距离（速度+曲率+横向误差）
  ↓
查找前瞻点（距离 > 前瞻距离）
  ↓
线性插值目标点（圆与轨迹段交点）
  ↓
计算目标曲率（半径法）
  ↓
曲率转转向角：δ = arctan(L × κ)
  ↓
限制转向角范围
  ↓
输出：转向角命令
```

### 9.2 曲率计算两种方法对比

**方法1：半径法（单点）**
- 位置：`calcCurvature(target, current_pose)`
- 输入：目标点 + 当前姿态
- 原理：计算通过当前点和目标点的圆弧半径
- 优点：实时性好，适合动态目标点
- 缺点：对目标点位置敏感

**方法2：三点法（轨迹）**
- 位置：`calcCurvature(closest_idx)`
- 输入：轨迹上的三个点
- 原理：通过三点拟合圆弧
- 优点：更稳定，适合轨迹分析
- 缺点：需要足够的轨迹点

---

## 十、数学公式总结

### 10.1 核心公式

1. **曲率与半径：**
   ```
   κ = 1 / R
   ```

2. **转向角与曲率：**
   ```
   δ = arctan(L × κ)
   ```
   其中：
   - `L`: 轴距
   - `κ`: 曲率
   - `δ`: 转向角

3. **半径计算：**
   ```
   R = (x² + y²) / (2y)
   ```
   其中：
   - `(x, y)`: 目标点在车辆相对坐标系中的位置
   - `y`: 横向距离

4. **前瞻距离：**
   ```
   LD = clamp(
     α × |v| - β × |κ| + γ × |e|,
     LD_min,
     LD_max
   )
   ```
   其中：
   - `v`: 速度
   - `κ`: 曲率
   - `e`: 横向误差
   - `α, β, γ`: 比例系数

5. **Yaw 角变化：**
   ```
   Δψ = (ds × tan(δ)) / L
   ```
   其中：
   - `ds`: 前进距离
   - `δ`: 转向角
   - `L`: 轴距

---

## 十一、关键特性总结

### 11.1 角度计算特点

1. **几何方法：** 基于几何关系，计算简单高效
2. **自适应前瞻：** 根据速度、曲率、误差动态调整
3. **鲁棒性：** 使用距离和角度双重约束查找最近点
4. **平滑性：** 支持轨迹平滑和预测轨迹生成

### 11.2 适用场景

- **优点：**
  - 计算简单，实时性好
  - 参数少，易于调试
  - 适合低速场景

- **限制：**
  - 基于自行车模型，忽略轮胎侧滑
  - 高速时可能不够精确
  - 对轨迹质量要求较高

---

## 十二、参考资料

1. **README.md：** Pure Pursuit 控制器总体说明
2. **Pure Pursuit 算法原理：** 几何路径跟踪算法
3. **自行车模型：** 车辆运动学模型
4. https://blog.51cto.com/u_16954161/12209946
---

**文档版本：** 1.0  
**最后更新：** 2025-01-XX  
**作者：** AI Assistant

