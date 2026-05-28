// Copyright 2024 Tier IV, Inc.
//
// 调参工具：固定输入轨迹，逐组打印不同参数配置的速度/加速度输出，用于寻找最优参数。
//
// 用法1 - 直线合成轨迹：
//   ./build/autoware_velocity_smoother/test_jerk_smoother_tuning
//
// 用法2 - 从 extract_traj_sec.py 生成的 txt 文件读取真实轨迹：
//   ./build/autoware_velocity_smoother/test_jerk_smoother_tuning <path/to/xxx.txt> [v0] [a0]
//   v0/a0 可选，默认取轨迹首点速度和 0.0

#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using autoware::velocity_smoother::JerkFilteredSmoother;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

// ====================================================================
// 从 ROS Trajectory YAML 日志（ros2 topic echo 转存）加载轨迹
// 格式：包含 points: 列表，每个 point 有 pose.position/orientation 和
//       longitudinal_velocity_mps / acceleration_mps2 字段。
// ====================================================================
static TrajectoryPoints load_trajectory_from_ros_log(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    fprintf(stderr, "ERROR: cannot open file: %s\n", path.c_str());
    return {};
  }

  // 辅助：去除首尾空白
  auto trim = [](const std::string & s) -> std::string {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
  };
  // 辅助：解析 "key: value" 中的 value
  auto get_val = [&trim](const std::string & line) -> double {
    size_t pos = line.find(':');
    if (pos == std::string::npos) return 0.0;
    return std::stod(trim(line.substr(pos + 1)));
  };

  TrajectoryPoints pts;
  TrajectoryPoint cur;
  bool in_points = false;  // 是否进入 points: 块
  bool in_point  = false;  // 是否在某个 point 内
  bool in_position    = false;
  bool in_orientation = false;

  std::string line;
  while (std::getline(f, line)) {
    const std::string t = trim(line);
    if (t.empty()) continue;

    // 进入 points: 列表
    if (!in_points) {
      if (t == "points:") in_points = true;
      continue;
    }

    // 新点开始（"- time_from_start:" 或 "- pose:" 等带 '-' 前缀）
    if (t.size() >= 2 && t[0] == '-' && t[1] == ' ') {
      if (in_point) pts.push_back(cur);
      cur = TrajectoryPoint{};
      in_point  = true;
      in_position    = false;
      in_orientation = false;
      // 仍需继续解析当前行其余部分（但本格式该行仅是 "- time_from_start:"）
      continue;
    }

    if (!in_point) continue;

    // 子块标记
    if (t == "position:")    { in_position = true;  in_orientation = false; continue; }
    if (t == "orientation:") { in_orientation = true; in_position = false;  continue; }
    if (t == "pose:")        { in_position = false; in_orientation = false; continue; }

    // 脱离 position/orientation 子块的字段（缩进层级变化不好判断，用关键字匹配）
    if (t.rfind("longitudinal_velocity_mps:", 0) == 0) {
      cur.longitudinal_velocity_mps = static_cast<float>(get_val(t));
      in_position = in_orientation = false;
      continue;
    }
    if (t.rfind("acceleration_mps2:", 0) == 0) {
      cur.acceleration_mps2 = static_cast<float>(get_val(t));
      in_position = in_orientation = false;
      continue;
    }

    // position 字段
    if (in_position) {
      if (t.rfind("x:", 0) == 0) { cur.pose.position.x = get_val(t); continue; }
      if (t.rfind("y:", 0) == 0) { cur.pose.position.y = get_val(t); continue; }
      if (t.rfind("z:", 0) == 0) { cur.pose.position.z = get_val(t); continue; }
    }
    // orientation 字段
    if (in_orientation) {
      if (t.rfind("x:", 0) == 0) { cur.pose.orientation.x = get_val(t); continue; }
      if (t.rfind("y:", 0) == 0) { cur.pose.orientation.y = get_val(t); continue; }
      if (t.rfind("z:", 0) == 0) { cur.pose.orientation.z = get_val(t); continue; }
      if (t.rfind("w:", 0) == 0) { cur.pose.orientation.w = get_val(t); continue; }
    }
  }
  if (in_point) pts.push_back(cur);

  if (pts.empty()) {
    fprintf(stderr, "ERROR: no trajectory points parsed from %s\n", path.c_str());
  }
  return pts;
}

// ====================================================================
// 从 extract_traj_sec.py 输出的 txt 文件加载轨迹（CSV 格式）
// 格式：头部 key:value 行，之后 CSV "idx,x,longitudinal_velocity_mps"
// x 为轨迹点的 x 坐标[m]，y=0，朝向由相邻点差分推算。
// ====================================================================
static TrajectoryPoints load_trajectory_from_file(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    fprintf(stderr, "ERROR: cannot open file: %s\n", path.c_str());
    return {};
  }

  std::string line;
  bool found_header = false;
  while (std::getline(f, line)) {
    if (line.find("idx,x,longitudinal_velocity_mps") != std::string::npos) {
      found_header = true;
      break;
    }
  }
  if (!found_header) {
    fprintf(stderr, "ERROR: CSV header not found in %s\n", path.c_str());
    return {};
  }

  struct Row { double x; double v; };
  std::vector<Row> rows;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::string tok;
    int col = 0;
    Row r{0.0, 0.0};
    while (std::getline(ss, tok, ',')) {
      if (col == 1) r.x = std::stod(tok);
      if (col == 2) r.v = std::stod(tok);
      ++col;
    }
    if (col >= 3) rows.push_back(r);
  }

  if (rows.empty()) {
    fprintf(stderr, "ERROR: no data rows in %s\n", path.c_str());
    return {};
  }

  TrajectoryPoints pts;
  pts.resize(rows.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    pts[i].pose.position.x = rows[i].x;
    pts[i].pose.position.y = 0.0;
    pts[i].pose.position.z = 0.0;
    pts[i].longitudinal_velocity_mps = static_cast<float>(rows[i].v);
    pts[i].acceleration_mps2 = 0.0f;
  }

  // 由相邻差推算 yaw，转为四元数
  auto set_yaw = [&pts](size_t i, double yaw) {
    pts[i].pose.orientation.x = 0.0;
    pts[i].pose.orientation.y = 0.0;
    pts[i].pose.orientation.z = std::sin(yaw / 2.0);
    pts[i].pose.orientation.w = std::cos(yaw / 2.0);
  };
  for (size_t i = 0; i + 1 < rows.size(); ++i) {
    set_yaw(i, std::atan2(0.0, rows[i + 1].x - rows[i].x));
  }
  if (rows.size() >= 2) {
    pts.back().pose.orientation = pts[rows.size() - 2].pose.orientation;
  } else {
    set_yaw(0, 0.0);
  }
  return pts;
}

// ====================================================================
// 合成直线轨迹（默认输入，无文件时使用）
// ====================================================================
static TrajectoryPoints make_straight_trajectory(
  const double cruise_vel, const int n_points, const double ds)
{
  TrajectoryPoints pts;
  pts.reserve(n_points);
  for (int i = 0; i < n_points; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * ds;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.longitudinal_velocity_mps =
      (i == n_points - 1) ? 0.0f : static_cast<float>(cruise_vel);
    p.acceleration_mps2 = 0.0f;
    pts.push_back(p);
  }
  return pts;
}

// ====================================================================
// 计算轨迹累积弧长（用于打印）
// ====================================================================
static std::vector<double> calc_arclength(const TrajectoryPoints & pts)
{
  std::vector<double> s(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    double dx = pts[i].pose.position.x - pts[i - 1].pose.position.x;
    double dy = pts[i].pose.position.y - pts[i - 1].pose.position.y;
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }
  return s;
}

// ====================================================================
// 打印输入 TrajectoryPoints
// ====================================================================
static void print_input(const TrajectoryPoints & input)
{
  const auto s = calc_arclength(input);
  printf("--- Input TrajectoryPoints (%zu pts) ---\n", input.size());
  printf("  %5s  %8s  %8s  %10s  %10s\n",
    "i", "s[m]", "x[m]", "v[m/s]", "a[m/s2]");
  printf("  %5s  %8s  %8s  %10s  %10s\n",
    "-----", "--------", "--------", "----------", "----------");
  for (size_t i = 0; i < input.size(); ++i) {
    printf("  %5zu  %8.3f  %8.3f  %10.4f  %10.4f\n",
      i, s[i],
      input[i].pose.position.x,
      static_cast<double>(input[i].longitudinal_velocity_mps),
      static_cast<double>(input[i].acceleration_mps2));
  }
}

// ====================================================================
// 打印一组优化结果（输出 TrajectoryPoints，与 apply() 返回值完全一致）
// ====================================================================
static void print_result(
  const std::string & config_name, bool ok, const TrajectoryPoints & output)
{
  printf("\n=== Config: %-20s  (apply: %s) ===\n",
    config_name.c_str(), ok ? "OK" : "FAILED");
  if (!ok || output.empty()) return;

  // 弧长从输出自身计算，不依赖输入索引
  const auto s = calc_arclength(output);
  printf("  %5s  %8s  %8s  %10s  %10s\n",
    "i", "s[m]", "x[m]", "v[m/s]", "a[m/s2]");
  printf("  %5s  %8s  %8s  %10s  %10s\n",
    "-----", "--------", "--------", "----------", "----------");
  for (size_t i = 0; i < output.size(); ++i) {
    printf("  %5zu  %8.3f  %8.3f  %10.4f  %10.4f\n",
      i, s[i],
      output[i].pose.position.x,
      static_cast<double>(output[i].longitudinal_velocity_mps),
      static_cast<double>(output[i].acceleration_mps2));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ====================================================================
  // 节点参数（对应 SmootherBase 和 JerkFilteredSmoother 构造函数中的 declare_parameter）
  // 修改此处可调整 base_param_（加速度/jerk 硬约束）
  // ====================================================================
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
    {// --- SmootherBase base params ---
     {"normal.max_acc", 1.0},                        // 最大加速度 [m/s²]
     {"normal.min_acc", -2.5},                        // 最大减速度 [m/s²]
     {"stop_decel", -0.5},                            // 停止点减速度 [m/s²]
     {"normal.max_jerk", 1.0},                        // 最大 jerk [m/s³]
     {"normal.min_jerk", -1.5},                       // 最小 jerk [m/s³]
     {"max_lateral_accel", 2.0},
     {"min_decel_for_lateral_acc_lim_filter", -2.5},
     {"resample_ds", 0.1},
     {"curvature_threshold", 0.02},
     {"max_steering_angle_rate", 40.0},
     {"curvature_calculation_distance", 5.0},
     {"decel_distance_before_curve", 3.5},
     {"decel_distance_after_curve", 0.0},
     {"min_curve_velocity", 1.38},
     {"max_trajectory_length", 200.0},
     {"min_trajectory_length", 30.0},
     {"resample_time", 10.0},
     {"dense_resample_dt", 0.1},
     {"dense_min_interval_distance", 0.1},
     {"sparse_resample_dt", 0.5},
     {"sparse_min_interval_distance", 4.0},
     // --- JerkFilteredSmoother smoother params (构造时读取；setParam 之后可覆盖) ---
     {"jerk_weight", 10.0},
     {"over_v_weight", 100000.0},
     {"over_a_weight", 5000.0},
     {"over_j_weight", 2000.0},
     {"jerk_filter_ds", 0.1},
     {"acc_weight", 0.0}});

  auto node = std::make_shared<rclcpp::Node>("jerk_smoother_tuning", opts);
  auto time_keeper = std::make_shared<autoware_utils::TimeKeeper>();
  auto smoother = std::make_shared<JerkFilteredSmoother>(*node, time_keeper);

  // ====================================================================
  // 构造输入轨迹
  //   无参数：使用内置合成直线轨迹
  //   有参数：从 txt 文件加载，可追加 v0 / a0 覆盖
  // ====================================================================
  TrajectoryPoints input;
  double v0 = 3.0;
  double a0 = 0.0;

  if (argc >= 2 && std::strlen(argv[1]) > 0 && argv[1][0] != '-') {
    // 自动检测格式：扫描文件头几行，有 "points:" 则为 ROS YAML，否则尝试 CSV
    bool is_ros_yaml = false;
    {
      std::ifstream probe(argv[1]);
      std::string l;
      int scan = 0;
      while (scan++ < 20 && std::getline(probe, l)) {
        if (l.find("points:") != std::string::npos) { is_ros_yaml = true; break; }
      }
    }

    if (is_ros_yaml) {
      input = load_trajectory_from_ros_log(argv[1]);
      printf("Input from ROS YAML : %s\n", argv[1]);
    } else {
      input = load_trajectory_from_file(argv[1]);
      printf("Input from CSV      : %s\n", argv[1]);
    }
    if (input.empty()) { rclcpp::shutdown(); return 1; }
    v0 = input.front().longitudinal_velocity_mps;
    a0 = static_cast<double>(input.front().acceleration_mps2);
    if (argc >= 3) v0 = std::stod(argv[2]);
    if (argc >= 4) a0 = std::stod(argv[3]);
  } else {
    input = make_straight_trajectory(v0, 50, 1.0);
    printf("Input           : synthetic straight trajectory\n");
  }
  printf("Points: %zu  v0=%.3f m/s  a0=%.3f m/s²\n\n", input.size(), v0, a0);

  // 打印输入轨迹一次，与 apply() 的入参完全一致
  print_input(input);
  printf("\n");

  // ====================================================================
  // 参数组定义
  // 在此处添加、删除或修改你想对比的参数组合
  //
  // JerkFilteredSmoother::Param 成员：
  //   jerk_weight    : 目标函数中 jerk 平方项的权重（越大轨迹越平滑）
  //   over_v_weight  : 超速惩罚权重（保持超大值）
  //   over_a_weight  : 超加速度惩罚权重
  //   over_j_weight  : 超 jerk 惩罚权重
  //   jerk_filter_ds : 重采样间距（影响滤波分辨率）
  //   acc_weight     : 加速度平方惩罚权重（直接抑制大 |a|，0=禁用）
  // ====================================================================
  struct Config
  {
    std::string name;
    JerkFilteredSmoother::Param param;
    //                         jerk_w   over_v     over_a  over_j  ds    acc_w
  };

  const std::vector<Config> configs = {
    {"default",      {10.0,   100000.0, 5000.0, 2000.0, 0.1, 0.0}},
    {"smooth_jerk",  {100.0,  100000.0, 5000.0, 2000.0, 0.1, 0.0}},   // 更平滑的 jerk
    {"tight_over_a", {10.0,   100000.0, 20000.0, 2000.0, 0.1, 0.0}},  // 更严格的加速度惩罚
    {"acc_penalty",  {10.0,   100000.0, 5000.0, 2000.0, 0.1, 200.0}}, // 直接惩罚大 |a|
    {"combined",     {50.0,   100000.0, 10000.0, 5000.0, 0.1, 100.0}}, // 综合调优
  };

  for (const auto & cfg : configs) {
    smoother->setParam(cfg.param);

    TrajectoryPoints output;
    std::vector<TrajectoryPoints> debug_trajs;
    const bool ok = smoother->apply(v0, a0, input, output, debug_trajs, false);

    print_result(cfg.name, ok, output);
  }

  printf("\n");
  rclcpp::shutdown();
  return 0;
}
