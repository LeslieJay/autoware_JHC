#!/usr/bin/env python3
"""
从 bag 文件中提取三个话题，分别保存为 CSV 日志文件。

输出（在 output_dir 下）:
  trajectory.csv       — /planning/scenario_planning/trajectory
                         列: bag_ts, header_stamp, point_idx, x, y, z,
                             longitudinal_velocity_mps, lateral_velocity_mps,
                             acceleration_mps2, heading_rate_rps
  kinematic_state.csv  — /localization/kinematic_state
                         列: bag_ts, header_stamp, x, y, z,
                             qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
  control_cmd.csv      — /control/command/control_cmd
                         列: bag_ts, header_stamp,
                             steering_tire_angle, steering_tire_rotation_rate,
                             velocity, acceleration, jerk

用法:
    python3 src/byd/extract_topics.py <input_bag_dir> [output_dir]

默认 output_dir 为 <input_bag_dir>_logs，与输入 bag 同级目录。
"""

import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control

TOPICS = [
    "/planning/scenario_planning/trajectory",
    "/localization/kinematic_state",
    "/control/command/control_cmd",
]

MSG_TYPES = {
    "/planning/scenario_planning/trajectory": Trajectory,
    "/localization/kinematic_state": Odometry,
    "/control/command/control_cmd": Control,
}


def ts_sec(nanoseconds: int) -> float:
    return nanoseconds * 1e-9


def extract(input_bag: str, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)

    # --- Reader ---
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_opts, conv_opts)
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))

    all_meta = {t.name: t for t in reader.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in all_meta]
    if missing:
        print(f"[警告] 以下话题在 bag 中不存在，将跳过: {missing}")

    # --- 打开输出文件 ---
    traj_path = os.path.join(output_dir, "trajectory.csv")
    loc_path  = os.path.join(output_dir, "kinematic_state.csv")
    cmd_path  = os.path.join(output_dir, "control_cmd.csv")

    f_traj = open(traj_path, "w", newline="")
    f_loc  = open(loc_path,  "w", newline="")
    f_cmd  = open(cmd_path,  "w", newline="")

    w_traj = csv.writer(f_traj)
    w_loc  = csv.writer(f_loc)
    w_cmd  = csv.writer(f_cmd)

    # 写表头
    w_traj.writerow(["bag_ts", "header_stamp", "point_idx",
                     "x", "y", "z",
                     "longitudinal_velocity_mps", "lateral_velocity_mps",
                     "acceleration_mps2", "heading_rate_rps"])
    w_loc.writerow(["bag_ts", "header_stamp",
                    "x", "y", "z",
                    "qx", "qy", "qz", "qw",
                    "vx", "vy", "vz",
                    "wx", "wy", "wz"])
    w_cmd.writerow(["bag_ts", "header_stamp",
                    "steering_tire_angle", "steering_tire_rotation_rate",
                    "velocity", "acceleration", "jerk"])

    counts = {t: 0 for t in TOPICS}

    # --- 逐条读取并反序列化 ---
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        bag_ts = f"{ts_sec(ts_ns):.9f}"

        if topic == "/planning/scenario_planning/trajectory":
            msg: Trajectory = deserialize_message(data, Trajectory)
            header_stamp = f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}"
            for i, pt in enumerate(msg.points):
                w_traj.writerow([
                    bag_ts, header_stamp, i,
                    f"{pt.pose.position.x:.6f}",
                    f"{pt.pose.position.y:.6f}",
                    f"{pt.pose.position.z:.6f}",
                    f"{pt.longitudinal_velocity_mps:.6f}",
                    f"{pt.lateral_velocity_mps:.6f}",
                    f"{pt.acceleration_mps2:.6f}",
                    f"{pt.heading_rate_rps:.6f}",
                ])
            counts[topic] += 1

        elif topic == "/localization/kinematic_state":
            msg: Odometry = deserialize_message(data, Odometry)
            header_stamp = f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}"
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            lv = msg.twist.twist.linear
            av = msg.twist.twist.angular
            w_loc.writerow([
                bag_ts, header_stamp,
                f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                f"{o.x:.6f}", f"{o.y:.6f}", f"{o.z:.6f}", f"{o.w:.6f}",
                f"{lv.x:.6f}", f"{lv.y:.6f}", f"{lv.z:.6f}",
                f"{av.x:.6f}", f"{av.y:.6f}", f"{av.z:.6f}",
            ])
            counts[topic] += 1

        elif topic == "/control/command/control_cmd":
            msg: Control = deserialize_message(data, Control)
            header_stamp = f"{msg.stamp.sec + msg.stamp.nanosec * 1e-9:.9f}"
            w_cmd.writerow([
                bag_ts, header_stamp,
                f"{msg.lateral.steering_tire_angle:.6f}",
                f"{msg.lateral.steering_tire_rotation_rate:.6f}",
                f"{msg.longitudinal.velocity:.6f}",
                f"{msg.longitudinal.acceleration:.6f}",
                f"{msg.longitudinal.jerk:.6f}",
            ])
            counts[topic] += 1

    f_traj.close()
    f_loc.close()
    f_cmd.close()

    print(f"\n输出目录: {output_dir}")
    for topic, n in counts.items():
        fname = {
            "/planning/scenario_planning/trajectory": "trajectory.csv",
            "/localization/kinematic_state": "kinematic_state.csv",
            "/control/command/control_cmd": "control_cmd.csv",
        }[topic]
        print(f"  {fname:30s}  {n} 条消息")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_bag = sys.argv[1].rstrip("/")

    if len(sys.argv) >= 3:
        output_dir = sys.argv[2].rstrip("/")
    else:
        base = os.path.dirname(input_bag)
        name = os.path.basename(input_bag)
        output_dir = os.path.join(base, name + "_logs")

    print(f"输入 bag    : {input_bag}")
    print(f"输出目录    : {output_dir}")
    print(f"提取话题    : {TOPICS}\n")

    extract(input_bag, output_dir)


if __name__ == "__main__":
    main()
