#!/usr/bin/env python3
"""
截取 rosbag2 时间段，并将关键话题导出为 CSV。

步骤：
  1. 按时间窗口裁剪 bag → <output_bag>
  2. 从裁剪后的 bag 中提取话题 → <output_bag>_logs/

支持两种时间指定方式：
  A. 绝对时间（Unix 秒）：  --start 1778118520 --end 1778118530
  B. 相对偏移：             --start-offset 417 --duration 3

提取话题（CSV）：
  trajectory.csv       — /planning/scenario_planning/trajectory
  kinematic_state.csv  — /localization/kinematic_state
  control_cmd.csv      — /control/command/control_cmd

用法示例：
  python3 src/byd/bag_clip_extract.py log/0507/planning_control_bag_02 log/0507/clip_02 \
      --start 1778135090 --end 1778135100

  python3 src/byd/bag_clip_extract.py log/0507/planning_control_bag_02 log/0507/clip_02 \
      --start-offset 417 --duration 10 --topics /control/command/control_cmd /localization/kinematic_state

  # 只裁剪，不提取 CSV
  python3 src/byd/bag_clip_extract.py ... --no-extract
"""

import argparse
import csv
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control


# ─────────────────────────────────────────────────────────────────────────────
# bag 裁剪
# ─────────────────────────────────────────────────────────────────────────────

def get_bag_start_ns(in_dir: str) -> int:
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=in_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    _, _, bag_ts = r.read_next()
    return bag_ts


def clip(in_dir: str, out_dir: str, t_start_ns: int, t_end_ns: int, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=in_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    all_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topics:
        missing = [t for t in topics if t not in all_types]
        if missing:
            print(f"[warn] bag 中不存在，已跳过: {missing}")
        keep = [t for t in topics if t in all_types]
    else:
        keep = list(all_types.keys())

    if not keep:
        print("[error] 无有效 topic")
        sys.exit(1)

    reader.set_filter(rosbag2_py.StorageFilter(topics=keep))

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=out_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for name in keep:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name,
            type=all_types[name],
            serialization_format="cdr",
        ))

    count = 0
    while reader.has_next():
        topic, data, bag_ts = reader.read_next()
        if bag_ts < t_start_ns:
            continue
        if bag_ts > t_end_ns:
            break
        writer.write(topic, data, bag_ts)
        count += 1

    print(f"写入消息数: {count}")
    print(f"输出 bag  : {os.path.abspath(out_dir)}")
    return count


# ─────────────────────────────────────────────────────────────────────────────
# 话题提取为 CSV
# ─────────────────────────────────────────────────────────────────────────────

EXTRACT_TOPICS = [
    "/planning/scenario_planning/trajectory",
    "/localization/kinematic_state",
    "/control/command/control_cmd",
]


def ts_sec(ns: int) -> float:
    return ns * 1e-9


def extract(input_bag: str, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=EXTRACT_TOPICS))

    all_meta = {t.name: t for t in reader.get_all_topics_and_types()}
    missing = [t for t in EXTRACT_TOPICS if t not in all_meta]
    if missing:
        print(f"[警告] 以下话题在 bag 中不存在，将跳过: {missing}")

    f_traj = open(os.path.join(output_dir, "trajectory.csv"),      "w", newline="")
    f_loc  = open(os.path.join(output_dir, "kinematic_state.csv"), "w", newline="")
    f_cmd  = open(os.path.join(output_dir, "control_cmd.csv"),     "w", newline="")

    w_traj = csv.writer(f_traj)
    w_loc  = csv.writer(f_loc)
    w_cmd  = csv.writer(f_cmd)

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

    counts = {t: 0 for t in EXTRACT_TOPICS}

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
    fname_map = {
        "/planning/scenario_planning/trajectory": "trajectory.csv",
        "/localization/kinematic_state":          "kinematic_state.csv",
        "/control/command/control_cmd":           "control_cmd.csv",
    }
    for t, n in counts.items():
        print(f"  {fname_map[t]:30s}  {n} 条消息")


# ─────────────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="截取 rosbag2 时间段并导出关键话题为 CSV",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("input_bag",  help="原始 bag 目录")
    ap.add_argument("output_bag", help="输出 bag 目录（必须不存在或为空）")

    ap.add_argument("--start", type=float, metavar="UNIX_SEC",
                    help="截取起始时间（Unix 秒，支持小数）")
    ap.add_argument("--end",   type=float, metavar="UNIX_SEC",
                    help="截取结束时间（Unix 秒，支持小数）")

    ap.add_argument("--start-offset", type=float, metavar="SEC",
                    help="相对 bag 起始时间的偏移秒数")
    ap.add_argument("--duration", type=float, metavar="SEC",
                    help="截取时长（秒），与 --start-offset 配合使用")

    ap.add_argument("--topics", nargs="+", metavar="TOPIC",
                    help="裁剪时只保留指定话题（默认保留全部；CSV 提取始终使用固定三个话题）")
    ap.add_argument("--no-extract", action="store_true",
                    help="只裁剪 bag，不提取 CSV")

    args = ap.parse_args()

    in_dir  = os.path.abspath(args.input_bag)
    out_dir = os.path.abspath(args.output_bag)

    if os.path.exists(out_dir):
        print(f"[error] 输出目录已存在: {out_dir}")
        print("请指定一个新目录或手动删除后重试")
        sys.exit(1)

    # 解析时间窗口
    if args.start is not None and args.end is not None:
        t_start_ns = int(args.start * 1e9)
        t_end_ns   = int(args.end   * 1e9)
    elif args.start_offset is not None and args.duration is not None:
        bag_start_ns = get_bag_start_ns(in_dir)
        t_start_ns = bag_start_ns + int(args.start_offset * 1e9)
        t_end_ns   = t_start_ns   + int(args.duration     * 1e9)
    else:
        ap.error("请指定 (--start + --end) 或 (--start-offset + --duration)")

    t_start_s = t_start_ns / 1e9
    t_end_s   = t_end_ns   / 1e9

    print(f"输入 bag : {in_dir}")
    print(f"输出 bag : {out_dir}")
    print(f"时间窗口 : [{t_start_s:.3f}, {t_end_s:.3f}]  ({t_end_s - t_start_s:.3f}s)")
    print(f"过滤话题 : {'全部' if not args.topics else args.topics}")
    print()

    # ── 步骤 1：裁剪 ──────────────────────────────────────────────────────────
    print("=== 步骤 1/2：裁剪 bag ===")
    clip(in_dir, out_dir, t_start_ns, t_end_ns, args.topics or [])

    if args.no_extract:
        return

    # ── 步骤 2：提取 CSV ──────────────────────────────────────────────────────
    logs_dir = out_dir + "_logs"
    print(f"\n=== 步骤 2/2：提取 CSV → {logs_dir} ===")
    extract(out_dir, logs_dir)


if __name__ == "__main__":
    main()
