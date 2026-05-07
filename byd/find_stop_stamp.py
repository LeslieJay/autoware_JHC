#!/usr/bin/env python3
"""
从 bag 文件（或已提取的 CSV 目录）中查找：
  当车辆距轨迹终点 < dist_thresh 米时，/control/command/control_cmd
  第一次出现 longitudinal.velocity == 0 的 header_stamp。

用法:
    # 直接读 bag（会从 bag 重新解析）
    python3 src/byd/find_stop_stamp.py <bag_dir>

    # 读已提取的 CSV 目录（更快）
    python3 src/byd/find_stop_stamp.py --csv <logs_dir>

可选参数:
    --thresh  距离阈值（米），默认 0.5
    --vel     速度阈值（m/s），绝对值小于此值视为 0，默认 0.01
"""

import argparse
import csv
import math
import os
import sys


# ────────────────────────────────────────────────────────────────────────────
# 从 CSV 读取（已由 extract_topics.py 生成）
# ────────────────────────────────────────────────────────────────────────────

def read_csv(path: str):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def analyze_from_csv(logs_dir: str, dist_thresh: float, vel_thresh: float):
    traj_rows = read_csv(os.path.join(logs_dir, "trajectory.csv"))
    loc_rows  = read_csv(os.path.join(logs_dir, "kinematic_state.csv"))
    cmd_rows  = read_csv(os.path.join(logs_dir, "control_cmd.csv"))

    # 每条轨迹消息只取最后一个点作为终点
    # traj_rows 每行是轨迹中的一个 point，同一 bag_ts 属于同一帧
    # 先把轨迹帧整理成 {bag_ts: (last_x, last_y)}
    traj_endpoints: dict[float, tuple[float, float]] = {}
    for row in traj_rows:
        ts = float(row["bag_ts"])
        x  = float(row["x"])
        y  = float(row["y"])
        # 顺序遍历，后写的 point_idx 更大，直接覆盖即可
        traj_endpoints[ts] = (x, y)

    traj_times = sorted(traj_endpoints.keys())

    def latest_endpoint(t: float):
        """返回时刻 t 之前最新的轨迹终点"""
        lo, hi = 0, len(traj_times) - 1
        idx = -1
        while lo <= hi:
            mid = (lo + hi) // 2
            if traj_times[mid] <= t:
                idx = mid
                lo = mid + 1
            else:
                hi = mid - 1
        if idx < 0:
            return None
        return traj_endpoints[traj_times[idx]]

    # 把 control_cmd 转成带时间的列表
    cmd_list = [(float(r["bag_ts"]), float(r["header_stamp"]),
                 float(r["velocity"])) for r in cmd_rows]

    # 找到第一个"已在终点附近"的时刻，然后找速度为 0 的控制指令
    near_goal_ts = None

    for row in loc_rows:
        ts  = float(row["bag_ts"])
        vx  = float(row["x"])
        vy  = float(row["y"])

        ep = latest_endpoint(ts)
        if ep is None:
            continue

        dist = math.hypot(vx - ep[0], vy - ep[1])
        if dist < dist_thresh:
            near_goal_ts = ts
            print(f"[INFO] 首次进入终点附近: bag_ts={ts:.3f}, 距终点={dist:.3f} m")
            break

    if near_goal_ts is None:
        print("[结果] 整个 bag 内车辆从未进入终点附近（阈值 {:.2f} m）".format(dist_thresh))
        return

    # 在 near_goal_ts 之后找第一个 velocity ≈ 0 的控制指令
    for bag_ts, header_stamp, vel in cmd_list:
        if bag_ts < near_goal_ts:
            continue
        if abs(vel) < vel_thresh:
            print(f"[结果] 第一次 velocity≈0 的 header_stamp = {header_stamp:.9f} s")
            print(f"        bag_ts = {bag_ts:.9f} s,  velocity = {vel}")
            return

    print("[结果] 进入终点附近后，控制指令中未找到 velocity≈0 的消息。")


# ────────────────────────────────────────────────────────────────────────────
# 从 bag 文件直接读取
# ────────────────────────────────────────────────────────────────────────────

def analyze_from_bag(bag_dir: str, dist_thresh: float, vel_thresh: float):
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

    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_opts, conv_opts)
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))

    latest_endpoint = None   # (x, y) — 最新轨迹终点
    near_goal       = False

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()

        if topic == "/planning/scenario_planning/trajectory":
            msg: Trajectory = deserialize_message(data, Trajectory)
            if msg.points:
                pt = msg.points[-1]
                latest_endpoint = (pt.pose.position.x, pt.pose.position.y)

        elif topic == "/localization/kinematic_state" and not near_goal:
            if latest_endpoint is None:
                continue
            msg: Odometry = deserialize_message(data, Odometry)
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            dist = math.hypot(px - latest_endpoint[0], py - latest_endpoint[1])
            if dist < dist_thresh:
                near_goal = True
                bag_ts = ts_ns * 1e-9
                print(f"[INFO] 首次进入终点附近: bag_ts={bag_ts:.3f}, 距终点={dist:.3f} m")

        elif topic == "/control/command/control_cmd" and near_goal:
            msg: Control = deserialize_message(data, Control)
            if abs(msg.longitudinal.velocity) < vel_thresh:
                bag_ts      = ts_ns * 1e-9
                header_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
                print(f"[结果] 第一次 velocity≈0 的 header_stamp = {header_stamp:.9f} s")
                print(f"        bag_ts = {bag_ts:.9f} s,  velocity = {msg.longitudinal.velocity}")
                return

    if not near_goal:
        print("[结果] 整个 bag 内车辆从未进入终点附近（阈值 {:.2f} m）".format(dist_thresh))
    else:
        print("[结果] 进入终点附近后，控制指令中未找到 velocity≈0 的消息。")


# ────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", help="bag 目录 或 CSV 日志目录")
    parser.add_argument("--csv",    action="store_true", help="path 为已提取的 CSV 目录")
    parser.add_argument("--thresh", type=float, default=0.5, help="距终点阈值（米），默认 0.5")
    parser.add_argument("--vel",    type=float, default=0.01, help="速度绝对值阈值（m/s），默认 0.01")
    args = parser.parse_args()

    print(f"路径        : {args.path}")
    print(f"距离阈值    : {args.thresh} m")
    print(f"速度阈值    : |v| < {args.vel} m/s\n")

    if args.csv:
        analyze_from_csv(args.path, args.thresh, args.vel)
    else:
        analyze_from_bag(args.path, args.thresh, args.vel)


if __name__ == "__main__":
    main()
