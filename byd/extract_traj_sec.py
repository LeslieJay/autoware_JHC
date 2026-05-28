#!/usr/bin/env python3
"""
从 bag 文件中提取指定秒（header.stamp.sec）的轨迹话题数据，保存为 txt 文件。

模式 A（默认）：提取 /planning/scenario_planning/trajectory
  输出: <base>_sec_<sec>_plots/raw/<序号>_<bag_ts_ns>.txt

模式 B（--velocity-smoother）：提取 velocity_smoother 全部 9 个轨迹 debug 话题
  输出: <base>_sec_<sec>_plots/{raw,output,forward_filtered,...}/<序号>_<bag_ts_ns>.txt
        <base>_sec_<sec>_plots/summary.txt

用法:
  python3 src/byd/extract_traj_sec.py <bag_dir> <sec>
  python3 src/byd/extract_traj_sec.py <bag_dir> <sec> --velocity-smoother
  python3 src/byd/extract_traj_sec.py <bag_dir> <sec> --out <output_base>
"""

import argparse
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_planning_msgs.msg import Trajectory

# ── 话题定义 ──────────────────────────────────────────────────────────────────

TOPIC_TRAJ = "/planning/scenario_planning/trajectory"

# (子目录名, 话题名)
VS_TOPICS = [
    ("raw",              "/planning/scenario_planning/velocity_smoother/debug/trajectory_raw"),
    ("ext_vel_limited",  "/planning/scenario_planning/velocity_smoother/debug/trajectory_external_velocity_limited"),
    ("lat_acc",          "/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered"),
    ("steer_rate_limited", "/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited"),
    ("time_resampled",   "/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled"),
    ("forward_filtered", "/planning/scenario_planning/velocity_smoother/debug/forward_filtered_trajectory"),
    ("backward_filtered", "/planning/scenario_planning/velocity_smoother/debug/backward_filtered_trajectory"),
    ("merged_filtered",  "/planning/scenario_planning/velocity_smoother/debug/merged_filtered_trajectory"),
    ("clipped",          "/planning/scenario_planning/velocity_smoother/debug/trajectory_clipped"),
    ("smoothed_pre_overwrite", "/planning/scenario_planning/velocity_smoother/debug/trajectory_smoothed_pre_overwrite"),
    ("output",           "/planning/scenario_planning/velocity_smoother/trajectory"),
]


# ── 共用写文件逻辑 ────────────────────────────────────────────────────────────

def write_txt(path: str, bag_ts_ns: int, header_sec: int, points) -> None:
    n = len(points)
    max_jump = 0.0
    jump_idx = 0
    for i in range(n - 1):
        diff = abs(points[i + 1].longitudinal_velocity_mps
                   - points[i].longitudinal_velocity_mps)
        if diff > max_jump:
            max_jump = diff
            jump_idx = i

    v_from = points[jump_idx].longitudinal_velocity_mps if n > 0 else 0.0
    v_to   = points[jump_idx + 1].longitudinal_velocity_mps if n > 1 else 0.0

    with open(path, "w") as f:
        f.write(f"bag_timestamp_ns: {bag_ts_ns}\n")
        f.write(f"header_sec: {header_sec}\n")
        f.write(f"point_count: {n}\n")
        f.write(f"max_adjacent_jump: {max_jump:.9f}\n")
        f.write(f"jump_index: {jump_idx}\n")
        f.write(f"jump_values: {v_from:.9f} -> {v_to:.9f}\n")
        f.write("\n")
        f.write("idx,x,longitudinal_velocity_mps\n")
        for i, pt in enumerate(points):
            f.write(f"{i},{pt.pose.position.x:.9f},{pt.longitudinal_velocity_mps:.9f}\n")


def _open_reader(bag_dir: str, topics: list[str]) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


# ── 模式 A：单话题 ────────────────────────────────────────────────────────────

def extract_sec(bag_dir: str, target_sec: int, out_base: str) -> None:
    raw_dir = os.path.join(f"{out_base}_sec_{target_sec}_plots", "scenario_planning_trajectory")
    os.makedirs(raw_dir, exist_ok=True)

    reader = _open_reader(bag_dir, [TOPIC_TRAJ])
    seq = 0
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        msg: Trajectory = deserialize_message(data, Trajectory)
        if msg.header.stamp.sec != target_sec:
            continue
        fname = f"{seq:03d}_{ts_ns}.txt"
        write_txt(os.path.join(raw_dir, fname), ts_ns, msg.header.stamp.sec, msg.points)
        print(f"  {fname}  ({len(msg.points)} points)")
        seq += 1

    print(f"\n共写入 {seq} 个文件 → {raw_dir}")


# ── 模式 B：velocity_smoother 全部话题 ───────────────────────────────────────

def extract_velocity_smoother_sec(bag_dir: str, target_sec: int, out_base: str) -> None:
    plots_dir = f"{out_base}_sec_{target_sec}_plots"

    all_topics = [t for _, t in VS_TOPICS]
    reader = _open_reader(bag_dir, all_topics)

    # 每个子目录维护独立序号和消息计数
    counters: dict[str, int] = {subdir: 0 for subdir, _ in VS_TOPICS}
    topic_to_subdir = {t: subdir for subdir, t in VS_TOPICS}
    topic_counts: dict[str, int] = {t: 0 for _, t in VS_TOPICS}

    # 预建目录
    for subdir, _ in VS_TOPICS:
        os.makedirs(os.path.join(plots_dir, subdir), exist_ok=True)

    total = 0
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        msg: Trajectory = deserialize_message(data, Trajectory)
        if msg.header.stamp.sec != target_sec:
            continue

        subdir = topic_to_subdir[topic]
        seq    = counters[subdir]
        fname  = f"{seq:03d}_{ts_ns}.txt"
        fpath  = os.path.join(plots_dir, subdir, fname)
        write_txt(fpath, ts_ns, msg.header.stamp.sec, msg.points)
        print(f"  [{subdir}] {fname}  ({len(msg.points)} points)")
        counters[subdir] += 1
        topic_counts[topic] += 1
        total += 1

    # 写 summary.txt
    summary_path = os.path.join(plots_dir, "summary.txt")
    with open(summary_path, "w") as f:
        f.write(f"bag: {bag_dir}\n")
        f.write(f"target_sec: {target_sec}\n")
        f.write(f"filter: header_sec == {target_sec}\n\n")
        f.write(f"total_plots: {total}\n")
        for subdir, topic in VS_TOPICS:
            f.write(f"[{subdir}]  topic={topic}\n")
            f.write(f"  msg_count={topic_counts[topic]}\n")
            f.write(f"  out_dir={os.path.join(plots_dir, subdir)}\n\n")

    print(f"\n共写入 {total} 个文件 → {plots_dir}/")
    print(f"summary → {summary_path}")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="提取指定秒的轨迹帧，保存为 txt",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("bag_dir", help="bag 目录")
    ap.add_argument("sec", type=int, help="目标 header.stamp.sec（整数秒）")
    ap.add_argument("--out", metavar="BASE", default=None,
                    help="输出基础路径，默认与 bag_dir 同名放在同级 _logs 目录下")
    ap.add_argument("--velocity-smoother", action="store_true",
                    help="提取 velocity_smoother 全部 9 个 debug 轨迹话题")
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag_dir)
    if args.out:
        out_base = os.path.abspath(args.out)
    else:
        parent = os.path.dirname(bag_dir)
        name   = os.path.basename(bag_dir)
        out_base = os.path.join(parent, name + "_logs", name)

    mode = "velocity_smoother" if args.velocity_smoother else "trajectory"
    print(f"bag 目录 : {bag_dir}")
    print(f"目标秒   : {args.sec}")
    print(f"模式     : {mode}")
    print(f"输出根目录: {out_base}_sec_{args.sec}_plots/\n")

    if args.velocity_smoother:
        extract_velocity_smoother_sec(bag_dir, args.sec, out_base)
    else:
        extract_sec(bag_dir, args.sec, out_base)


if __name__ == "__main__":
    main()
