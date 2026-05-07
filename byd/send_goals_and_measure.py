#!/usr/bin/env python3
"""
逐个发送 goal，每次按 Enter 发送下一个 goal；
车辆到达后再按 Enter 采集 kinematic_state，
结果追加写入 measured_points.md。
"""

import os
import subprocess
import json
import math
import re
import sys
from datetime import datetime

# 在脚本运行的当前工作目录下保存结果文件
OUTPUT_FILE = os.path.join(os.getcwd(), "measured_points.md")

GOALS = [
    {
        "id": 1,
        "x": -166.75697572081393, "y": -84.12780794007236, "z": 0.5922776638379166,
        "qx": 0.0009473683847744576, "qy": 0.00033535554962371337,
        "qz": -0.9788338857946326,  "qw": 0.20465388843112373,
    },
    {
        "id": 2,
        "x": -219.460342, "y": -124.656952, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": -0.553195, "qw": 0.833052,
    },
    {
        "id": 3,
        "x": -205.707260, "y": -105.723206, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.197890, "qw": 0.980224,
    },
    {
        "id": 4,
        "x": -132.189285, "y": -68.292519, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.172715, "qw": 0.984972,
    },
    {
        "id": 5,
        "x": -24.877525, "y": -13.787126, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.197889, "qw": 0.980224,
    },
    {
        "id": 6,
        "x": -24.199051, "y": -11.531457, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": -0.976130, "qw": 0.217186,
    },
]


def send_goal(g):
    cmd = (
        f"ros2 topic pub --once /planning/mission_planning/goal "
        f"geometry_msgs/msg/PoseStamped "
        f'"{{header: {{frame_id: map}}, pose: {{position: '
        f'{{x: {g["x"]}, y: {g["y"]}, z: {g["z"]}}}, '
        f'orientation: {{x: {g["qx"]}, y: {g["qy"]}, z: {g["qz"]}, w: {g["qw"]}}}}}}}"'
    )
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
    if result.returncode != 0:
        print(f"[警告] goal 发送可能失败:\n{result.stderr.strip()}")
    else:
        print("[✓] goal 已发送")


def get_kinematic_state():
    """读取一帧 /localization/kinematic_state，返回解析后的字典"""
    result = subprocess.run(
        ["ros2", "topic", "echo", "--once", "--spin-time", "5",
         "/localization/kinematic_state"],
        capture_output=True, text=True, timeout=10
    )
    raw = result.stdout

    def extract(pattern, text, cast=float):
        m = re.search(pattern, text)
        return cast(m.group(1)) if m else None

    # 提取 stamp
    sec  = extract(r"stamp:\s*\n\s*sec:\s*(\d+)", raw, int)
    nsec = extract(r"nanosec:\s*(\d+)", raw, int)

    # 提取 position（取 pose.pose.position 块内首次出现的 x/y/z）
    pos_block = re.search(
        r"pose:\s*\n\s*pose:\s*\n\s*position:\s*\n(.*?)\n\s*orientation:",
        raw, re.DOTALL
    )
    px = py = pz = None
    if pos_block:
        blk = pos_block.group(1)
        px = extract(r"x:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)
        py = extract(r"y:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)
        pz = extract(r"z:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)

    # 提取 orientation（取首次出现的四元数）
    ori_block = re.search(
        r"orientation:\s*\n(.*?)(?:\n\s*covariance:|\Z)",
        raw, re.DOTALL
    )
    ox = oy = oz = ow = None
    if ori_block:
        blk = ori_block.group(1)
        ox = extract(r"x:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)
        oy = extract(r"y:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)
        oz = extract(r"z:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)
        ow = extract(r"w:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)", blk)

    return {"sec": sec, "nsec": nsec,
            "px": px, "py": py, "pz": pz,
            "ox": ox, "oy": oy, "oz": oz, "ow": ow,
            "raw": raw}


def yaw_from_quat(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def compute_error(g, meas):
    dx = meas["px"] - g["x"]
    dy = meas["py"] - g["y"]
    dist = math.sqrt(dx**2 + dy**2)
    yaw_goal = yaw_from_quat(g["qz"], g["qw"])
    yaw_meas = yaw_from_quat(meas["oz"], meas["ow"])
    dyaw_deg = math.degrees(angle_diff(yaw_meas, yaw_goal))
    s_long =  dx * math.cos(yaw_goal) + dy * math.sin(yaw_goal)
    s_lat  = -dx * math.sin(yaw_goal) + dy * math.cos(yaw_goal)
    return dx, dy, dist, dyaw_deg, s_long, s_lat


def append_result(g, meas, dx, dy, dist, dyaw_deg, s_long, s_lat):
    ts = f"{meas['sec']}.{meas['nsec']:09d}" if meas["sec"] else "unknown"
    with open(OUTPUT_FILE, "a") as f:
        f.write(f"\n## Goal {g['id']}\n\n")
        f.write(f"**采集时间戳**: `{ts}`  \n")
        f.write(f"**记录时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("### Goal 目标位姿\n")
        f.write(f"- position: x={g['x']:.6f}, y={g['y']:.6f}, z={g['z']:.6f}\n")
        f.write(f"- orientation: qx={g['qx']:.6f}, qy={g['qy']:.6f}, qz={g['qz']:.6f}, qw={g['qw']:.6f}\n\n")
        f.write("### 实测位姿 (kinematic_state)\n")
        f.write(f"- position: x={meas['px']:.6f}, y={meas['py']:.6f}, z={meas['pz']:.6f}\n")
        f.write(f"- orientation: qx={meas['ox']:.6f}, qy={meas['oy']:.6f}, qz={meas['oz']:.6f}, qw={meas['ow']:.6f}\n\n")
        f.write("### 误差\n")
        f.write(f"| 指标 | 数值 |\n|------|------|\n")
        f.write(f"| Δx | {dx:+.4f} m |\n")
        f.write(f"| Δy | {dy:+.4f} m |\n")
        f.write(f"| 2D 距离误差 | {dist:.4f} m |\n")
        f.write(f"| 纵向误差 s_long | {s_long:+.4f} m |\n")
        f.write(f"| 横向误差 s_lat | {s_lat:+.4f} m |\n")
        f.write(f"| 偏航角误差 | {dyaw_deg:+.3f}° |\n")


def init_output_file():
    with open(OUTPUT_FILE, "w") as f:
        f.write(f"# Goal 到达精度测量\n\n")
        f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"总计 {len(GOALS)} 个 goal\n")


def main():
    print("=" * 55)
    print("  Goal 精度测量脚本")
    print(f"  结果保存至: {OUTPUT_FILE}")
    print("=" * 55)

    init_output_file()

    for i, g in enumerate(GOALS):
        print(f"\n{'─'*50}")
        print(f"[{i+1}/{len(GOALS)}] Goal {g['id']}: "
              f"x={g['x']:.3f}, y={g['y']:.3f}")
        input("  >>> 按 Enter 发送 goal ...")

        send_goal(g)

        input("  >>> 车辆到达后，按 Enter 采集当前位姿 ...")

        print("  正在读取 /localization/kinematic_state ...")
        try:
            meas = get_kinematic_state()
        except subprocess.TimeoutExpired:
            print("[错误] 读取 kinematic_state 超时，跳过本 goal")
            continue

        if meas["px"] is None:
            print("[错误] 解析 kinematic_state 失败，原始输出:")
            print(meas["raw"][:500])
            continue

        dx, dy, dist, dyaw_deg, s_long, s_lat = compute_error(g, meas)

        print(f"  实测位置: x={meas['px']:.4f}, y={meas['py']:.4f}")
        print(f"  误差: Δx={dx:+.4f}m  Δy={dy:+.4f}m  dist={dist:.4f}m  Δyaw={dyaw_deg:+.3f}°")
        print(f"  纵向={s_long:+.4f}m  横向={s_lat:+.4f}m")

        append_result(g, meas, dx, dy, dist, dyaw_deg, s_long, s_lat)
        print(f"  [✓] 已追加写入 {OUTPUT_FILE}")

    print(f"\n{'='*50}")
    print(f"全部完成！结果已保存至 {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
