#!/usr/bin/env python3
"""
从 ROS2 bag 文件中提取 /control/command/control_cmd 话题，
保存为 CSV 文件。

输出格式（与 control_cmd.csv 一致）:
  header_stamp,velocity

用法:
    python3 src/byd/extract_control_cmd.py <input_bag_dir> [output_csv]

    <input_bag_dir>  bag 目录路径（含 metadata.yaml）
    [output_csv]     可选，输出 CSV 路径
                     默认为 <input_bag_dir>_logs/control_cmd.csv

示例:
    python3 extract_control_cmd.py planning_control_localization_06.bag
    python3 extract_control_cmd.py planning_control_localization_06.bag /tmp/cmd.csv
"""

import os
import sys
import csv

import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_control_msgs.msg import Control

TOPIC = "/control/command/control_cmd"


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_opts, conv_opts)
    return reader


def extract(input_bag: str, output_csv: str) -> None:
    reader = open_reader(input_bag)

    available = {t.name for t in reader.get_all_topics_and_types()}
    if TOPIC not in available:
        print(f"[错误] 话题 {TOPIC!r} 在 bag 中不存在")
        print(f"  可用话题: {sorted(available)}")
        sys.exit(1)

    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    out_dir = os.path.dirname(output_csv)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    count = 0
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["header_stamp", "velocity"])

        while reader.has_next():
            _topic, data, _ts_ns = reader.read_next()
            msg: Control = deserialize_message(data, Control)
            header_stamp = f"{msg.stamp.sec + msg.stamp.nanosec * 1e-9:.9f}"
            velocity = f"{msg.longitudinal.velocity:.6f}"
            writer.writerow([header_stamp, velocity])
            count += 1

    print(f"提取完成: {count} 条消息 -> {output_csv}")


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_bag = sys.argv[1].rstrip("/")

    if len(sys.argv) >= 3:
        output_csv = sys.argv[2]
    else:
        base = os.path.dirname(os.path.abspath(input_bag))
        name = os.path.basename(input_bag)
        output_dir = os.path.join(base, name + "_logs")
        output_csv = os.path.join(output_dir, "control_cmd.csv")

    print(f"输入 bag : {input_bag}")
    print(f"输出文件 : {output_csv}")
    print(f"话题     : {TOPIC}")

    extract(input_bag, output_csv)


if __name__ == "__main__":
    main()
