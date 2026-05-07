#!/usr/bin/env python3
"""截取 rosbag2 指定时间段，保存为新 bag。

支持两种时间指定方式：
  1. 绝对时间（Unix 秒）：  --start 1778118520 --end 1778118530
  2. 相对偏移（对齐 bag play）：--start-offset 417 --duration 3

用法示例：
  # 绝对时间，截取所有话题
  python3 src/byd/bag_trim.py log/0507/planning_control_bag_02 log/0507/clip_02 --start 1778134923 --end 1778134910

  # 相对偏移 + 时长，只保留指定话题
  python3 src/byd/bag_trim.py log/0507/planning_control_bag_01 log/0507/clip_01 --start-offset 417 --duration 3 --topics /control/command/control_cmd /localization/kinematic_state
"""

import argparse
import os
import sys

import rosbag2_py


def get_bag_start_ns(in_dir: str) -> int:
    """读取 bag 最早一条消息的 bag 接收时间（ns）。"""
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=in_dir, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    _, _, bag_ts = r.read_next()
    return bag_ts


def clip(in_dir: str, out_dir: str, t_start_ns: int, t_end_ns: int, topics: list[str]):
    # ── reader ───────────────────────────────────────────────────────────────
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=in_dir, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    all_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topics:
        missing = [t for t in topics if t not in all_types]
        if missing:
            print(f'[warn] bag 中不存在，已跳过: {missing}')
        keep = [t for t in topics if t in all_types]
    else:
        keep = list(all_types.keys())

    if not keep:
        print('[error] 无有效 topic')
        sys.exit(1)

    reader.set_filter(rosbag2_py.StorageFilter(topics=keep))

    # ── writer ───────────────────────────────────────────────────────────────
    # 注意：不能提前 makedirs，rosbag2_py writer 会自己创建目录，
    # 若目录已存在会报 "can't overwrite existing database" 错误
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=out_dir, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    for name in keep:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name,
            type=all_types[name],
            serialization_format='cdr',
        ))

    # ── 遍历写入 ─────────────────────────────────────────────────────────────
    count = 0
    while reader.has_next():
        topic, data, bag_ts = reader.read_next()
        if bag_ts < t_start_ns:
            continue
        if bag_ts > t_end_ns:
            break   # bag 消息按时间递增，可提前退出
        writer.write(topic, data, bag_ts)
        count += 1

    print(f'写入消息数: {count}')
    print(f'输出 bag  : {os.path.abspath(out_dir)}')


def main():
    ap = argparse.ArgumentParser(
        description='截取 rosbag2 时间段，保存为新 bag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument('input_bag',  help='原始 bag 目录')
    ap.add_argument('output_bag', help='输出 bag 目录（必须不存在或为空）')

    # 时间方式 A：绝对时间
    ap.add_argument('--start', type=float, metavar='UNIX_SEC',
                    help='截取起始时间（Unix 秒，支持小数）')
    ap.add_argument('--end',   type=float, metavar='UNIX_SEC',
                    help='截取结束时间（Unix 秒，支持小数）')

    # 时间方式 B：相对偏移（与 ros2 bag play 对齐）
    ap.add_argument('--start-offset', type=float, metavar='SEC',
                    help='相对 bag 起始时间的偏移秒数（对应 ros2 bag play --start-offset）')
    ap.add_argument('--duration', type=float, metavar='SEC',
                    help='截取时长（秒），与 --start-offset 配合使用')

    ap.add_argument('--topics', nargs='+', metavar='TOPIC',
                    help='只保留指定话题（默认保留全部）')

    args = ap.parse_args()

    in_dir  = os.path.abspath(args.input_bag)
    out_dir = os.path.abspath(args.output_bag)

    # 检查输出目录（不能已存在，writer 不允许覆盖）
    if os.path.exists(out_dir):
        print(f'[error] 输出目录已存在: {out_dir}')
        print('请指定一个新目录或手动删除后重试')
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
        ap.error('请指定 (--start + --end) 或 (--start-offset + --duration)')

    t_start_s = t_start_ns / 1e9
    t_end_s   = t_end_ns   / 1e9

    print(f'输入 bag : {in_dir}')
    print(f'输出 bag : {out_dir}')
    print(f'时间窗口 : [{t_start_s:.3f}, {t_end_s:.3f}]  ({t_end_s - t_start_s:.3f}s)')
    print(f'过滤话题 : {"全部" if not args.topics else args.topics}')
    print()

    clip(in_dir, out_dir, t_start_ns, t_end_ns, args.topics or [])


if __name__ == '__main__':
    main()
