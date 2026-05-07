#!/usr/bin/env python3
"""Extract path-related topics at target_sec and generate txt+png bundles."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.style.use('ggplot')

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG_PATH = '/media/f/nvme_storage/autoware/log/0424/planning_control_localization_06.bag'
TARGET_SEC = 1777013084
OUT_ROOT = Path(
    f'/media/f/nvme_storage/autoware/log/0424/'
    f'planning_control_localization_06.bag_logs/path_sec_{TARGET_SEC}_plots'
)

TOPICS_CONFIG = [
    (
        'root_reference_path',
        '/planning/scenario_planning/lane_driving/behavior_planning/'
        'behavior_path_planner/debug/root_reference_path',
        'visualization_msgs/msg/MarkerArray',
    ),
    (
        'path_with_lane_id',
        '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
        'autoware_internal_planning_msgs/msg/PathWithLaneId',
    ),
    (
        'path',
        '/planning/scenario_planning/lane_driving/behavior_planning/path',
        'autoware_planning_msgs/msg/Path',
    ),
]


def get_header_sec(msg, mtype):
    if 'MarkerArray' in mtype:
        if msg.markers:
            return msg.markers[0].header.stamp.sec
        return None
    return msg.header.stamp.sec


def get_path_points(msg, mtype):
    """Return list of (x, longitudinal_velocity_mps) tuples."""
    result = []
    for pt in msg.points:
        if 'PathWithLaneId' in mtype:
            x = pt.point.pose.position.x
            v = pt.point.longitudinal_velocity_mps
        else:
            x = pt.pose.position.x
            v = pt.longitudinal_velocity_mps
        result.append((x, v))
    return result


def compute_jump(vs):
    max_jump = 0.0
    jump_idx = -1
    jump_vals = (0.0, 0.0)
    for i in range(len(vs) - 1):
        diff = abs(vs[i] - vs[i + 1])
        if diff > max_jump:
            max_jump = diff
            jump_idx = i
            jump_vals = (vs[i], vs[i + 1])
    return max_jump, jump_idx, jump_vals


def save_path_msg(subdir, ts_ns, hs, label, pairs):
    n = len(pairs)
    xs = [p[0] for p in pairs]
    vs = [p[1] for p in pairs]
    max_jump, jump_idx, jump_vals = compute_jump(vs)

    txt_lines = [
        f'bag_timestamp_ns: {ts_ns}',
        f'header_sec: {hs}',
        f'point_count: {n}',
        f'max_adjacent_jump: {max_jump:.9f}',
        f'jump_index: {jump_idx}',
        f'jump_values: {jump_vals[0]:.9f} -> {jump_vals[1]:.9f}',
        '',
        'idx,x,longitudinal_velocity_mps',
    ]
    for i, (x, v) in enumerate(zip(xs, vs)):
        txt_lines.append(f'{i},{x:.9f},{v:.9f}')

    fname = f'{ts_ns}'
    (subdir / f'{fname}.txt').write_text('\n'.join(txt_lines))

    fig, ax = plt.subplots(figsize=(12.8, 7.68), dpi=100)
    ax.plot(range(n), vs, '-o', markersize=3)
    if jump_idx >= 0:
        ax.axvline(x=jump_idx, color='red', linestyle='--', alpha=0.5,
                   label=f'max_jump@{jump_idx}: {max_jump:.3f}')
    ax.set_xlabel('point index')
    ax.set_ylabel('longitudinal_velocity_mps')
    ax.set_title(f'{label} | bag_ts={ts_ns} | pts={n} | max_jump={max_jump:.3f}')
    ax.legend()
    plt.tight_layout()
    fig.savefig(str(subdir / f'{fname}.png'))
    plt.close(fig)


def save_marker_array_msg(subdir, ts_ns, hs, label, msg):
    marker_count = len(msg.markers)
    txt_lines = [
        f'bag_timestamp_ns: {ts_ns}',
        f'header_sec: {hs}',
        f'marker_count: {marker_count}',
        '',
        'marker_id,ns,type,point_count',
    ]
    for m in msg.markers:
        txt_lines.append(f'{m.id},{m.ns},{m.type},{len(m.points)}')

    fname = f'{ts_ns}'
    (subdir / f'{fname}.txt').write_text('\n'.join(txt_lines))

    fig, ax = plt.subplots(figsize=(12.8, 7.68), dpi=100)
    plotted = 0
    for m in msg.markers:
        if m.points:
            xs = [p.x for p in m.points]
            ys = [p.y for p in m.points]
            ax.plot(xs, ys, '-', linewidth=0.8, label=f'id={m.id}')
            plotted += 1
        elif m.pose.position.x != 0 or m.pose.position.y != 0:
            ax.plot(m.pose.position.x, m.pose.position.y, 'o', markersize=4)
            plotted += 1
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{label} | bag_ts={ts_ns} | markers={marker_count}')
    if plotted <= 20:
        ax.legend(fontsize=6)
    plt.tight_layout()
    fig.savefig(str(subdir / f'{fname}.png'))
    plt.close(fig)


def main():
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_opts, conv_opts)

    topic_names = [tc[1] for tc in TOPICS_CONFIG]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topic_names))

    topic_map = {tc[1]: tc for tc in TOPICS_CONFIG}
    msgs_by_label = {tc[0]: [] for tc in TOPICS_CONFIG}

    print(f'Reading bag, filtering for target_sec={TARGET_SEC} ...', flush=True)
    total_read = 0
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        total_read += 1
        if total_read % 500 == 0:
            print(f'  read {total_read} msgs so far...', flush=True)
        try:
            label, _, mtype = topic_map[topic]
            msg_cls = get_message(mtype)
            msg = deserialize_message(data, msg_cls)
            hs = get_header_sec(msg, mtype)
            if hs == TARGET_SEC:
                msgs_by_label[label].append((ts_ns, msg, mtype))
        except Exception as e:
            print(f'  WARN deserialize {topic}: {e}', file=sys.stderr)

    print(f'Finished reading {total_read} msgs total.')

    for label, msgs in msgs_by_label.items():
        print(f'  {label}: {len(msgs)} messages at sec={TARGET_SEC}')

    OUT_ROOT.mkdir(parents=True, exist_ok=True)

    for label, _, _ in TOPICS_CONFIG:
        msgs = msgs_by_label[label]
        subdir = OUT_ROOT / label
        subdir.mkdir(parents=True, exist_ok=True)

        for i, (ts_ns, msg, mtype) in enumerate(msgs):
            hs = get_header_sec(msg, mtype) or TARGET_SEC
            print(f'  [{label}] {i+1}/{len(msgs)} ts={ts_ns}', flush=True)

            if 'MarkerArray' in mtype:
                save_marker_array_msg(subdir, ts_ns, hs, label, msg)
            else:
                pairs = get_path_points(msg, mtype)
                save_path_msg(subdir, ts_ns, hs, label, pairs)

    # Write summary
    summary_lines = [f'target_sec: {TARGET_SEC}']
    total_plots = 0
    for label, _, _ in TOPICS_CONFIG:
        subdir = OUT_ROOT / label
        n = len(list(subdir.glob('*.txt')))
        total_plots += n
        summary_lines.append(f'{label}: {n} messages')
    summary_lines.append(f'total_plots: {total_plots}')
    (OUT_ROOT / 'summary.txt').write_text('\n'.join(summary_lines))
    print('\n--- summary ---')
    print('\n'.join(summary_lines))
    print('Done.')


if __name__ == '__main__':
    main()
