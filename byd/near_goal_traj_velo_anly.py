#!/usr/bin/env python3
"""Near-goal velocity/trajectory analysis.
Output format matches log/0424/planning_control_localization_05.bag_logs/
Usage: python3 near_goal_traj_velo_anly.py <BAG_DIR> <LOG_FILE> [OUT_DIR]
"""
import argparse
import csv
import math
import os
import re
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

EPS = 1e-3  # zero-velocity threshold (m/s)

# ── helpers ──────────────────────────────────────────────────────────────────

def make_reader(bag_dir, topics):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def type_map(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def deser(data, type_str):
    return rclpy.serialization.deserialize_message(data, get_message(type_str))


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# ── Step 1: log parsing ───────────────────────────────────────────────────────

def parse_goal(log_file):
    pat = re.compile(r'goal.*?x[=:]\s*([-\d.]+).*?y[=:]\s*([-\d.]+)', re.I)
    with open(log_file) as f:
        for line in f:
            m = pat.search(line)
            if m:
                return float(m.group(1)), float(m.group(2))
    return None, None


def parse_arrive_time(log_file):
    pat = re.compile(r'\[(\d+\.\d+)\].*AutowareState.*ArrivedGoal')
    with open(log_file) as f:
        for line in f:
            m = pat.search(line)
            if m:
                return float(m.group(1))
    return None


def parse_goal_from_bag(bag_dir):
    """从 bag /planning/mission_planning/route 首条消息提取 goal_pose (x, y)。"""
    route_topic = '/planning/mission_planning/route'
    try:
        rd = make_reader(bag_dir, [route_topic])
        tm = type_map(rd)
        if route_topic not in tm:
            return None, None
        while rd.has_next():
            topic, data, _ = rd.read_next()
            msg = deser(data, tm[topic])
            gp = msg.goal_pose
            return gp.position.x, gp.position.y
    except Exception as e:
        print(f'  [warn] parse_goal_from_bag failed: {e}')
    return None, None


def get_bag_time_range(bag_dir):
    """Return (start_sec, end_sec) of the bag based on control_cmd or kinematic_state."""
    for topic in ['/control/command/control_cmd', '/localization/kinematic_state']:
        try:
            rd = make_reader(bag_dir, [topic])
            tm = type_map(rd)
            stamps = []
            while rd.has_next():
                _, data, bag_ts = rd.read_next()
                stamps.append(bag_ts / 1e9)
                if len(stamps) > 2:
                    break
            if stamps:
                # read last stamp using bag_ts (reception time)
                rd2 = make_reader(bag_dir, [topic])
                last_ts = None
                while rd2.has_next():
                    _, _, bag_ts = rd2.read_next()
                    last_ts = bag_ts / 1e9
                return stamps[0], last_ts
        except Exception:
            continue
    return None, None


def fallback_arrive_time(bag_dir, goal_xy):
    gx, gy = goal_xy
    rd = make_reader(bag_dir, ['/localization/kinematic_state'])
    tm = type_map(rd)
    while rd.has_next():
        topic, data, _ = rd.read_next()
        msg = deser(data, tm[topic])
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.hypot(x - gx, y - gy) <= 2.0:
            return stamp_sec(msg.header.stamp)
    return None


# ── full-bag extraction ───────────────────────────────────────────────────────

def read_control_cmd(bag_dir):
    """Return sorted list of (stamp_sec, velocity)."""
    rows = []
    rd = make_reader(bag_dir, ['/control/command/control_cmd'])
    tm = type_map(rd)
    while rd.has_next():
        topic, data, _ = rd.read_next()
        msg = deser(data, tm[topic])
        rows.append((stamp_sec(msg.stamp), msg.longitudinal.velocity))
    rows.sort()
    return rows


def read_kinematic_state(bag_dir):
    """Return sorted list of (stamp_sec, x, y)."""
    rows = []
    rd = make_reader(bag_dir, ['/localization/kinematic_state'])
    tm = type_map(rd)
    while rd.has_next():
        topic, data, _ = rd.read_next()
        msg = deser(data, tm[topic])
        p = msg.pose.pose.position
        rows.append((stamp_sec(msg.header.stamp), p.x, p.y))
    rows.sort()
    return rows


def write_control_cmd_csv(rows, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['header_stamp', 'velocity'])
        for s, v in rows:
            w.writerow([f'{s:.9f}', f'{v:.6f}'])
    print(f'  control_cmd.csv: {len(rows)} rows -> {path}')


def write_kinematic_csv(rows, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['header_stamp', 'x', 'y'])
        for s, x, y in rows:
            w.writerow([f'{s:.9f}', f'{x:.6f}', f'{y:.6f}'])
    print(f'  kinematic_state.csv: {len(rows)} rows -> {path}')


# ── zero_vel_time (速度突变点) ────────────────────────────────────────────────

JUMP_THRESHOLD = 1.0  # |dv| > 此值视为突变 (m/s)


def find_jump_near_arrive(rows, arrive_time, threshold=JUMP_THRESHOLD):
    """找 arrive_time 之前最近的一次大减速跳变（dv < -threshold）。
    返回 (jump_stamp_curr, dv, v_prev, v_curr)，时间戳取突变后那一帧的 header_stamp。
    若无减速突变，回退为旧的 backward-scan 法。
    """
    # 只看 arrive_time 之前的帧
    prior = [(s, v) for s, v in rows if s <= arrive_time + 1.0]

    last_jump = None  # (stamp_curr, dv, v_prev, v_curr) 最近的大减速跳变
    for i in range(1, len(prior)):
        dv = prior[i][1] - prior[i - 1][1]
        if dv < -threshold:
            last_jump = (prior[i][0], dv, prior[i - 1][1], prior[i][1])

    if last_jump is not None:
        return last_jump  # stamp_curr, dv, v_prev, v_curr

    # 回退：backward-scan 找最后非零速度的下一帧
    for i in range(len(prior) - 1, -1, -1):
        if abs(prior[i][1]) >= EPS:
            fallback = prior[i + 1][0] if (i + 1) < len(prior) else prior[i][0]
            return fallback, 0.0, prior[i][1], 0.0
    return (prior[0][0] if prior else arrive_time), 0.0, 0.0, 0.0


# ── step-change analysis ──────────────────────────────────────────────────────

def top_step_changes(rows, n):
    """Sort by |dv| descending. Returns list of (stamp_i, dv, dt, v_prev, v_curr, est_acc)."""
    out = []
    for i in range(1, len(rows)):
        dt = rows[i][0] - rows[i - 1][0]
        if dt <= 0:
            continue
        dv = rows[i][1] - rows[i - 1][1]
        out.append((abs(dv), rows[i][0], dv, dt, rows[i - 1][1], rows[i][1], dv / dt))
    out.sort(key=lambda x: -x[0])
    return out[:n]


# ── summary writers ───────────────────────────────────────────────────────────

def write_summary_full(rows, csv_path, png_path, out_path):
    stamps = [r[0] for r in rows]
    vels = [r[1] for r in rows]
    base = stamps[0]
    changes = top_step_changes(rows, 5)
    lines = [
        f'input: {csv_path}',
        f'output_plot: {png_path}',
        f'samples: {len(rows)}',
        f'start_stamp: {stamps[0]:.9f}',
        f'end_stamp: {stamps[-1]:.9f}',
        f'duration_s: {stamps[-1] - stamps[0]:.6f}',
        f'min_velocity_mps: {min(vels):.6f}',
        f'max_velocity_mps: {max(vels):.6f}',
        '',
        'top_velocity_step_changes:',
    ]
    for (_, stamp, dv, dt, prev, curr, acc) in changes:
        rel = stamp - base
        lines.append(
            f'  rel_t={rel:.6f}s dv={dv:.6f}m/s dt={dt:.6f}s'
            f' prev={prev:.6f} curr={curr:.6f} est_acc={acc:.6f}m/s^2'
        )
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def write_summary_window(rows_win, sec0, sec1, full_csv, win_csv, png_path, out_path, include_acc):
    if not rows_win:
        return
    stamps = [r[0] for r in rows_win]
    vels = [r[1] for r in rows_win]
    changes = top_step_changes(rows_win, 8)
    lines = [
        f'input: {full_csv}',
        f'filtered_csv: {win_csv}',
        f'output_plot: {png_path}',
        f'time_window_s: [{float(sec0):.9f}, {float(sec1):.9f}]',
        f'samples: {len(rows_win)}',
        f'first_stamp: {stamps[0]:.9f}',
        f'last_stamp: {stamps[-1]:.9f}',
        f'duration_s: {stamps[-1] - stamps[0]:.6f}',
        f'min_velocity_mps: {min(vels):.6f}',
        f'max_velocity_mps: {max(vels):.6f}',
        '',
        'top_velocity_step_changes:',
    ]
    for (_, stamp, dv, dt, prev, curr, acc) in changes:
        rel = stamp - sec0
        if include_acc:
            lines.append(
                f'  rel_t={rel:.6f}s stamp={stamp:.9f} dv={dv:.6f}m/s dt={dt:.6f}s'
                f' prev={prev:.6f} curr={curr:.6f} est_acc={acc:.6f}m/s^2'
            )
        else:
            lines.append(
                f'  rel_t={rel:.6f}s stamp={stamp:.9f} dv={dv:.6f}m/s dt={dt:.6f}s'
                f' prev={prev:.6f} curr={curr:.6f}'
            )
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


# ── plots ─────────────────────────────────────────────────────────────────────

def plot_full(rows, out_png):
    if not rows:
        return
    stamps = [r[0] for r in rows]
    base = stamps[0]
    rel = [s - base for s in stamps]
    vels = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(rel, vels)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('velocity (m/s)')
    ax.set_title('control_cmd velocity (full bag)')
    fig.tight_layout()
    fig.savefig(out_png, dpi=100)
    plt.close(fig)


def plot_window(rows, sec0, out_png, show_acc=True):
    if not rows:
        return
    stamps = [r[0] for r in rows]
    rel = [s - sec0 for s in stamps]
    vels = [r[1] for r in rows]
    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax1.plot(rel, vels, 'b-', label='velocity')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('velocity (m/s)')
    if show_acc and len(rows) > 1:
        accs = [0.0]
        for i in range(1, len(rows)):
            dt = rows[i][0] - rows[i - 1][0]
            accs.append((rows[i][1] - rows[i - 1][1]) / dt if dt > 0 else 0.0)
        ax2 = ax1.twinx()
        ax2.plot(rel, accs, 'r--', alpha=0.5, label='est_acc')
        ax2.set_ylabel('est_acc (m/s²)')
    ax1.set_title(f'control_cmd [{sec0}, {sec0 + 2}]')
    fig.tight_layout()
    fig.savefig(out_png, dpi=100)
    plt.close(fig)


# ── velocity_smoother bundle ──────────────────────────────────────────────────

SMOOTHER = [
    ('raw',               '/planning/scenario_planning/velocity_smoother/debug/trajectory_raw'),
    ('ext_vel_limited',   '/planning/scenario_planning/velocity_smoother/debug/trajectory_external_velocity_limited'),
    ('lat_acc',           '/planning/scenario_planning/velocity_smoother/debug/trajectory_lateral_acc_filtered'),
    ('steer_rate_limited','/planning/scenario_planning/velocity_smoother/debug/trajectory_steering_rate_limited'),
    ('time_resampled',    '/planning/scenario_planning/velocity_smoother/debug/trajectory_time_resampled'),
    ('forward_filtered',  '/planning/scenario_planning/velocity_smoother/debug/forward_filtered_trajectory'),
    ('backward_filtered', '/planning/scenario_planning/velocity_smoother/debug/backward_filtered_trajectory'),
    ('merged_filtered',   '/planning/scenario_planning/velocity_smoother/debug/merged_filtered_trajectory'),
    ('output',            '/planning/scenario_planning/velocity_smoother/trajectory'),
]

STAGE_LABELS = [
    'raw', 'external limited', 'lateral acc', 'steer rate',
    'time resampled', 'forward filtered', 'backward filtered',
    'merged filtered', 'velocity_smoother out',
]


def adj_jump(vs):
    """Return (max_jump, jump_index, v[i], v[i+1])."""
    best = (0.0, 0, 0.0, 0.0)
    for i in range(len(vs) - 1):
        j = abs(vs[i + 1] - vs[i])
        if j > best[0]:
            best = (j, i, vs[i], vs[i + 1])
    return best


def save_traj_txt_png(msg, bag_ts, idx, out_dir):
    """Write NNN_ts.txt + .png for one Trajectory message. Returns meta dict."""
    pts = msg.points
    xs = [p.pose.position.x for p in pts]
    vs = [p.longitudinal_velocity_mps for p in pts]
    n = len(pts)
    max_j, ji, va, vb = adj_jump(vs)

    stem = f'{idx:03d}_{bag_ts}'
    txt_path = os.path.join(out_dir, stem + '.txt')
    png_path = os.path.join(out_dir, stem + '.png')

    with open(txt_path, 'w') as f:
        f.write(f'bag_timestamp_ns: {bag_ts}\n')
        f.write(f'header_sec: {msg.header.stamp.sec}\n')
        f.write(f'point_count: {n}\n')
        f.write(f'max_adjacent_jump: {max_j:.9f}\n')
        f.write(f'jump_index: {ji}\n')
        f.write(f'jump_values: {va:.9f} -> {vb:.9f}\n')
        f.write('\n')
        f.write('idx,x,longitudinal_velocity_mps\n')
        for i, (x, v) in enumerate(zip(xs, vs)):
            f.write(f'{i},{x:.9f},{v:.9f}\n')

    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(range(n), vs)
    ax.set_xlabel('idx')
    ax.set_ylabel('longitudinal_velocity_mps')
    fig.tight_layout()
    fig.savefig(png_path, dpi=100)
    plt.close(fig)

    return {
        'bag_ts': bag_ts,
        'n': n,
        'max_j': max_j,
        'ji': ji,
        'va': va,
        'vb': vb,
        'xs': xs,
        'vs': vs,
        'hdr_stamp': stamp_sec(msg.header.stamp),
    }


def dump_smoother_bundle(bag_dir, zero_vel_sec, plots_dir):
    """Dump 9-topic bundle. Returns per_sub dict: sub -> [meta, ...]."""
    for sub, _ in SMOOTHER:
        os.makedirs(os.path.join(plots_dir, sub), exist_ok=True)

    all_topics = [tp for _, tp in SMOOTHER]
    rd = make_reader(bag_dir, all_topics)
    tm = type_map(rd)

    by_topic = {tp: [] for _, tp in SMOOTHER}
    while rd.has_next():
        topic, data, bag_ts = rd.read_next()
        if topic not in by_topic:
            continue
        msg = deser(data, tm[topic])
        if msg.header.stamp.sec == zero_vel_sec:
            by_topic[topic].append((bag_ts, msg))

    per_sub = {}
    total = 0
    smry = [
        f'bag: {os.path.relpath(bag_dir)}',
        f'target_sec: {zero_vel_sec}',
        f'filter: header_sec == {zero_vel_sec}',
        '',
    ]

    for sub, tp in SMOOTHER:
        msgs = by_topic[tp]
        out_dir = os.path.join(plots_dir, sub)
        metas = []
        for i, (bag_ts, msg) in enumerate(msgs):
            metas.append(save_traj_txt_png(msg, bag_ts, i, out_dir))
        per_sub[sub] = metas
        total += len(msgs)
        smry += [
            f'[{sub}]  topic={tp}',
            f'  msg_count={len(msgs)}',
            f'  out_dir={os.path.relpath(out_dir)}',
            '',
        ]

    smry.insert(4, f'total_plots: {total}')
    with open(os.path.join(plots_dir, 'summary.txt'), 'w') as f:
        f.write('\n'.join(smry) + '\n')

    print(f'  velocity_smoother bundle: total_plots={total} -> {plots_dir}')
    return per_sub


# ── stage overlay ─────────────────────────────────────────────────────────────

def dump_stage_overlay(win_rows, per_sub, zero_vel_sec, out_dir, bag_dir):
    """Write stage_overlay.{txt,png}. win_rows = 2s window control_cmd rows."""
    # control_jump_stamp = stamp of largest |dv/dt| step in 2s window
    changes = top_step_changes(win_rows, 1)
    if changes:
        jump_stamp = changes[0][1]
    elif win_rows:
        jump_stamp = win_rows[-1][0]
    else:
        jump_stamp = float(zero_vel_sec)

    jump_ns = int(jump_stamp * 1e9)

    stage_sample_index = 0
    overlay = []

    for (sub, _), label in zip(SMOOTHER, STAGE_LABELS):
        metas = per_sub.get(sub, [])
        sel = None
        sel_idx = 0
        for i, m in enumerate(metas):
            if m['bag_ts'] <= jump_ns:
                sel = m
                sel_idx = i
        if sel is None and metas:
            sel = metas[0]
            sel_idx = 0
        if sub == 'raw':
            stage_sample_index = sel_idx

        if sel:
            vs = sel['vs']
            v_min = min(vs) if vs else 0.0
            v_max = max(vs) if vs else 0.0
            overlay.append({
                'label': label, 'ts_ns': sel['bag_ts'], 'n': sel['n'],
                'v_min': v_min, 'v_max': v_max,
                'max_j': sel['max_j'], 'va': sel['va'], 'vb': sel['vb'],
                'xs': sel['xs'], 'vs': vs,
            })
        else:
            overlay.append({
                'label': label, 'ts_ns': 0, 'n': 0,
                'v_min': 0.0, 'v_max': 0.0,
                'max_j': 0.0, 'va': 0.0, 'vb': 0.0,
                'xs': [], 'vs': [],
            })

    # Write txt
    txt_path = os.path.join(out_dir, f'velocity_smoother_sec_{zero_vel_sec}_stage_overlay.txt')
    lines = [
        f'source_bag: {bag_dir}',
        f'control_jump_stamp: {jump_stamp:.9f}',
        f'stage_sample_index: {stage_sample_index:03d}',
        f'note: stage sample is the velocity_smoother cycle immediately before the control_cmd drop.',
        '',
    ]
    for o in overlay:
        lines.append(
            f'{o["label"]}: ts_ns={o["ts_ns"]} points={o["n"]}'
            f' v_range=[{o["v_min"]:.6f}, {o["v_max"]:.6f}]'
            f' max_adjacent_jump={o["max_j"]:.9f}'
            f' jump_values={o["va"]:.9f} -> {o["vb"]:.9f}'
        )
    with open(txt_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    # Write png
    png_path = os.path.join(out_dir, f'velocity_smoother_sec_{zero_vel_sec}_stage_overlay.png')
    fig, ax = plt.subplots(figsize=(12, 5))
    for o in overlay:
        if o['xs'] and o['vs']:
            ax.plot(o['xs'], o['vs'], label=o['label'])
    ax.set_xlabel('x')
    ax.set_ylabel('longitudinal_velocity_mps')
    ax.set_title(f'velocity_smoother stages  sec={zero_vel_sec}')
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(png_path, dpi=100)
    plt.close(fig)

    print(f'  stage_overlay -> {txt_path}')


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('log_file')
    ap.add_argument('out_dir', nargs='?', default=None)
    args = ap.parse_args()

    bag_dir  = os.path.abspath(args.bag_dir)
    log_file = os.path.abspath(args.log_file)
    out_dir  = os.path.abspath(args.out_dir) if args.out_dir else bag_dir.rstrip('/') + '_logs'
    os.makedirs(out_dir, exist_ok=True)

    print(f'bag_dir  : {bag_dir}')
    print(f'log_file : {log_file}')
    print(f'out_dir  : {out_dir}')

    # ── Step 1: goal + arrive_time ─────────────────────────────────────────
    print('\n=== Step 1: goal + arrive_time ===')
    bag_t0, bag_t1 = get_bag_time_range(bag_dir)
    print(f'  bag time range: [{bag_t0:.3f}, {bag_t1:.3f}]')

    goal_xy = parse_goal(log_file)
    arrive_time = parse_arrive_time(log_file)

    # 若 log 的 arrive_time 不在 bag 时间范围内，丢弃
    if arrive_time is not None and bag_t0 is not None:
        if not (bag_t0 - 5 <= arrive_time <= bag_t1 + 5):
            print(f'  [warn] log arrive_time {arrive_time:.3f} outside bag range, discarding')
            arrive_time = None

    if goal_xy[0] is None:
        print('  goal not found in log, falling back to bag /planning/mission_planning/route')
        goal_xy = parse_goal_from_bag(bag_dir)
    if arrive_time is None:
        print('  ArrivedGoal not found in log (or out of range), using distance fallback')
        if goal_xy[0] is not None:
            arrive_time = fallback_arrive_time(bag_dir, goal_xy)
        else:
            print('  [warn] no goal available, using bag end time as arrive_time fallback')
            cmd_tmp = read_control_cmd(bag_dir)
            arrive_time = cmd_tmp[-1][0] if cmd_tmp else None
    print(f'  goal: x={goal_xy[0]}, y={goal_xy[1]}')
    print(f'  arrive_time: {arrive_time} s')

    # Copy LOG_FILE as {NN}_traj.log
    nn = re.search(r'(\d+)', os.path.basename(log_file))
    nn_str = nn.group(1) if nn else '00'
    traj_log_dst = os.path.join(out_dir, f'{nn_str}_traj.log')
    shutil.copy2(log_file, traj_log_dst)
    print(f'  copied {os.path.basename(log_file)} -> {traj_log_dst}')

    # ── Step 2: full-bag extraction ────────────────────────────────────────
    print('\n=== Step 2: full-bag control_cmd + kinematic_state ===')
    cmd_rows = read_control_cmd(bag_dir)
    kin_rows = read_kinematic_state(bag_dir)
    cmd_csv  = os.path.join(out_dir, 'control_cmd.csv')
    kin_csv  = os.path.join(out_dir, 'kinematic_state.csv')
    write_control_cmd_csv(cmd_rows, cmd_csv)
    write_kinematic_csv(kin_rows, kin_csv)

    # ── Step 3: zero_vel_time（速度突变点） ─────────────────────────────────
    print('\n=== Step 3: zero_vel_sec (velocity jump point) ===')
    jump_stamp, jump_dv, jump_v_prev, jump_v_curr = find_jump_near_arrive(cmd_rows, arrive_time)
    zero_vel_sec = int(math.floor(jump_stamp))
    sec0, sec1   = zero_vel_sec, zero_vel_sec + 2
    print(f'  jump_stamp = {jump_stamp:.9f} s  (dv={jump_dv:+.6f} m/s, {jump_v_prev:.6f} -> {jump_v_curr:.6f})')
    print(f'  zero_vel_sec  = {zero_vel_sec}')
    print(f'  window: [{sec0}, {sec1}]')

    win_rows = [(s, v) for s, v in cmd_rows if sec0 <= s <= sec1]
    win_csv  = os.path.join(out_dir, f'control_cmd_{sec0}_{sec1}.csv')
    with open(win_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['header_stamp', 'relative_time_s', 'velocity'])
        for s, v in win_rows:
            w.writerow([f'{s:.9f}', f'{s - sec0:.9f}', f'{v:.6f}'])
    print(f'  window control_cmd: {len(win_rows)} rows -> {win_csv}')

    # ── Step 4: plots + summaries ──────────────────────────────────────────
    print('\n=== Step 4: plots ===')

    # Full bag: velocity curve
    full_png  = os.path.join(out_dir, 'control_cmd_velocity_curve.png')
    full_smry = os.path.join(out_dir, 'control_cmd_velocity_curve_summary.txt')
    plot_full(cmd_rows, full_png)
    write_summary_full(cmd_rows, cmd_csv, full_png, full_smry)
    print(f'  full-bag plot -> {full_png}')

    # Window: velocity + acc curve
    win_png  = os.path.join(out_dir, f'control_cmd_velocity_curve_{sec0}_{sec1}.png')
    win_smry = os.path.join(out_dir, f'control_cmd_velocity_curve_{sec0}_{sec1}_summary.txt')
    plot_window(win_rows, sec0, win_png, show_acc=True)
    write_summary_window(win_rows, sec0, sec1, cmd_csv, win_csv, win_png, win_smry, include_acc=True)
    print(f'  window curve plot -> {win_png}')

    # Window: velocity only
    only_png  = os.path.join(out_dir, f'control_cmd_velocity_only_{sec0}_{sec1}.png')
    only_smry = os.path.join(out_dir, f'control_cmd_velocity_only_{sec0}_{sec1}_summary.txt')
    plot_window(win_rows, sec0, only_png, show_acc=False)
    write_summary_window(win_rows, sec0, sec1, cmd_csv, win_csv, only_png, only_smry, include_acc=False)
    print(f'  window only plot -> {only_png}')

    # ── Step 5: velocity_smoother bundle ──────────────────────────────────
    print('\n=== Step 5: velocity_smoother bundle ===')
    plots_dir = os.path.join(out_dir, f'velocity_smoother_sec_{zero_vel_sec}_plots')
    per_sub = dump_smoother_bundle(bag_dir, zero_vel_sec, plots_dir)

    # ── Step 6: stage overlay ─────────────────────────────────────────────
    print('\n=== Step 6: stage overlay ===')
    dump_stage_overlay(win_rows, per_sub, zero_vel_sec, out_dir, bag_dir)

    print(f'\n=== Done ===\nOutput: {out_dir}')


if __name__ == '__main__':
    main()
