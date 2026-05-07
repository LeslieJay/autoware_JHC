#!/usr/bin/env python3
"""
找出 control_cmd.csv 中相邻两行速度差绝对值 > threshold 的 header_stamp。
用法: python3 find_vel_jump.py <csv_path> [threshold]
默认 threshold=1.0 m/s
"""
import csv
import sys

def main():
    csv_path  = sys.argv[1] if len(sys.argv) > 1 else 'control_cmd.csv'
    threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((float(row['header_stamp']), float(row['velocity'])))

    print(f'csv: {csv_path}')
    print(f'threshold: |dv| > {threshold} m/s')
    print(f'total rows: {len(rows)}')
    print()
    print(f'{"#":<5}  {"header_stamp_prev":>22}  {"header_stamp_curr":>22}  {"v_prev":>10}  {"v_curr":>10}  {"dv":>10}  {"dt_ms":>8}')
    print('-' * 100)

    count = 0
    for i in range(1, len(rows)):
        s_prev, v_prev = rows[i - 1]
        s_curr, v_curr = rows[i]
        dv = v_curr - v_prev
        if abs(dv) > threshold:
            dt_ms = (s_curr - s_prev) * 1000.0
            count += 1
            print(f'{count:<5}  {s_prev:>22.9f}  {s_curr:>22.9f}  {v_prev:>10.6f}  {v_curr:>10.6f}  {dv:>+10.6f}  {dt_ms:>8.3f}')

    print()
    print(f'total jumps: {count}')

if __name__ == '__main__':
    main()
