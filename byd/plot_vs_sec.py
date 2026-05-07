#!/usr/bin/env python3
"""
将 velocity_smoother 各阶段轨迹绘制在同一张图中。

横坐标：距终点的路径距离（m），由各点 x 坐标沿轨迹累积计算
纵坐标：longitudinal_velocity_mps

每个子目录对应一条曲线（同一 sec 内多帧取平均或全部绘出，可用 --all-frames）

用法:
  python3 src/byd/plot_vs_sec.py <plots_dir>
  python3 src/byd/plot_vs_sec.py <plots_dir> --all-frames
  python3 src/byd/plot_vs_sec.py <plots_dir> --out fig.png
"""

import argparse
import os
import glob

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# 子目录顺序（velocity_smoother 处理流程顺序）
SUBDIRS_ORDER = [
    ("raw",              "Raw Input"),
    ("ext_vel_limited",  "Ext Vel Limited"),
    ("lat_acc",          "Lat Acc Filtered"),
    ("steer_rate_limited", "Steer Rate Limited"),
    ("time_resampled",   "Time Resampled"),
    ("forward_filtered", "Forward Filtered"),
    ("backward_filtered","Backward Filtered"),
    ("merged_filtered",  "Merged Filtered"),
    ("output",           "Output"),
]


def read_txt(path: str):
    """返回 (x_array, vel_array)"""
    xs, vels = [], []
    in_data = False
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("idx,"):
                in_data = True
                continue
            if in_data and line:
                parts = line.split(",")
                xs.append(float(parts[1]))
                vels.append(float(parts[2]))
    return np.array(xs), np.array(vels)


def x_to_dist_from_end(xs: np.ndarray) -> np.ndarray:
    """用 x 坐标差累积计算各点到终点的路径距离。"""
    if len(xs) < 2:
        return np.zeros(len(xs))
    dx = np.abs(np.diff(xs))
    # 从末尾向前累积
    dist = np.zeros(len(xs))
    for i in range(len(xs) - 2, -1, -1):
        dist[i] = dist[i + 1] + dx[i]
    return dist


def plot(plots_dir: str, all_frames: bool, out_path: str) -> None:
    colors = cm.tab10(np.linspace(0, 1, len(SUBDIRS_ORDER)))

    fig, ax = plt.subplots(figsize=(12, 6))

    plots_dir = os.path.abspath(plots_dir)
    sec = os.path.basename(plots_dir).split("_sec_")[1].split("_")[0] if "_sec_" in os.path.basename(plots_dir) else ""

    for (subdir, label), color in zip(SUBDIRS_ORDER, colors):
        d = os.path.join(plots_dir, subdir)
        if not os.path.isdir(d):
            continue

        txts = sorted(glob.glob(os.path.join(d, "*.txt")))
        if not txts:
            continue

        if all_frames:
            # 绘制该 topic 每一帧，透明度区分
            for i, path in enumerate(txts):
                xs, vels = read_txt(path)
                dist = x_to_dist_from_end(xs)
                alpha = 0.3 + 0.7 * (i / max(len(txts) - 1, 1))
                ax.plot(dist, vels, color=color, alpha=alpha,
                        linewidth=1.0,
                        label=label if i == len(txts) - 1 else "_")
        else:
            # 只绘制最后一帧（最新状态）
            xs, vels = read_txt(txts[-1])
            dist = x_to_dist_from_end(xs)
            ax.plot(dist, vels, color=color, linewidth=1.8, label=label)

    ax.set_xlabel("Distance to endpoint (m)", fontsize=12)
    ax.set_ylabel("longitudinal_velocity_mps (m/s)", fontsize=12)
    title = f"Velocity Smoother Pipeline — sec={sec}"
    if all_frames:
        title += "  (all frames, darker = later)"
    ax.set_title(title, fontsize=13)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.invert_xaxis()   # 左边距终点远，右边距终点近（终点在原点）

    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
        print(f"图片已保存: {out_path}")
    else:
        default = os.path.join(plots_dir, "vs_velocity_profile.png")
        plt.savefig(default, dpi=150)
        print(f"图片已保存: {default}")
    plt.show()


def main():
    ap = argparse.ArgumentParser(
        description="绘制 velocity_smoother 各阶段速度曲线",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("plots_dir", help="velocity_smoother_sec_XXX_plots 目录")
    ap.add_argument("--all-frames", action="store_true",
                    help="绘制每帧（默认只绘最新一帧）")
    ap.add_argument("--out", metavar="FILE", default=None,
                    help="输出图片路径（默认保存到 plots_dir/vs_velocity_profile.png）")
    args = ap.parse_args()

    plot(args.plots_dir, args.all_frames, args.out)


if __name__ == "__main__":
    main()
