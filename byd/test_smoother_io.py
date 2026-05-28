#!/usr/bin/env python3
"""
velocity_smoother 输入输出测试脚本

功能：
  向运行中的 velocity_smoother 发布指定测试轨迹，打印输入/输出速度对比。

前提：
  velocity_smoother 节点已启动（或 Autoware 正在运行）。

用法：
  python3 src/byd/test_smoother_io.py [选项]

选项：
  --v0        FLOAT   ego 当前速度 (m/s)，默认 0.0
  --max-vel   FLOAT   直线轨迹中间段最大速度 (m/s)，默认 3.0
  --length    FLOAT   直线轨迹总长度 (m)，默认 30.0
  --num-pts   INT     轨迹点数，默认 60
  --csv       PATH    从 CSV 读取自定义轨迹 (列: x,y,yaw_deg,vel_mps)
                      最后一行 vel_mps 会被强制设为 0
  --timeout   FLOAT   等待输出超时秒数，默认 5.0
  --ns        STR     velocity_smoother 话题命名空间前缀, 默认 ""
                      例如 /planning/scenario_planning

示例：
  # 发布一段 30m 直线，最高速 3m/s，ego 静止
  python3 src/byd/test_smoother_io.py

  # ego 以 2m/s 行驶，发布 20m 轨迹
  python3 src/byd/test_smoother_io.py --v0 2.0 --length 20 --max-vel 2.0

  # 从 CSV 文件读取自定义轨迹
  python3 src/byd/test_smoother_io.py --csv my_traj.csv
"""

import argparse
import csv
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import AccelWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry


# ── 辅助：由 yaw 角生成四元数 ──────────────────────────────────────────
def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


# ── 构造直线测试轨迹 ────────────────────────────────────────────────────
def make_straight_trajectory(length: float, num_pts: int, max_vel: float) -> list[dict]:
    """
    返回轨迹点列表，每点含 x, y, yaw_deg, vel_mps。
    速度剖面：前半段加速，后半段保持，终点强制 0。
    """
    pts = []
    for i in range(num_pts):
        x = i * length / (num_pts - 1)
        ratio = i / (num_pts - 1)
        # 梯形速度：前 20% 线性升到 max_vel，中间 60% 保持，后 20% 线性降到 0
        if ratio < 0.2:
            vel = max_vel * (ratio / 0.2)
        elif ratio < 0.8:
            vel = max_vel
        else:
            vel = max_vel * (1.0 - (ratio - 0.8) / 0.2)
        pts.append({"x": x, "y": 0.0, "yaw_deg": 0.0, "vel_mps": vel})
    pts[-1]["vel_mps"] = 0.0
    return pts


# ── 从 CSV 读取自定义轨迹 ───────────────────────────────────────────────
def load_csv_trajectory(path: str) -> list[dict]:
    pts = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append(
                {
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "yaw_deg": float(row.get("yaw_deg", "0")),
                    "vel_mps": float(row["vel_mps"]),
                }
            )
    if pts:
        pts[-1]["vel_mps"] = 0.0
    return pts


# ── 把 dict 列表转成 Trajectory 消息 ────────────────────────────────────
def make_trajectory_msg(pts: list[dict]) -> Trajectory:
    msg = Trajectory()
    msg.header.frame_id = "map"
    for p in pts:
        tp = TrajectoryPoint()
        tp.pose.position.x = p["x"]
        tp.pose.position.y = p["y"]
        yaw = math.radians(p["yaw_deg"])
        tp.pose.orientation = yaw_to_quat(yaw)
        tp.longitudinal_velocity_mps = float(p["vel_mps"])
        msg.points.append(tp)
    return msg


# ── 测试节点 ─────────────────────────────────────────────────────────────
class SmootherTestNode(Node):
    def __init__(self, args):
        super().__init__("smoother_io_test")

        self.args = args
        self.received_output = None
        self._lock = threading.Lock()

        # 话题名称（与 launch 文件 remap 一致）
        input_traj_topic = "/planning/scenario_planning/scenario_selector/trajectory"
        output_traj_topic = "/planning/scenario_planning/trajectory"
        odom_topic = "/localization/kinematic_state"
        accel_topic = "/localization/acceleration"
        op_mode_topic = "/system/operation_mode/state"
        vel_limit_topic = "/planning/scenario_planning/max_velocity"

        # Publishers
        self.pub_traj = self.create_publisher(Trajectory, input_traj_topic, 1)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 1)
        self.pub_accel = self.create_publisher(AccelWithCovarianceStamped, accel_topic, 1)

        # OperationMode: transient_local QoS
        op_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.pub_op_mode = self.create_publisher(OperationModeState, op_mode_topic, op_qos)

        # VelocityLimit: transient_local QoS
        vl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.pub_vel_limit = self.create_publisher(VelocityLimit, vel_limit_topic, vl_qos)

        # Subscriber
        self.sub_output = self.create_subscription(
            Trajectory, output_traj_topic, self._on_output, 1
        )

        self.get_logger().info(f"输入话题: {input_traj_topic}")
        self.get_logger().info(f"输出话题: {output_traj_topic}")

    def _on_output(self, msg: Trajectory):
        with self._lock:
            if self.received_output is None:
                self.received_output = msg
                self.get_logger().info(f"收到输出轨迹，共 {len(msg.points)} 点")

    def _publish_background(self, v0: float, stop_event: threading.Event):
        """持续发布 odometry / accel / op_mode，直到 stop_event 被设置。"""
        rate_hz = 20
        while not stop_event.is_set():
            now = self.get_clock().now().to_msg()

            # odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = v0
            self.pub_odom.publish(odom)

            # acceleration
            accel = AccelWithCovarianceStamped()
            accel.header.stamp = now
            accel.header.frame_id = "base_link"
            # covariance: set diagonal to small positive
            accel.accel.covariance[0] = 0.01
            self.pub_accel.publish(accel)

            # operation mode: AUTONOMOUS
            op = OperationModeState()
            op.stamp = now
            op.mode = OperationModeState.AUTONOMOUS
            op.is_autoware_control_enabled = True
            self.pub_op_mode.publish(op)

            time.sleep(1.0 / rate_hz)

    def run_test(self, pts: list[dict]) -> bool:
        """
        执行一次测试：发布轨迹，等待输出，打印对比。
        返回 True 表示成功收到输出。
        """
        v0 = self.args.v0

        # 发布最大速度限制
        vl = VelocityLimit()
        vl.stamp = self.get_clock().now().to_msg()
        vl.max_velocity = 50.0  # 不限制，让 smoother 自行决定
        self.pub_vel_limit.publish(vl)

        # 启动后台发布线程
        stop_event = threading.Event()
        bg_thread = threading.Thread(
            target=self._publish_background, args=(v0, stop_event), daemon=True
        )
        bg_thread.start()

        # 等待后台话题稳定
        self.get_logger().info("等待后台话题就绪 (0.5s)...")
        time.sleep(0.5)

        # 发布测试轨迹
        traj_msg = make_trajectory_msg(pts)
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_traj.publish(traj_msg)
        self.get_logger().info(f"已发布输入轨迹，共 {len(pts)} 点")

        # 等待输出
        deadline = time.time() + self.args.timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                if self.received_output is not None:
                    break

        stop_event.set()

        with self._lock:
            if self.received_output is None:
                self.get_logger().error(
                    f"超时 {self.args.timeout}s 未收到输出！请确认 velocity_smoother 节点正在运行。"
                )
                return False

        self._print_comparison(pts, self.received_output)
        return True

    def _print_comparison(self, input_pts: list[dict], output_msg: Trajectory):
        """格式化打印输入与输出速度剖面对比。"""
        out_pts = output_msg.points

        print("\n" + "=" * 70)
        print("  velocity_smoother 输入 / 输出 速度剖面对比")
        print("=" * 70)
        print(f"  输入点数: {len(input_pts)}   输出点数: {len(out_pts)}")
        print("-" * 70)
        print(f"  {'idx':>4}  {'in_x':>8}  {'in_vel(m/s)':>12}  "
              f"{'out_x':>8}  {'out_vel(m/s)':>12}  {'out_acc':>9}")
        print("-" * 70)

        max_rows = max(len(input_pts), len(out_pts))
        for i in range(max_rows):
            # 输入
            if i < len(input_pts):
                ix = input_pts[i]["x"]
                iv = input_pts[i]["vel_mps"]
                in_str = f"{ix:8.3f}  {iv:12.4f}"
            else:
                in_str = " " * 22

            # 输出
            if i < len(out_pts):
                op = out_pts[i]
                ox = op.pose.position.x
                ov = op.longitudinal_velocity_mps
                oa = op.acceleration_mps2
                out_str = f"{ox:8.3f}  {ov:12.4f}  {oa:9.4f}"
            else:
                out_str = ""

            print(f"  {i:4d}  {in_str}  {out_str}")

        print("=" * 70)

        # 统计
        if out_pts:
            vels = [p.longitudinal_velocity_mps for p in out_pts]
            print(f"\n  输出速度统计: max={max(vels):.4f}  min={min(vels):.4f}  "
                  f"avg={sum(vels)/len(vels):.4f} m/s")
        print()


# ── 主程序 ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="velocity_smoother 输入输出测试脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--v0", type=float, default=0.0, help="ego 当前速度 (m/s), 默认 0.0")
    parser.add_argument("--max-vel", type=float, default=3.0,
                        help="直线轨迹最大速度 (m/s), 默认 3.0")
    parser.add_argument("--length", type=float, default=30.0,
                        help="直线轨迹总长度 (m), 默认 30.0")
    parser.add_argument("--num-pts", type=int, default=60,
                        help="直线轨迹点数, 默认 60")
    parser.add_argument("--csv", type=str, default=None,
                        help="从 CSV 读取自定义轨迹 (列: x,y,yaw_deg,vel_mps)")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="等待输出超时 (s), 默认 5.0")
    args = parser.parse_args()

    # 构造测试轨迹
    if args.csv:
        pts = load_csv_trajectory(args.csv)
        print(f"从 CSV 读取轨迹：{len(pts)} 点  文件：{args.csv}")
    else:
        pts = make_straight_trajectory(args.length, args.num_pts, args.max_vel)
        print(f"使用内置直线轨迹：{len(pts)} 点  长度={args.length}m  最高速={args.max_vel}m/s")

    print(f"ego 初始速度 v0={args.v0} m/s")

    # 运行 ROS2 节点
    rclpy.init(args=None)
    node = SmootherTestNode(args)

    try:
        success = node.run_test(pts)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
