#!/usr/bin/env python3
"""
goal_error_monitor.py
---------------------
订阅 /planning/mission_planning/route 和 /localization/kinematic_state，
实时计算并打印车辆当前位置与 goal_pose 的误差。

用法：
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/byd/goal_error_monitor.py [--once] [--interval 0.5]

选项：
    --once       收到一帧 odometry 后打印一次误差，然后退出（适合脚本化）
    --interval   打印间隔（秒），默认 0.5；0 = 每帧都打印
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Odometry


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class GoalErrorMonitor(Node):
    def __init__(self, once: bool, interval: float):
        super().__init__("goal_error_monitor")
        self._once = once
        self._interval = interval
        self._goal_pose = None
        self._last_print = 0.0

        # route: transient_local 保证拿到历史消息
        route_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                                 self._on_route, route_qos)

        odom_qos = QoSProfile(depth=1,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._on_odom, odom_qos)

        self.get_logger().info("等待 route + kinematic_state ...")

    def _on_route(self, msg: LaneletRoute):
        self._goal_pose = msg.goal_pose
        gp = self._goal_pose.position
        self.get_logger().info(
            f"[Route] goal_pose: x={gp.x:.6f}  y={gp.y:.6f}  z={gp.z:.6f}"
        )

    def _on_odom(self, msg: Odometry):
        if self._goal_pose is None:
            return

        now = time.monotonic()
        if self._interval > 0 and (now - self._last_print) < self._interval:
            return
        self._last_print = now

        gp = self._goal_pose.position
        go = self._goal_pose.orientation
        ep = msg.pose.pose.position
        eo = msg.pose.pose.orientation
        ev = msg.twist.twist.linear.x

        dx = ep.x - gp.x
        dy = ep.y - gp.y
        dz = ep.z - gp.z
        dist_2d = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

        yaw_goal = yaw_from_quat(go.x, go.y, go.z, go.w)
        yaw_ego  = yaw_from_quat(eo.x, eo.y, eo.z, eo.w)
        dyaw = (yaw_ego - yaw_goal + math.pi) % (2 * math.pi) - math.pi

        # 在 goal 坐标系下投影纵向/横向误差
        s_long = dx * math.cos(yaw_goal) + dy * math.sin(yaw_goal)
        s_lat  = -dx * math.sin(yaw_goal) + dy * math.cos(yaw_goal)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        print(
            f"[{stamp:.3f}]  "
            f"dist_2d={dist_2d*100:+7.2f} cm  "
            f"s_long={s_long*100:+7.2f} cm  "
            f"s_lat={s_lat*100:+7.2f} cm  "
            f"dyaw={math.degrees(dyaw):+6.2f}°  "
            f"ego_v={ev:+5.3f} m/s",
            flush=True,
        )

        if self._once:
            # 打印详细分解后退出
            print()
            print(f"  ego  : x={ep.x:.9f}  y={ep.y:.9f}  z={ep.z:.9f}")
            print(f"  goal : x={gp.x:.9f}  y={gp.y:.9f}  z={gp.z:.9f}")
            print(f"  Δx={dx*100:+.4f} cm  Δy={dy*100:+.4f} cm  Δz={dz*100:+.4f} cm")
            print(f"  平面距离  = {dist_2d*100:.4f} cm")
            print(f"  3D 距离   = {dist_3d*100:.4f} cm")
            print(f"  纵向偏差  = {s_long*100:+.4f} cm  (>0 已越过 goal)")
            print(f"  横向偏差  = {s_lat*100:+.4f} cm")
            print(f"  航向误差  = {math.degrees(dyaw):+.4f}°")
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="实时打印 ego vs goal 误差")
    parser.add_argument("--once", action="store_true",
                        help="打印一次后退出")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="打印间隔秒（默认 0.5）")
    args = parser.parse_args()

    rclpy.init()
    node = GoalErrorMonitor(once=args.once, interval=args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
