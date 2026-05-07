#!/usr/bin/env python3
"""
实时监听话题，当车辆距轨迹终点 < dist_thresh 米时，
打印第一次 /control/command/control_cmd 中 velocity == 0 的 header_stamp，
然后自动退出。

用法:
    python3 src/byd/find_stop_stamp_live.py [--thresh 0.5] [--vel 0.01]
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control


class StopStampFinder(Node):
    def __init__(self, dist_thresh: float, vel_thresh: float):
        super().__init__("stop_stamp_finder")
        self._dist_thresh = dist_thresh
        self._vel_thresh  = vel_thresh

        self._endpoint: tuple[float, float] | None = None  # 最新轨迹终点 (x, y)
        self._near_goal = False   # 是否已进入终点附近
        self._done = False

        self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            self._traj_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._loc_cb,
            10,
        )
        self.create_subscription(
            Control,
            "/control/command/control_cmd",
            self._cmd_cb,
            10,
        )
        self.get_logger().info(
            f"监听中... 距离阈值={dist_thresh} m，速度阈值=|v|<{vel_thresh} m/s"
        )

    def _traj_cb(self, msg: Trajectory):
        if msg.points:
            pt = msg.points[-1]
            self._endpoint = (pt.pose.position.x, pt.pose.position.y)

    def _loc_cb(self, msg: Odometry):
        if self._near_goal or self._done or self._endpoint is None:
            return
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        dist = math.hypot(px - self._endpoint[0], py - self._endpoint[1])
        if dist < self._dist_thresh:
            self._near_goal = True
            self.get_logger().info(
                f"[触发] 进入终点附近，距终点 {dist:.3f} m，开始监听 control_cmd..."
            )

    def _cmd_cb(self, msg: Control):
        if self._done or not self._near_goal:
            return
        if abs(msg.longitudinal.velocity) < self._vel_thresh:
            stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            print(f"\n[结果] 第一次 velocity≈0 的 header_stamp = {stamp:.9f} s")
            print(f"        velocity = {msg.longitudinal.velocity}")
            self._done = True
            # 通知主循环退出
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--thresh", type=float, default=0.5,
                        help="距终点距离阈值（米），默认 0.5")
    parser.add_argument("--vel",    type=float, default=0.01,
                        help="速度绝对值阈值（m/s），默认 0.01")
    args = parser.parse_args()

    rclpy.init()
    node = StopStampFinder(args.thresh, args.vel)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
