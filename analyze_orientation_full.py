import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

def normalize_yaw(yaw):
    while yaw > math.pi: yaw -= 2.0 * math.pi
    while yaw < -math.pi: yaw += 2.0 * math.pi
    return yaw

def analyze_full_bag(input_bag, output_csv):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    
    topics = ["/autoware_orientation", "/localization/kinematic_state"]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    data_rtk = []
    data_ekf = []

    print("Extracting data from bag...")
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic == "/autoware_orientation":
            msg = deserialize_message(data, GnssInsOrientationStamped)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.orientation.orientation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            data_rtk.append([t, yaw])
        elif topic == "/localization/kinematic_state":
            msg = deserialize_message(data, Odometry)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            data_ekf.append([t, yaw])

    print(f"Total RTK points: {len(data_rtk)}")
    print(f"Total EKF points: {len(data_ekf)}")

    if not data_rtk or not data_ekf:
        print("Missing topics in bag.")
        return

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ekf", "yaw_rtk", "yaw_ekf", "diff_deg"])
        
        rtk_ptr = 0
        diffs = []
        for t_e, y_e in data_ekf:
            while rtk_ptr < len(data_rtk) and data_rtk[rtk_ptr][0] < t_e - 0.02:
                rtk_ptr += 1
            if rtk_ptr < len(data_rtk) and abs(data_rtk[rtk_ptr][0] - t_e) < 0.1:
                diff = normalize_yaw(data_rtk[rtk_ptr][1] - y_e)
                diff_deg = math.degrees(diff)
                writer.writerow([t_e, data_rtk[rtk_ptr][1], y_e, diff_deg])
                diffs.append(abs(diff_deg))

        if diffs:
            print(f"\nAnalysis Results:")
            print(f"  Mean Diff: {np.mean(diffs):.4f} deg")
            print(f"  Max Diff:  {np.max(diffs):.4f} deg")
            print(f"  Std Dev:   {np.std(diffs):.4f} deg")

if __name__ == "__main__":
    analyze_full_bag(sys.argv[1], "oscillation_analysis_logs/orientation_compare_full.csv")
