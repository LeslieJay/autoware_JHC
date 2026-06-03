import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
import math
import numpy as np
import matplotlib.pyplot as plt

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def process_new_bag(bag_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    
    topic = "/autoware_orientation"
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    data = []
    while reader.has_next():
        topic_name, msg_data, ts_ns = reader.read_next()
        msg = deserialize_message(msg_data, GnssInsOrientationStamped)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.orientation.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        data.append([t, yaw, q.x, q.y, q.z, q.w])

    if not data:
        print("No data found.")
        return

    df = pd.DataFrame(data, columns=['t', 'yaw', 'qx', 'qy', 'qz', 'qw'])
    df = df.sort_values('t')
    
    # Plot
    plt.figure(figsize=(15, 7))
    plt.plot(df['t'] - df['t'].iloc[0], np.degrees(df['yaw']), 'r-', label='RTK Yaw')
    plt.title('New Trip: RTK Yaw Orientation')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [deg]')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.savefig('new_rtk_yaw.png')
    print("Plot saved to new_rtk_yaw.png")
    
    # Check freezing
    df['is_frozen'] = (df['qx'] == df['qx'].shift()) & \
                      (df['qy'] == df['qy'].shift()) & \
                      (df['qz'] == df['qz'].shift()) & \
                      (df['qw'] == df['qw'].shift())
    
    frozen_count = df['is_frozen'].sum()
    print(f"\nValue Freezing Analysis:")
    print(f"  Total messages: {len(df)}")
    print(f"  Frozen messages: {frozen_count} ({frozen_count/len(df)*100:.2f}%)")
    
    # Jumps
    df['dt'] = df['t'].diff()
    df['dyaw'] = np.degrees((df['yaw'].diff() + np.pi) % (2 * np.pi) - np.pi)
    df['wz_deg'] = df['dyaw'] / df['dt']
    jumps = df[df['wz_deg'].abs() > 300]
    print(f"  Non-physical jumps detected: {len(jumps)}")

if __name__ == "__main__":
    import pandas as pd
    process_new_bag(sys.argv[1], "new_bag_analysis")

