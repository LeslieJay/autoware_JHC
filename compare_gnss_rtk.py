import pandas as pd
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
import math

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def extract_yaws(input_bag):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    
    topics = ["/sensing/gnss/pose_with_covariance", "/autoware_orientation"]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    gnss_yaws = []
    rtk_yaws = []

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic == "/sensing/gnss/pose_with_covariance":
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.pose.pose.orientation
            gnss_yaws.append([t, euler_from_quaternion(q.x, q.y, q.z, q.w)])
        else:
            msg = deserialize_message(data, GnssInsOrientationStamped)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.orientation.orientation
            rtk_yaws.append([t, euler_from_quaternion(q.x, q.y, q.z, q.w)])
            
    df_gnss = pd.DataFrame(gnss_yaws, columns=['t', 'yaw_gnss'])
    df_rtk = pd.DataFrame(rtk_yaws, columns=['t', 'yaw_rtk'])
    
    df_gnss = df_gnss.sort_values('t')
    df_rtk = df_rtk.sort_values('t')
    
    df_comp = pd.merge_asof(df_gnss, df_rtk, on='t')
    df_comp['diff'] = (df_comp['yaw_gnss'] - df_comp['yaw_rtk'] + np.pi) % (2 * np.pi) - np.pi
    
    print(f"Comparison: /sensing/gnss/pose_with_covariance vs /autoware_orientation:")
    print(f"  Mean Diff: {np.degrees(df_comp['diff'].mean()):.4f} deg")
    print(f"  Max Diff:  {np.degrees(df_comp['diff'].abs().max()):.4f} deg")
    print(f"  Correlation: {df_comp['yaw_gnss'].corr(df_comp['yaw_rtk']):.4f}")

if __name__ == "__main__":
    import sys
    extract_yaws(sys.argv[1])
