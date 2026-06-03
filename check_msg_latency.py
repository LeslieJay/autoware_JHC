import pandas as pd
import numpy as np

def check_msg_latency(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"), 
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    
    topics = ["/sensing/imu/tamagawa/imu_raw", "/localization/kinematic_state"]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    
    imu_stamps = []
    ekf_stamps = []
    
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic == "/sensing/imu/tamagawa/imu_raw":
            msg = deserialize_message(data, Imu)
            t_header = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            imu_stamps.append([ts_ns * 1e-9, t_header])
        else:
            msg = deserialize_message(data, Odometry)
            t_header = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ekf_stamps.append([ts_ns * 1e-9, t_header])
            
    df_imu = pd.DataFrame(imu_stamps, columns=['bag_ts', 'header_ts'])
    df_ekf = pd.DataFrame(ekf_stamps, columns=['bag_ts', 'header_ts'])
    
    # Processing latency: bag_ts - header_ts
    df_imu['latency'] = df_imu['bag_ts'] - df_imu['header_ts']
    df_ekf['latency'] = df_ekf['bag_ts'] - df_ekf['header_ts']
    
    print(f"IMU Latency (Bag TS - Header TS): {df_imu['latency'].mean():.4f}s")
    print(f"EKF Latency (Bag TS - Header TS): {df_ekf['latency'].mean():.4f}s")
    
    # Inter-topic latency: When an EKF message with header T is recorded, 
    # what was the latest IMU message header T?
    df_imu = df_imu.sort_values('bag_ts')
    df_ekf = df_ekf.sort_values('bag_ts')
    
    df_sync = pd.merge_asof(df_ekf, df_imu, on='bag_ts', suffixes=('_ekf', '_imu'))
    df_sync['msg_age'] = df_sync['header_ts_ekf'] - df_sync['header_ts_imu']
    
    print(f"EKF Header relative to latest IMU Header at record time: {df_sync['msg_age'].mean():.4f}s")

if __name__ == "__main__":
    import sys
    check_msg_latency(sys.argv[1])
