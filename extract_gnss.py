import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped

def extract_gnss(input_bag, output_file):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/sensing/gnss/pose_with_covariance"]))

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["header_stamp", "x", "y", "cov_x", "cov_y"])
        while reader.has_next():
            topic, data, ts_ns = reader.read_next()
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            writer.writerow([f"{t:.9f}", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.covariance[0], msg.pose.covariance[7]])

if __name__ == "__main__":
    extract_gnss(sys.argv[1], "oscillation_analysis_logs/gnss_pose.csv")
