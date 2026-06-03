import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix

def extract_fix_status(input_bag, output_csv):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/sensing/gnss/rtk/nav_sat_fix"]))

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "status"])
        while reader.has_next():
            topic, data, ts_ns = reader.read_next()
            msg = deserialize_message(data, NavSatFix)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            writer.writerow([t, msg.status.status])

if __name__ == "__main__":
    extract_fix_status(sys.argv[1], "oscillation_analysis_logs/fix_status.csv")
