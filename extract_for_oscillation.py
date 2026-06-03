import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from sensor_msgs.msg import Imu

TOPICS = [
    "/localization/kinematic_state",
    "/vehicle/status/steering_status",
    "/vehicle/status/velocity_status",
    "/sensing/imu/tamagawa/imu_raw",
    "/rtk_imu/data_raw",
]

def ts_sec(nanoseconds: int) -> float:
    return nanoseconds * 1e-9

def extract(input_bag: str, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))

    f_loc = open(os.path.join(output_dir, "kinematic_state.csv"), "w", newline="")
    f_steer = open(os.path.join(output_dir, "steering_status.csv"), "w", newline="")
    f_vel = open(os.path.join(output_dir, "velocity_status.csv"), "w", newline="")
    f_imu = open(os.path.join(output_dir, "imu_raw.csv"), "w", newline="")
    f_rtk_imu = open(os.path.join(output_dir, "rtk_imu_raw.csv"), "w", newline="")

    w_loc = csv.writer(f_loc)
    w_steer = csv.writer(f_steer)
    w_vel = csv.writer(f_vel)
    w_imu = csv.writer(f_imu)
    w_rtk_imu = csv.writer(f_rtk_imu)

    w_loc.writerow(["bag_ts", "header_stamp", "x", "y", "vx", "vy", "wz"])
    w_steer.writerow(["bag_ts", "header_stamp", "steering_tire_angle"])
    w_vel.writerow(["bag_ts", "header_stamp", "longitudinal_velocity"])
    w_imu.writerow(["bag_ts", "header_stamp", "ax", "ay", "az", "wx", "wy", "wz"])
    w_rtk_imu.writerow(["bag_ts", "header_stamp", "ax", "ay", "az", "wx", "wy", "wz"])

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        bag_ts = f"{ts_sec(ts_ns):.9f}"

        if topic == "/localization/kinematic_state":
            msg = deserialize_message(data, Odometry)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            w_loc.writerow([bag_ts, f"{t:.9f}", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
        elif topic == "/vehicle/status/steering_status":
            msg = deserialize_message(data, SteeringReport)
            t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            w_steer.writerow([bag_ts, f"{t:.9f}", msg.steering_tire_angle])
        elif topic == "/vehicle/status/velocity_status":
            msg = deserialize_message(data, VelocityReport)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            w_vel.writerow([bag_ts, f"{t:.9f}", msg.longitudinal_velocity])
        elif topic == "/sensing/imu/tamagawa/imu_raw":
            msg = deserialize_message(data, Imu)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            w_imu.writerow([bag_ts, f"{t:.9f}", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        elif topic == "/rtk_imu/data_raw":
            msg = deserialize_message(data, Imu)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            w_rtk_imu.writerow([bag_ts, f"{t:.9f}", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])


    f_loc.close()
    f_steer.close()
    f_vel.close()
    f_imu.close()

if __name__ == "__main__":
    extract(sys.argv[1], sys.argv[2])
