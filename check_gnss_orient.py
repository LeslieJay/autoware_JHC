import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import sys

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def check_gnss_orientation(input_bag):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_opts, conv_opts)
    
    topic = "/sensing/gnss/pose_with_covariance"
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    print(f"Checking orientation in {topic}...")
    count = 0
    zero_orient_count = 0
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # Check if quaternion is identity (often used when orientation is invalid)
        if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 1:
            zero_orient_count += 1
        
        if count < 5:
            print(f"Yaw: {math.degrees(yaw):.2f} deg, Q: [{q.x}, {q.y}, {q.z}, {q.w}]")
        count += 1

    print(f"Total messages: {count}")
    print(f"Identity orientation (Q=[0,0,0,1]): {zero_orient_count}")

if __name__ == "__main__":
    check_gnss_orientation(sys.argv[1])
