import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix

def check_gnss_status(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'), 
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sensing/gnss/rtk/nav_sat_fix']))
    
    statuses = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, NavSatFix)
        s = msg.status.status
        statuses[s] = statuses.get(s, 0) + 1
    
    print("GNSS Status counts:")
    for s, count in statuses.items():
        # status.status: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX (usually RTK is 2)
        print(f"  Status {s}: {count}")

if __name__ == "__main__":
    import sys
    check_gnss_status(sys.argv[1])
