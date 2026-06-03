import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
import pandas as pd

def check_raw_freezing(bag_path, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'), 
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    
    vals = []
    stamps = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, GnssInsOrientationStamped)
        # Check quaternion components to see if they are identical
        q = msg.orientation.orientation
        vals.append((q.x, q.y, q.z, q.w))
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        
    df = pd.DataFrame(vals, columns=['x', 'y', 'z', 'w'])
    df['ts'] = stamps
    
    # Check for identical consecutive quaternions
    df['is_frozen'] = (df['x'] == df['x'].shift()) & \
                      (df['y'] == df['y'].shift()) & \
                      (df['z'] == df['z'].shift()) & \
                      (df['w'] == df['w'].shift())
    
    print(f"Raw Value Freezing Analysis for {topic}:")
    print(f"  Total messages: {len(df)}")
    print(f"  Frozen messages: {df['is_frozen'].sum()} ({df['is_frozen'].sum()/len(df)*100:.2f}%)")
    
    # Blocks
    df['block'] = (df[['x','y','z','w']] != df[['x','y','z','w']].shift()).any(axis=1).cumsum()
    blocks = df.groupby('block').agg(
        start_t=('ts', 'first'),
        end_t=('ts', 'last'),
        count=('ts', 'count')
    )
    blocks['duration'] = blocks['end_t'] - blocks['start_t']
    frozen_blocks = blocks[blocks['count'] > 1]
    
    if not frozen_blocks.empty:
        print(f"\nTop 5 Longest Frozen Segments:")
        print(frozen_blocks.sort_values('duration', ascending=False)[['duration', 'count', 'start_t']].head(5).to_string(index=False))

if __name__ == "__main__":
    import sys
    check_raw_freezing("/media/f/nvme_storage/can_ws/rosbag2_2026_06_01-14_19_11", "/autoware_orientation")
