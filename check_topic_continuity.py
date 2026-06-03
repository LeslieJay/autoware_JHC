import rosbag2_py
from rclpy.serialization import deserialize_message
import pandas as pd
import numpy as np

def check_topic_continuity(bag_path, topic_name):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'), 
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))
    
    stamps = []
    while reader.has_next():
        _, _, ts_ns = reader.read_next()
        stamps.append(ts_ns * 1e-9)
        
    if not stamps:
        print(f"No messages found in {topic_name}")
        return
        
    df = pd.DataFrame(stamps, columns=['ts'])
    df['dt'] = df['ts'].diff()
    
    print(f"Continuity Analysis for {topic_name}:")
    print(f"  Total messages: {len(df)}")
    print(f"  Mean interval:  {df['dt'].mean():.4f}s ({1/df['dt'].mean():.2f} Hz)")
    print(f"  Max gap:        {df['dt'].max():.4f}s")
    
    large_gaps = df[df['dt'] > 0.5] # Gaps larger than 0.5s
    if not large_gaps.empty:
        print(f"\nDetected {len(large_gaps)} gaps larger than 0.5s:")
        for i, row in large_gaps.iterrows():
            print(f"  Gap at {row['ts'] - stamps[0]:.2f}s, Duration: {row['dt']:.4f}s")
    else:
        print("\nNo gaps larger than 0.5s detected.")

if __name__ == "__main__":
    import sys
    bag = "/media/f/nvme_storage/can_ws/rosbag2_2026_06_01-14_19_11"
    check_topic_continuity(bag, "/autoware_orientation")
    print("-" * 30)
    check_topic_continuity(bag, "/sensing/gnss/pose_with_covariance")
