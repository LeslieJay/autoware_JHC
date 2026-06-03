import pandas as pd
import numpy as np

def check_value_freezing(csv_path, col_name):
    df = pd.read_csv(csv_path)
    
    # Calculate difference between consecutive values
    # Use a very small epsilon for floating point comparison
    df['diff_val'] = df[col_name].diff().abs()
    
    # Identify where diff is 0 (value hasn't changed)
    df['is_frozen'] = df['diff_val'] == 0
    
    print(f"Value Freezing Analysis for {col_name}:")
    
    frozen_count = df['is_frozen'].sum()
    print(f"  Total messages with same value as previous: {frozen_count} ({frozen_count/len(df)*100:.2f}%)")
    
    # Identify segments
    df['block'] = (df[col_name] != df[col_name].shift()).cumsum()
    blocks = df.groupby('block').agg(
        start_t=('t_ekf', 'first'),
        end_t=('t_ekf', 'last'),
        count=(col_name, 'count'),
        val=(col_name, 'first')
    )
    
    # Filter for blocks with more than 1 message (frozen)
    frozen_blocks = blocks[blocks['count'] > 1].copy()
    frozen_blocks['duration'] = frozen_blocks['end_t'] - frozen_blocks['start_t']
    
    if not frozen_blocks.empty:
        print(f"\nTop 10 Longest Frozen Segments:")
        print(frozen_blocks.sort_values('duration', ascending=False)[['duration', 'count', 'start_t']].head(10).to_string(index=False))
        
        print(f"\nSummary:")
        print(f"  Max frozen duration: {frozen_blocks['duration'].max():.4f}s")
        print(f"  Mean frozen duration: {frozen_blocks['duration'].mean():.4f}s")
    else:
        print("\nNo frozen values detected (all messages have unique floats).")

if __name__ == "__main__":
    check_value_freezing("oscillation_analysis_logs/orientation_compare_full.csv", "yaw_rtk")
