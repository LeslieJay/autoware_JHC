import pandas as pd
import numpy as np
import math

def analyze_turn_performance(orient_csv, imu_csv, wz_threshold=0.05):
    df_orient = pd.read_csv(orient_csv)
    df_imu = pd.read_csv(imu_csv)
    
    # Merge orientation and IMU to get wz
    df_imu = df_imu.sort_values('header_stamp')
    df_orient = df_orient.sort_values('t_ekf')
    
    df = pd.merge_asof(df_orient, df_imu[['header_stamp', 'wz']], left_on='t_ekf', right_on='header_stamp')
    
    # Filter for turns
    turns = df[df['wz'].abs() > wz_threshold].copy()
    
    if turns.empty:
        print(f"No turns found with |wz| > {wz_threshold} rad/s")
        return

    print(f"Orientation analysis during turns (|wz| > {wz_threshold} rad/s):")
    print(f"  Mean Diff: {turns['diff_deg'].abs().mean():.4f} deg")
    print(f"  Max Diff:  {turns['diff_deg'].abs().max():.4f} deg")
    print(f"  Std Dev:   {turns['diff_deg'].std():.4f} deg")
    
    # Identify specific turn segments
    turns['is_turn'] = True
    df['is_turn'] = df['wz'].abs() > wz_threshold
    
    print("\nTurn Segment Breakdown:")
    df['turn_change'] = df['is_turn'].diff()
    starts = df[df['turn_change'] == True].index
    
    segment_id = 0
    for start_idx in starts:
        if df.loc[start_idx, 'is_turn']:
            # Find end
            end_candidates = df.index[(df.index > start_idx) & (df['is_turn'] == False)]
            if len(end_candidates) == 0:
                end_idx = df.index[-1]
            else:
                end_idx = end_candidates[0]
            
            seg = df.loc[start_idx:end_idx]
            duration = seg['t_ekf'].iloc[-1] - seg['t_ekf'].iloc[0]
            if duration > 0.5: # Only segments longer than 0.5s
                segment_id += 1
                max_diff = seg['diff_deg'].abs().max()
                mean_diff = seg['diff_deg'].abs().mean()
                print(f"  Turn {segment_id}: Duration {duration:.2f}s, Mean Diff: {mean_diff:.2f} deg, Max Diff: {max_diff:.2f} deg")

if __name__ == "__main__":
    analyze_turn_performance("oscillation_analysis_logs/orientation_compare_full.csv", 
                             "oscillation_analysis_logs/imu_raw.csv")

