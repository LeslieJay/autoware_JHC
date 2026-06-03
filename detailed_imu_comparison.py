import pandas as pd
import numpy as np

def detailed_imu_comparison(imu_tamagawa_csv, imu_rtk_csv):
    df1 = pd.read_csv(imu_tamagawa_csv).sort_values('header_stamp')
    df2 = pd.read_csv(imu_rtk_csv).sort_values('header_stamp')
    
    # Merge on header_stamp
    df = pd.merge_asof(df1, df2, on='header_stamp', suffixes=('_tam', '_rtk'))
    
    axes = ['ax', 'ay', 'az', 'wx', 'wy', 'wz']
    
    print(f"{'Axis':<5} | {'Mean Diff':<12} | {'Max Diff':<12} | {'Std Diff':<12} | {'Corr':<8}")
    print("-" * 60)
    
    for ax in axes:
        col_tam = f"{ax}_tam"
        col_rtk = f"{ax}_rtk"
        
        # Filter NaNs
        valid = df.dropna(subset=[col_tam, col_rtk])
        if valid.empty:
            continue
            
        diff = valid[col_tam] - valid[col_rtk]
        corr = valid[col_tam].corr(valid[col_rtk])
        
        print(f"{ax:<5} | {diff.mean():>12.6f} | {diff.abs().max():>12.6f} | {diff.std():>12.6f} | {corr:>8.4f}")

    # Check for potential gravity magnitude difference (az)
    print(f"\nGravity check (az):")
    print(f"  Tamagawa Mean az: {df['az_tam'].mean():.4f}")
    print(f"  RTK IMU Mean az:  {df['az_rtk'].mean():.4f}")

if __name__ == "__main__":
    detailed_imu_comparison("oscillation_analysis_logs/imu_raw.csv", "oscillation_analysis_logs/rtk_imu_raw.csv")

