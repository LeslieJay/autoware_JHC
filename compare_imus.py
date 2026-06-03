import pandas as pd
import numpy as np

def compare_imus(imu1_csv, imu2_csv):
    df1 = pd.read_csv(imu1_csv)
    df2 = pd.read_csv(imu2_csv)
    
    # Synchronize
    df1 = df1.sort_values('header_stamp')
    df2 = df2.sort_values('header_stamp')
    
    df_comp = pd.merge_asof(df1, df2, on='header_stamp', suffixes=('_tamagawa', '_rtk'))
    
    df_comp['wz_diff'] = df_comp['wz_tamagawa'] - df_comp['wz_rtk']
    
    print(f"Comparison of Angular Velocity Z (wz) [rad/s]:")
    print(f"  Tamagawa Mean: {df_comp['wz_tamagawa'].mean():.4f}")
    print(f"  RTK IMU Mean:  {df_comp['wz_rtk'].mean():.4f}")
    print(f"  Mean Diff:     {df_comp['wz_diff'].mean():.4f}")
    print(f"  Max Diff:      {df_comp['wz_diff'].abs().max():.4f}")
    print(f"  Correlation:   {df_comp['wz_tamagawa'].corr(df_comp['wz_rtk']):.4f}")

if __name__ == "__main__":
    compare_imus("oscillation_analysis_logs/imu_raw.csv", "oscillation_analysis_logs/rtk_imu_raw.csv")
