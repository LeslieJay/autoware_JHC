import pandas as pd
import numpy as np

def check_ekf_wz(loc_csv, imu_csv):
    df_loc = pd.read_csv(loc_csv)
    df_imu = pd.read_csv(imu_csv)
    
    df_loc = df_loc.sort_values('header_stamp')
    df_imu = df_imu.sort_values('header_stamp')
    
    df_comp = pd.merge_asof(df_loc, df_imu, on='header_stamp')
    
    # wz from loc is EKF's estimated angular velocity
    # wz from imu is raw IMU data
    
    print(f"Comparison: EKF Estimated wz vs Raw IMU wz:")
    print(f"  EKF Mean wz: {df_comp['wz_x'].mean():.4f}") # wz_x is from kinematic_state.csv (it has x,y,vx,vy,wz columns, but pandas might have named them differently if not careful)
    # Re-check columns
    # w_loc.writerow(["bag_ts", "header_stamp", "x", "y", "vx", "vy", "wz"])
    
    print(f"  EKF Mean wz: {df_comp['wz_x'].mean():.4f}")
    print(f"  IMU Mean wz: {df_comp['wz_y'].mean():.4f}") # wz_y is from imu_raw.csv
    print(f"  Correlation: {df_comp['wz_x'].corr(df_comp['wz_y']):.4f}")

if __name__ == "__main__":
    import pandas as pd
    df_loc = pd.read_csv("oscillation_analysis_logs/kinematic_state.csv")
    print("Loc columns:", df_loc.columns.tolist())
    df_imu = pd.read_csv("oscillation_analysis_logs/imu_raw.csv")
    print("IMU columns:", df_imu.columns.tolist())
    
    df_loc = df_loc.sort_values('header_stamp')
    df_imu = df_imu.sort_values('header_stamp')
    df_comp = pd.merge_asof(df_loc, df_imu, on='header_stamp')
    
    print(f"Correlation: {df_comp['wz_x'].corr(df_comp['wz_y']):.4f}")
