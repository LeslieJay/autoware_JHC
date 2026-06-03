import pandas as pd
import numpy as np
import math

def analyze_wz_sources(imu_csv, steer_csv, vel_csv, orientation_csv, wheelbase=1.01):
    df_imu = pd.read_csv(imu_csv)
    df_steer = pd.read_csv(steer_csv)
    df_vel = pd.read_csv(vel_csv)
    df_orient = pd.read_csv(orientation_csv)
    
    # Pre-process RTK Yaw rate (using t_ekf as the time base from that file)
    df_orient['t'] = df_orient['t_ekf']
    df_orient = df_orient.sort_values('t')
    df_orient['dt'] = df_orient['t'].diff()
    dyaw = df_orient['yaw_rtk'].diff()
    dyaw = dyaw.map(lambda x: (x + math.pi) % (2 * math.pi) - math.pi)
    df_orient['wz_rtk'] = dyaw / df_orient['dt']
    
    # Merge Chassis
    df_steer = df_steer.sort_values('header_stamp')
    df_vel = df_vel.sort_values('header_stamp')
    df_chassis = pd.merge_asof(df_steer, df_vel, on='header_stamp')
    df_chassis['steer_rad'] = np.radians(df_chassis['steering_tire_angle'])
    df_chassis['wz_chassis'] = df_chassis['longitudinal_velocity'] * np.tan(df_chassis['steer_rad']) / wheelbase
    
    # IMU wz
    df_imu = df_imu.sort_values('header_stamp')
    
    # Synchronize
    df_comp = pd.merge_asof(df_chassis, df_imu, on='header_stamp')
    df_comp = pd.merge_asof(df_comp, df_orient[['t', 'wz_rtk']], left_on='header_stamp', right_on='t')
    
    moving = df_comp[df_comp['longitudinal_velocity'].abs() > 0.5].copy()
    
    if moving.empty:
        print("No moving segments found for comparison.")
        return

    # Filter out NaNs
    moving = moving.dropna(subset=['wz', 'wz_rtk', 'wz_chassis'])

    print(f"Comparison of Angular Velocity (wz) [rad/s] during moving segments:")
    print(f"{'Source':<15} | {'Mean':<10} | {'Max':<10} | {'Std':<10}")
    print("-" * 55)
    for col in ['wz', 'wz_rtk', 'wz_chassis']:
        name = "IMU" if col == 'wz' else ("RTK" if col == 'wz_rtk' else "Chassis")
        print(f"{name:<15} | {moving[col].mean():.4f} | {moving[col].max():.4f} | {moving[col].std():.4f}")

    # Specific check for correlation
    corr_imu_chassis = moving['wz'].corr(moving['wz_chassis'])
    corr_rtk_chassis = moving['wz_rtk'].corr(moving['wz_chassis'])
    
    print(f"\nCorrelation with Chassis:")
    print(f"  IMU vs Chassis: {corr_imu_chassis:.4f}")
    print(f"  RTK vs Chassis: {corr_rtk_chassis:.4f}")

if __name__ == "__main__":
    analyze_wz_sources("oscillation_analysis_logs/imu_raw.csv", 
                       "oscillation_analysis_logs/steering_status.csv", 
                       "oscillation_analysis_logs/velocity_status.csv", 
                       "oscillation_analysis_logs/orientation_compare_full.csv")
