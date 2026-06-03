import pandas as pd
import numpy as np

def analyze_gnss(gnss_csv, kinematic_csv):
    df_gnss = pd.read_csv(gnss_csv)
    df_kin = pd.read_csv(kinematic_csv)
    
    df_gnss['header_stamp'] = df_gnss['header_stamp'].astype(float)
    df_kin['header_stamp'] = df_kin['header_stamp'].astype(float)
    
    # Check GNSS jumps
    df_gnss['dx'] = df_gnss['x'].diff()
    df_gnss['dy'] = df_gnss['y'].diff()
    df_gnss['dt'] = df_gnss['header_stamp'].diff()
    df_gnss['dist_jump'] = np.sqrt(df_gnss['dx']**2 + df_gnss['dy']**2)
    
    print(f"GNSS Max jump: {df_gnss['dist_jump'].max():.4f} m")
    print(f"GNSS Mean jump: {df_gnss['dist_jump'].mean():.4f} m")
    
    # Check covariance
    print(f"GNSS Mean covariance (x): {df_gnss['cov_x'].mean():.6f}")
    
    # Comparison between GNSS and Kinematic State (EKF output)
    df_gnss = df_gnss.sort_values('header_stamp')
    df_kin = df_kin.sort_values('header_stamp')
    
    merged = pd.merge_asof(df_kin, df_gnss, on='header_stamp', suffixes=('_kin', '_gnss'))
    merged['diff_dist'] = np.sqrt((merged['x_kin'] - merged['x_gnss'])**2 + (merged['y_kin'] - merged['y_gnss'])**2)
    
    print(f"Max diff between EKF and GNSS: {merged['diff_dist'].max():.4f} m")
    print(f"Mean diff between EKF and GNSS: {merged['diff_dist'].mean():.4f} m")

if __name__ == "__main__":
    analyze_gnss("oscillation_analysis_logs/gnss_pose.csv", "oscillation_analysis_logs/kinematic_state.csv")
