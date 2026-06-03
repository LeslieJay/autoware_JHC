import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def analyze_latency(kinematic_state_csv):
    df = pd.read_csv(kinematic_state_csv)
    df['latency'] = df['bag_ts'] - df['header_stamp']
    
    print(f"Latency (Record Time - Header Time) for /localization/kinematic_state:")
    print(f"  Mean: {df['latency'].mean():.4f} s")
    print(f"  Max:  {df['latency'].max():.4f} s")
    print(f"  Min:  {df['latency'].min():.4f} s")
    print(f"  Std:  {df['latency'].std():.4f} s")

    # Check if latency is increasing
    correlation = np.corrcoef(df['bag_ts'], df['latency'])[0, 1]
    print(f"  Correlation with time: {correlation:.4f} (High value indicates increasing delay)")

def compare_yaw_latency(orientation_compare_csv):
    df = pd.read_csv(orientation_compare_csv)
    # yaw_rtk, yaw_ekf
    # Find best delay for yaw_ekf to match yaw_rtk
    
    best_delay = 0
    min_error = float('inf')
    
    # Simple sweep
    delays = np.linspace(-2.0, 2.0, 401) # -2s to 2s
    
    for delay in delays:
        # Shift EKF yaw (effectively)
        # We need to interpolate RTK yaw at shifted EKF times
        t_shifted = df['t_ekf'] - delay
        y_rtk_interp = np.interp(t_shifted, df['t_ekf'], df['yaw_rtk'])
        
        # Calculate error (handling wrap-around)
        diff = (y_rtk_interp - df['yaw_ekf'] + np.pi) % (2 * np.pi) - np.pi
        error = np.mean(np.abs(diff))
        
        if error < min_error:
            min_error = error
            best_delay = delay
            
    print(f"\nYaw Latency Analysis:")
    print(f"  Best fit delay (RTK relative to EKF): {best_delay:.3f} s")
    print(f"  Min mean error at best delay: {np.degrees(min_error):.4f} deg")
    
    # Error without delay
    diff_no_delay = (df['yaw_rtk'] - df['yaw_ekf'] + np.pi) % (2 * np.pi) - np.pi
    error_no_delay = np.mean(np.abs(diff_no_delay))
    print(f"  Mean error without delay: {np.degrees(error_no_delay):.4f} deg")

if __name__ == "__main__":
    analyze_latency("oscillation_analysis_logs/kinematic_state.csv")
    compare_yaw_latency("oscillation_analysis_logs/orientation_compare_full.csv")
