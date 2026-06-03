import pandas as pd
import numpy as np

def find_drift_events(orientation_compare_csv):
    df = pd.read_csv(orientation_compare_csv)
    df['diff_abs'] = df['diff_deg'].abs()
    
    # Find segments where diff > 10 deg
    df['drifting'] = df['diff_abs'] > 10
    
    print(f"Drift segments (> 10 deg):")
    
    start_time = None
    for i, row in df.iterrows():
        if row['drifting'] and start_time is None:
            start_time = row['t_ekf']
        elif not row['drifting'] and start_time is not None:
            duration = row['t_ekf'] - start_time
            print(f"  Start: {start_time:.2f}, Duration: {duration:.2f}s, Max Diff in segment: {df[(df['t_ekf'] >= start_time) & (df['t_ekf'] <= row['t_ekf'])]['diff_deg'].max():.2f}")
            start_time = None

if __name__ == "__main__":
    find_drift_events("oscillation_analysis_logs/orientation_compare_full.csv")
