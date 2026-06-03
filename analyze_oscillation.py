import pandas as pd
import numpy as np

def analyze_oscillation(steering_csv, velocity_csv):
    df_steer = pd.read_csv(steering_csv)
    df_vel = pd.read_csv(velocity_csv)
    
    # Merge or align by timestamp? Steering and velocity might have different rates.
    # For a quick look, let's just use steering and interpolate velocity.
    df_steer['header_stamp'] = df_steer['header_stamp'].astype(float)
    df_vel['header_stamp'] = df_vel['header_stamp'].astype(float)
    
    df_steer = df_steer.sort_values('header_stamp')
    df_vel = df_vel.sort_values('header_stamp')
    
    df = pd.merge_asof(df_steer, df_vel, on='header_stamp')
    
    # Calculate steering rate
    df['dt'] = df['header_stamp'].diff()
    df['steer_rate'] = df['steering_tire_angle'].diff() / df['dt']
    
    # Detect sign changes in steer_rate (oscillation)
    df['rate_sign'] = np.sign(df['steer_rate'])
    df['sign_change'] = (df['rate_sign'].diff() != 0) & (df['rate_sign'] != 0)
    
    # Filter by moving state (velocity > 0.1 m/s)
    df_moving = df[df['longitudinal_velocity'] > 0.1].copy()
    
    if len(df_moving) < 10:
        print("Not enough moving data found.")
        return

    # Windowed oscillation check
    window_size = 20 # ~1 second at 20Hz
    df_moving['oscillation_count'] = df_moving['sign_change'].rolling(window=window_size).sum()
    
    # Significant oscillation if more than 5 sign changes in 1 second
    high_osc = df_moving[df_moving['oscillation_count'] > 5]
    
    if not high_osc.empty:
        print(f"Found {len(high_osc)} points with high steering oscillation.")
        print("Sample high oscillation points:")
        print(high_osc[['header_stamp', 'steering_tire_angle', 'longitudinal_velocity', 'oscillation_count']].head(20))
        
        # Check max steer rate during oscillation
        print(f"Max steer rate during moving: {df_moving['steer_rate'].abs().max():.4f} rad/s")
    else:
        print("No significant steering oscillation detected based on sign changes.")

    # Check for large steering angles at high speed
    print(f"Max speed: {df_moving['longitudinal_velocity'].max():.2f} m/s")
    
if __name__ == "__main__":
    analyze_oscillation("oscillation_analysis_logs/steering_status.csv", "oscillation_analysis_logs/velocity_status.csv")
