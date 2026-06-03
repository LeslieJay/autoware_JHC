import pandas as pd
import numpy as np

def analyze_localization(kinematic_csv):
    df = pd.read_csv(kinematic_csv)
    df['header_stamp'] = df['header_stamp'].astype(float)
    df = df.sort_values('header_stamp')
    
    # Calculate jumps in x and y
    df['dx'] = df['x'].diff()
    df['dy'] = df['y'].diff()
    df['dist_jump'] = np.sqrt(df['dx']**2 + df['dy']**2)
    
    # Calculate yaw from vx, vy if possible, or just look at wz
    # wz is yaw rate.
    
    print(f"Max distance jump: {df['dist_jump'].max():.4f} m")
    
    # Filter for moving
    df_moving = df[df['vx'].abs() > 0.1]
    if not df_moving.empty:
        print(f"Mean dist jump when moving: {df_moving['dist_jump'].mean():.4f} m")
        print(f"Max dist jump when moving: {df_moving['dist_jump'].max():.4f} m")
        
        # Check wz (yaw rate) oscillation
        df_moving['wz_rate'] = df_moving['wz'].diff() / df_moving['header_stamp'].diff()
        df_moving['wz_sign_change'] = (np.sign(df_moving['wz_rate']).diff() != 0) & (np.sign(df_moving['wz_rate']) != 0)
        
        window_size = 20
        df_moving['wz_osc_count'] = df_moving['wz_sign_change'].rolling(window=window_size).sum()
        
        high_wz_osc = df_moving[df_moving['wz_osc_count'] > 5]
        if not high_wz_osc.empty:
            print(f"Found {len(high_wz_osc)} points with high yaw rate oscillation.")
            print(high_wz_osc[['header_stamp', 'wz', 'wz_osc_count']].head(10))
        else:
            print("No high yaw rate oscillation detected.")

if __name__ == "__main__":
    analyze_localization("oscillation_analysis_logs/kinematic_state.csv")
