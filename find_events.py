import pandas as pd

def find_events(kinematic_csv, steering_csv):
    df_loc = pd.read_csv(kinematic_csv)
    df_steer = pd.read_csv(steering_csv)
    
    df_loc['header_stamp'] = df_loc['header_stamp'].astype(float)
    df_steer['header_stamp'] = df_steer['header_stamp'].astype(float)
    
    # Filter for moving and turning
    moving_turning = df_loc[(df_loc['vx'].abs() > 0.5) & (df_loc['wz'].abs() > 0.05)]
    
    if moving_turning.empty:
        print("No moving and turning events found in kinematic_state.")
        # Try just moving
        moving = df_loc[df_loc['vx'].abs() > 0.5]
        if moving.empty:
            print("No moving events found at all.")
            return
        else:
            print(f"Moving events found. Sample stamps: {moving['header_stamp'].head(5).values}")
    else:
        print(f"Moving and turning events found. {len(moving_turning)} points.")
        print(moving_turning[['header_stamp', 'vx', 'wz']].head(20))
        
        # Check steering during these periods
        df_steer_moving = pd.merge_asof(moving_turning, df_steer, on='header_stamp')
        print("Steering during turning:")
        print(df_steer_moving[['header_stamp', 'vx', 'wz', 'steering_tire_angle']].head(20))

if __name__ == "__main__":
    find_events("oscillation_analysis_logs/kinematic_state.csv", "oscillation_analysis_logs/steering_status.csv")
