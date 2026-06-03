import pandas as pd
import numpy as np

def check_jumps(kinematic_csv):
    df = pd.read_csv(kinematic_csv)
    df['header_stamp'] = df['header_stamp'].astype(float)
    df = df.sort_values('header_stamp')
    df['dx'] = df['x'].diff()
    df['dy'] = df['y'].diff()
    df['dt'] = df['header_stamp'].diff()
    df['dist_jump'] = np.sqrt(df['dx']**2 + df['dy']**2)
    
    # Filter for moving
    df_moving = df[df['vx'].abs() > 0.1]
    
    jumps = df_moving[df_moving['dist_jump'] > 0.05]
    print(f"Total points: {len(df)}")
    print(f"Moving points: {len(df_moving)}")
    print(f"Jumps > 0.05m while moving: {len(jumps)}")
    
    if not jumps.empty:
        print("Sample jumps:")
        print(jumps[['header_stamp', 'x', 'y', 'dist_jump', 'dt']].head(20))
        
        # Calculate velocity from jumps
        jumps['v_jump'] = jumps['dist_jump'] / jumps['dt']
        print(f"Max v_jump: {jumps['v_jump'].max():.2f} m/s")

if __name__ == "__main__":
    check_jumps("oscillation_analysis_logs/kinematic_state.csv")
