import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_rtk_orientation_full(orientation_compare_csv):
    df = pd.read_csv(orientation_compare_csv)
    
    # Use EKF time as the common timeline
    t = df['t_ekf'] - df['t_ekf'].iloc[0]
    yaw_rtk_deg = np.degrees(df['yaw_rtk'])
    yaw_ekf_deg = np.degrees(df['yaw_ekf'])
    
    plt.figure(figsize=(16, 10))
    
    # Subplot 1: Raw Yaw values
    plt.subplot(2, 1, 1)
    plt.plot(t, yaw_rtk_deg, 'r-', label='RTK Yaw', linewidth=1.5)
    plt.plot(t, yaw_ekf_deg, 'b--', label='EKF Yaw (Autoware)', linewidth=1.0)
    plt.title('Full Trip: RTK vs EKF Yaw Orientation')
    plt.ylabel('Yaw [degrees]')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Subplot 2: Difference (Error)
    plt.subplot(2, 1, 2)
    # Correct diff for wrap-around to show true jump magnitude
    diff_deg = df['diff_deg']
    plt.plot(t, diff_deg, 'g-', label='Yaw Difference (RTK - EKF)')
    plt.axhline(y=10, color='r', linestyle=':', alpha=0.5)
    plt.axhline(y=-10, color='r', linestyle=':', alpha=0.5)
    plt.title('Orientation Discrepancy')
    plt.xlabel('Time [s]')
    plt.ylabel('Difference [degrees]')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('rtk_yaw_jump_analysis.png')
    print("Full trip visualization saved to rtk_yaw_jump_analysis.png")

    # Detect specific jump points in RTK data
    # Calculate angular velocity from RTK to see spikes
    df['dt'] = df['t_ekf'].diff()
    df['dyaw'] = np.degrees((df['yaw_rtk'].diff() + np.pi) % (2 * np.pi) - np.pi)
    df['wz_rtk_deg'] = df['dyaw'] / df['dt']
    
    jumps = df[df['wz_rtk_deg'].abs() > 300] # Jump faster than 300 deg/s is likely non-physical for a car
    if not jumps.empty:
        print("\nDetected Non-Physical RTK Yaw Jumps (>300 deg/s):")
        for i, row in jumps.iterrows():
            print(f"  Time: {row['t_ekf'] - df['t_ekf'].iloc[0]:.2f}s, Jump Magnitude: {row['dyaw']:.2f} deg")

if __name__ == "__main__":
    plot_rtk_orientation_full("oscillation_analysis_logs/orientation_compare_full.csv")
