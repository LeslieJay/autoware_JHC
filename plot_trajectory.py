import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_trajectories(gnss_csv, ekf_csv, output_img):
    df_gnss = pd.read_csv(gnss_csv)
    df_ekf = pd.read_csv(ekf_csv)
    
    plt.figure(figsize=(12, 10))
    
    # Plot RTK/GNSS trajectory
    plt.plot(df_gnss['x'], df_gnss['y'], 'r.', label='RTK (Input)', markersize=2, alpha=0.5)
    
    # Plot EKF trajectory
    plt.plot(df_ekf['x'], df_ekf['y'], 'b-', label='EKF (Output)', linewidth=1)
    
    # Mark start and end
    plt.plot(df_gnss['x'].iloc[0], df_gnss['y'].iloc[0], 'go', label='Start')
    plt.plot(df_gnss['x'].iloc[-1], df_gnss['y'].iloc[-1], 'ko', label='End')
    
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trajectory Comparison: RTK vs EKF')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.savefig(output_img)
    print(f"Plot saved to {output_img}")

if __name__ == "__main__":
    plot_trajectories("oscillation_analysis_logs/gnss_pose.csv", "oscillation_analysis_logs/kinematic_state.csv", "trajectory_comparison.png")
