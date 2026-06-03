import pandas as pd
import matplotlib.pyplot as plt

def plot_debug(imu_csv, vel_csv, kinematic_csv, orient_csv):
    df_imu = pd.read_csv(imu_csv)
    df_vel = pd.read_csv(vel_csv)
    df_kin = pd.read_csv(kinematic_csv)
    df_orient = pd.read_csv(orient_csv)
    
    t0 = df_kin['header_stamp'].iloc[0]
    
    fig, axs = plt.subplots(3, 1, figsize=(15, 12), sharex=True)
    
    # Plot 1: Velocity
    axs[0].plot(df_vel['header_stamp'] - t0, df_vel['longitudinal_velocity'], label='Velocity')
    axs[0].set_ylabel('m/s')
    axs[0].legend()
    axs[0].grid(True)
    
    # Plot 2: wz
    axs[1].plot(df_imu['header_stamp'] - t0, df_imu['wz'], label='IMU wz')
    axs[1].plot(df_kin['header_stamp'] - t0, df_kin['wz'], label='EKF wz', alpha=0.7)
    axs[1].set_ylabel('rad/s')
    axs[1].legend()
    axs[1].grid(True)
    
    # Plot 3: Yaw
    # Calculate RTK wz from yaw
    df_orient['t'] = df_orient['t_ekf']
    df_orient = df_orient.sort_values('t')
    df_orient['dt'] = df_orient['t'].diff()
    dyaw = (df_orient['yaw_rtk'].diff() + 3.14159) % (2*3.14159) - 3.14159
    df_orient['wz_rtk'] = dyaw / df_orient['dt']
    
    axs[2].plot(df_orient['t'] - t0, df_orient['yaw_rtk'], label='RTK Yaw')
    axs[2].plot(df_orient['t'] - t0, df_orient['yaw_ekf'], label='EKF Yaw')
    axs[2].set_ylabel('rad')
    axs[2].legend()
    axs[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('debug_plots.png')
    print("Plots saved to debug_plots.png")

if __name__ == "__main__":
    plot_debug("oscillation_analysis_logs/imu_raw.csv", 
               "oscillation_analysis_logs/velocity_status.csv", 
               "oscillation_analysis_logs/kinematic_state.csv", 
               "oscillation_analysis_logs/orientation_compare_full.csv")

