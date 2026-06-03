import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_orientations(orientation_compare_csv):
    df = pd.read_csv(orientation_compare_csv)
    plt.figure(figsize=(15, 8))
    plt.plot(df['t_ekf'] - df['t_ekf'].iloc[0], np.degrees(df['yaw_rtk']), label='RTK Yaw')
    plt.plot(df['t_ekf'] - df['t_ekf'].iloc[0], np.degrees(df['yaw_ekf']), label='EKF Yaw')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [deg]')
    plt.title('RTK vs EKF Yaw Comparison')
    plt.legend()
    plt.grid(True)
    plt.savefig('orientation_comparison.png')
    print("Plot saved to orientation_comparison.png")

if __name__ == "__main__":
    plot_orientations("oscillation_analysis_logs/orientation_compare_full.csv")
