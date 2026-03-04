import csv
import matplotlib.pyplot as plt
import math
from scipy.signal import butter, filtfilt
import numpy as np

def butter_lowpass_filter(data, cutoff, fs, order=2):
    """
    Apply a low-pass Butterworth filter to the data.
    
    :param data: Input data list or array
    :param cutoff: Cutoff frequency in Hz
    :param fs: Sampling frequency in Hz
    :param order: Order of the filter (default is 2)
    :return: Filtered data
    """
    if not data or len(data) < 10:
        return data
    
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def plot_simulation_data(file_path):
    times = []
    x = []
    y = []
    yaw = []
    velocity = []
    target_velocity = []
    roll = []
    roll_rate = []
    steering_delta = []
    lateral_error = []
    heading_error = []
    wheel_speed_l = []
    wheel_speed_r = []

    try:
        with open(file_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                try:
                    times.append(float(row['time']))
                    x.append(float(row['x']))
                    y.append(float(row['y']))
                    yaw.append(float(row['yaw']))
                    velocity.append(float(row['velocity']))
                    target_velocity.append(float(row['target_velocity']))
                    roll.append(math.degrees(float(row['roll'])))
                    roll_rate.append(math.degrees(float(row['roll_rate'])))
                    steering_delta.append(math.degrees(float(row['steering_delta'])))
                    lateral_error.append(float(row['lateral_error']))
                    heading_error.append(math.degrees(float(row['heading_error'])))
                    wheel_speed_l.append(float(row['wheel_speed_l']))
                    wheel_speed_r.append(float(row['wheel_speed_r']))
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"File not found: {file_path}")
        return

    # Apply Butterworth Filter
    # 2nd Order Low-Pass Filter at 2Hz cutoff (Assuming 50Hz sampling)
    print("Applying 2nd Order Butterworth Low-Pass Filter (Cutoff=2Hz)...")
    
    fs = 1.0 / np.mean(np.diff(times))  # Sampling frequency (Hz)
    cutoff = 0.5 # Cutoff frequency (Hz)
    order = 4

    velocity_filtered = butter_lowpass_filter(velocity, cutoff, fs, order)
    roll_rate_filtered = butter_lowpass_filter(roll_rate, cutoff, fs, order)
    roll_filtered = butter_lowpass_filter(roll, cutoff, fs, order)
    steering_delta_filtered = butter_lowpass_filter(steering_delta, cutoff, fs, order)
    wheel_speed_l_filtered = butter_lowpass_filter(wheel_speed_l, cutoff, fs, order)
    wheel_speed_r_filtered = butter_lowpass_filter(wheel_speed_r, cutoff, fs, order)
    
    # Plotting
    fig, axs = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Simulation Data Analysis (Filtered)', fontsize=16)

    # Trajectory (No filtering needed usually, but good to show context)
    axs[0, 0].plot(x, y, label='Path')
    axs[0, 0].set_title('Robot Trajectory (X-Y)')
    axs[0, 0].set_xlabel('X [m]')
    axs[0, 0].set_ylabel('Y [m]')
    axs[0, 0].axis('equal')
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    # Velocity
    axs[0, 1].plot(times, velocity, label='Raw Data', alpha=0.3, color='gray') # Show raw data faintly
    axs[0, 1].plot(times, velocity_filtered, label='Filtered Velocity', color='tab:blue', linewidth=2)
    axs[0, 1].plot(times, target_velocity, label='Target Velocity', linestyle='--', color='tab:orange')
    axs[0, 1].set_title('Velocity Tracking (Smoothed)')
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('Velocity [m/s]')
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    # Roll & Roll Rate
    color = 'tab:blue'
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Roll [deg]', color=color)
    axs[1, 0].plot(times, roll, color=color, label='Roll', alpha=0.3) # Roll is usually smooth enough, or filter slightly
    axs[1, 0].plot(times, roll_filtered, color='tab:green', label='Filtered Roll', linewidth=2)
    axs[1, 0].tick_params(axis='y', labelcolor=color)
    axs[1, 0].grid(True)
    
    ax2 = axs[1, 0].twinx()
    color = 'tab:red'
    ax2.set_ylabel('Roll Rate [deg/s]', color=color)
    ax2.plot(times, roll_rate, label='Raw Roll Rate', alpha=0.3, color='pink')
    ax2.plot(times, roll_rate_filtered, color=color, linestyle='--', label='Filtered Roll Rate')
    ax2.tick_params(axis='y', labelcolor=color)
    axs[1, 0].set_title('Roll Stability')

    # Steering & Errors
    axs[1, 1].plot(times, steering_delta, label='Raw Steering', alpha=0.3, color='lightblue')
    axs[1, 1].plot(times, steering_delta_filtered, label='Filtered Steering [deg]', color='tab:blue', linewidth=2)
    axs[1, 1].plot(times, heading_error, label='Heading Error [deg]', linestyle='--', color='tab:orange')
    axs[1, 1].set_title('Steering & Heading Error (Smoothed)')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Angle [deg]')
    axs[1, 1].grid(True)
    axs[1, 1].legend()

    # Lateral Error
    axs[2, 0].plot(times, lateral_error, color='purple', label='Lateral Error')
    axs[2, 0].set_title('Lateral Error (Stanley)')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_ylabel('Error [m]')
    axs[2, 0].grid(True)
    axs[2, 0].legend()

    # Wheel Speeds
    axs[2, 1].plot(times, wheel_speed_l, label='Left (Raw)', alpha=0.2, color='lightblue')
    axs[2, 1].plot(times, wheel_speed_r, label='Right (Raw)', alpha=0.2, color='moccasin')
    axs[2, 1].plot(times, wheel_speed_l_filtered, label='Left (Filtered)', color='tab:blue')
    axs[2, 1].plot(times, wheel_speed_r_filtered, label='Right (Filtered)', color='tab:orange')
    axs[2, 1].set_title('Wheel Speeds (Encoder) (Smoothed)')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel('Angular Vel [rad/s]')
    axs[2, 1].grid(True)
    axs[2, 1].legend()

    plt.tight_layout()
    # Save the plot
    plot_file = file_path.replace('.csv', '_filtered.png')
    plt.savefig(plot_file)
    print(f"Filtered plot saved to {plot_file}")
    plt.show()

if __name__ == "__main__":
    file_path = '/home/yinyudream/robot/simulation_data.csv'
    plot_simulation_data(file_path)