import csv
import matplotlib.pyplot as plt
import math
from scipy.signal import butter, filtfilt
import numpy as np

print(plt.style.available)  # 打印可用的样式列表
plt.style.use('seaborn-whitegrid')

def butter_lowpass_filter(data, cutoff, fs, order=2):
    """
    Apply a low-pass Butterworth filter to the data.
    
    :param data: Input data list or array
    :param cutoff: Cutoff frequency in Hz
    :param fs: Sampling frequency in Hz
    :param order: Order of the filter (default is 2)
    :return: Filtered data
    """
    # if not data or len(data) < 10:
    #    return data
    
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

    omega = target_velocity * np.tan(np.radians(steering_delta)) / 0.6
    v_left = (velocity - omega * 0.68 / 2) / 0.16
    v_right = (velocity + omega * 0.68 / 2) / 0.16

    velocity_filtered = butter_lowpass_filter(velocity, cutoff, fs, order)
    roll_rate_filtered = butter_lowpass_filter(roll_rate, cutoff, fs, order)
    roll_filtered = butter_lowpass_filter(roll, cutoff, fs, order)
    steering_delta_filtered = butter_lowpass_filter(steering_delta, cutoff, fs, order)
    wheel_speed_l_filtered = butter_lowpass_filter(wheel_speed_l, cutoff, fs, order)
    wheel_speed_r_filtered = butter_lowpass_filter(wheel_speed_r, cutoff, fs, order)
    
    # Plotting: create six separate figures and save each in the same location

    # 1) Trajectory (X-Y)
    fig = plt.figure(figsize=(8, 6))
    plt.plot(x, y, label='Path')
    plt.title('Robot Trajectory (X-Y)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    traj_file = file_path.replace('.csv', '_trajectory_filtered.png')
    fig.tight_layout()
    fig.savefig(traj_file)
    plt.close(fig)
    print(f"Trajectory plot saved to {traj_file}")

    # 2) Velocity
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, velocity, alpha=0.4, color='gray')
    ax.plot(times, velocity_filtered, label='Velocity', color='tab:blue', linewidth=2)
    ax.plot(times, butter_lowpass_filter(target_velocity, cutoff, fs, order), label='Target Velocity', linewidth=2, color='tab:orange')
    ax.plot(times, target_velocity, color='gray', alpha=0.4)

    # Shade periods where abs(x)>8 and abs(y)>8
    times_arr = np.array(times)
    x_arr = np.array(x)
    y_arr = np.array(y)
    mask = (np.abs(x_arr) > 8) & (np.abs(y_arr) > 8)
    if mask.any():
        segments = []
        in_seg = False
        for i, m in enumerate(mask):
            t = times_arr[i]
            if m and not in_seg:
                start = t
                in_seg = True
            elif not m and in_seg:
                end = times_arr[i-1]
                segments.append((start, end))
                in_seg = False
        if in_seg:
            segments.append((start, times_arr[-1]))
        for (s, e) in segments:
            ax.axvspan(s, e, color='gray', alpha=0.3)

    ax.set_title('Velocity Tracking (Smoothed)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.grid(True)
    ax.legend()
    vel_file = file_path.replace('.csv', '_velocity_filtered.png')
    fig.tight_layout()
    fig.savefig(vel_file)
    plt.close(fig)
    print(f"Velocity plot saved to {vel_file}")

    # 3) Roll & Roll Rate (dual y-axis)
    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Roll [deg]', color='tab:blue')
    ax1.plot(times, roll, color='tab:blue', label='Roll', alpha=0.3)
    ax1.plot(times, roll_filtered, color='tab:green', label='Filtered Roll', linewidth=2)
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.set_ylabel('Roll Rate [deg/s]', color='tab:red')
    ax2.plot(times, roll_rate, label='Raw Roll Rate', alpha=0.3, color='pink')
    ax2.plot(times, roll_rate_filtered, color='tab:red', linestyle='--', label='Filtered Roll Rate')
    ax2.tick_params(axis='y', labelcolor='tab:red')

    fig.suptitle('Roll Stability')
    roll_file = file_path.replace('.csv', '_roll_filtered.png')
    fig.tight_layout()
    fig.savefig(roll_file)
    plt.close(fig)
    print(f"Roll plot saved to {roll_file}")

    # 4) Steering & Heading Error
    fig = plt.figure(figsize=(10, 4))
    plt.plot(times, steering_delta, label='Raw Steering', alpha=0.3, color='lightblue')
    plt.plot(times, steering_delta_filtered, label='Filtered Steering [deg]', color='tab:blue', linewidth=2)
    plt.plot(times, heading_error, label='Heading Error [deg]', linestyle='--', color='tab:orange')
    plt.title('Steering & Heading Error (Smoothed)')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.grid(True)
    plt.legend()
    steer_file = file_path.replace('.csv', '_steering_filtered.png')
    fig.tight_layout()
    fig.savefig(steer_file)
    plt.close(fig)
    print(f"Steering plot saved to {steer_file}")

    # 5) Lateral Error
    fig = plt.figure(figsize=(10, 4))
    plt.plot(times, lateral_error, label='Lateral Error (Raw)', alpha=0.3, color='lightcoral')
    plt.plot(times, butter_lowpass_filter(lateral_error, cutoff, fs, order), color='purple', label='Lateral Error')
    plt.title('Lateral Error (Stanley)')
    plt.xlabel('Time [s]')
    plt.ylabel('Error [m]')
    plt.grid(True)
    plt.legend()
    lateral_file = file_path.replace('.csv', '_lateral_error_filtered.png')
    fig.tight_layout()
    fig.savefig(lateral_file)
    plt.close(fig)
    print(f"Lateral error plot saved to {lateral_file}")

    # 6) Wheel Speeds
    fig = plt.figure(figsize=(10, 4))
    plt.plot(times, wheel_speed_l, label='Left (Raw)', alpha=0.2, color='lightblue')
    plt.plot(times, wheel_speed_r, label='Right (Raw)', alpha=0.2, color='moccasin')
    plt.plot(times, wheel_speed_l_filtered, label='Left (Filtered)', color='tab:blue')
    plt.plot(times, wheel_speed_r_filtered, label='Right (Filtered)', color='tab:orange')
    print(len(v_left))
    plt.plot(times, butter_lowpass_filter(v_left, cutoff, fs, order), label='Left (Kinematic)', linestyle='--', color='cyan')
    plt.plot(times, butter_lowpass_filter(v_right, cutoff, fs, order), label='Right (Kinematic)', linestyle='--', color='coral')
    plt.title('Wheel Speeds (Encoder) (Smoothed)')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Vel [rad/s]')
    plt.grid(True)
    plt.legend()
    wheel_file = file_path.replace('.csv', '_wheel_speeds_filtered.png')
    fig.tight_layout()
    fig.savefig(wheel_file)
    plt.close(fig)
    print(f"Wheel speeds plot saved to {wheel_file}")

    tan_delta = np.tan(np.radians(steering_delta))
    delta_left = np.degrees(np.arctan(2 * 0.6 * tan_delta / (2 * 0.6 - 0.68 * tan_delta)))  
    delta_right = np.degrees(np.arctan(2 * 0.6 * tan_delta / (2 * 0.6 + 0.68 * tan_delta)))
    fig = plt.figure(figsize=(10, 4))
    plt.plot(times, butter_lowpass_filter(delta_left, cutoff, fs, order), label='Left Steering', color='blue')
    plt.plot(times, butter_lowpass_filter(delta_right, cutoff, fs, order), label='Right Steering', color='red')
    plt.title('Wheel Steering Angles (Smoothed)')
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.legend()
    plt.savefig(file_path.replace('.csv', '_steering_angles_filtered.png'))
    plt.close(fig)
    print(f"Steering angles plot saved to {file_path.replace('.csv', '_steering_angles_filtered.png')}")

if __name__ == "__main__":
    file_path = '/home/yinyudream/robot/simulation/control/simulation_data.csv'
    plot_simulation_data(file_path)