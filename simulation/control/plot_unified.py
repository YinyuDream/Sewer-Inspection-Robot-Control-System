import csv
import matplotlib.pyplot as plt
import math
from scipy.signal import butter, filtfilt
import numpy as np
from scipy.signal import savgol_filter

print(plt.style.available)

plt.style.use("seaborn-v0_8-whitegrid")


def butter_lowpass_filter(data, cutoff, fs, order=2):
    # return savgol_filter(data, window_length=51, polyorder=3, mode="interp")
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
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
    steering_delta = []
    lateral_error = []
    heading_error = []
    wheel_speed_l = []
    wheel_speed_r = []

    try:
        with open(file_path, "r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                try:
                    times.append(float(row["time"]))
                    x.append(float(row["x"]))
                    y.append(float(row["y"]))
                    yaw.append(float(row["yaw"]))
                    velocity.append(float(row["velocity"]))
                    target_velocity.append(float(row["target_velocity"]))
                    roll.append(math.degrees(float(row["roll"])))
                    steering_delta.append(math.degrees(float(row["steering_delta"])))
                    lateral_error.append(float(row["lateral_error"]))
                    heading_error.append(math.degrees(float(row["heading_error"])))
                    wheel_speed_l.append(float(row["wheel_speed_l"]))
                    wheel_speed_r.append(float(row["wheel_speed_r"]))
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"File not found: {file_path}")
        return

    fs = 1.0 / np.mean(np.diff(times))
    cutoff = 0.5
    order = 4

    max_time = 1000
    times = [t for t in times if t <= max_time]
    x = x[: len(times)]
    y = y[: len(times)]
    velocity = velocity[: len(times)]
    target_velocity = target_velocity[: len(times)]
    roll = roll[: len(times)]
    steering_delta = steering_delta[: len(times)]
    lateral_error = lateral_error[: len(times)]
    heading_error = heading_error[: len(times)]
    wheel_speed_l = wheel_speed_l[: len(times)]
    wheel_speed_r = wheel_speed_r[: len(times)]

    roll_filtered = butter_lowpass_filter(roll, cutoff, fs, order)
    steering_delta_filtered = butter_lowpass_filter(steering_delta, cutoff, fs, order)
    lateral_error_filtered = butter_lowpass_filter(lateral_error, cutoff, fs, order)
    heading_error_filtered = butter_lowpass_filter(heading_error, cutoff, fs, order)
    wheel_speed_l_filtered = butter_lowpass_filter(wheel_speed_l, cutoff, fs, order)
    wheel_speed_r_filtered = butter_lowpass_filter(wheel_speed_r, cutoff, fs, order)
    velocity_filtered = butter_lowpass_filter(velocity, cutoff, fs, order)

    base_name = file_path.replace(".csv", "_unified")

    fig = plt.figure(figsize=(8, 6))
    plt.plot(x, y, "b-", linewidth=2)
    plt.title("Robot Trajectory (X-Y)", fontsize=14, fontweight="bold")
    plt.xlabel("X [m]", fontsize=12)
    plt.ylabel("Y [m]", fontsize=12)
    plt.axis("equal")
    plt.grid(True)
    fig.tight_layout()
    fig.savefig(base_name + "_trajectory.png", dpi=300)
    plt.close(fig)
    print(f"Trajectory plot saved to {base_name}_trajectory.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, velocity_filtered, "b-", linewidth=2, label="Velocity")
    ax.plot(
        times,
        butter_lowpass_filter(target_velocity, cutoff, fs, order),
        "orange",
        linewidth=2,
        label="Target Velocity",
    )

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
                end = times_arr[i - 1]
                segments.append((start, end))
                in_seg = False
        if in_seg:
            segments.append((start, times_arr[-1]))
        for s, e in segments:
            ax.axvspan(s, e, color="gray", alpha=0.3)

    ax.set_title("Velocity Tracking", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=12)
    ax.set_ylabel("Velocity [m/s]", fontsize=12)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(base_name + "_velocity.png", dpi=300)
    plt.close(fig)
    print(f"Velocity plot saved to {base_name}_velocity.png")

    fig, ax = plt.subplots(figsize=(10, 4))

    line1 = ax.plot(times, roll_filtered, "b-", linewidth=2, label="Roll [deg]")
    line2 = ax.plot(
        times,
        heading_error_filtered,
        "orange",
        linestyle="-",
        linewidth=2,
        label="Heading Error [deg]",
    )

    ax2 = ax.twinx()
    line3 = ax2.plot(
        times,
        lateral_error_filtered,
        "purple",
        linestyle="-",
        linewidth=2,
        label="Lateral Error [m]",
    )

    ax.set_xlabel("Time [s]", fontsize=12)
    ax.set_ylabel("Roll and Heading Angle [deg]", fontsize=12, color="black")
    ax2.set_ylabel("Lateral Error [m]", fontsize=12, color="purple")

    ax.tick_params(axis="y", colors="black")
    ax2.tick_params(axis="y", labelcolor="purple")

    lines = line1 + line2 + line3
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc="upper right")

    lat_min = min(lateral_error_filtered)
    lat_max = max(lateral_error_filtered)
    lat_range = lat_max - lat_min
    ax2.set_ylim(lat_min - 0.1 * lat_range, lat_max + 0.1 * lat_range)

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
                end = times_arr[i - 1]
                segments.append((start, end))
                in_seg = False
        if in_seg:
            segments.append((start, times_arr[-1]))
        for s, e in segments:
            ax.axvspan(s, e, color="gray", alpha=0.3)

    ax.set_title("Roll, Heading Error & Lateral Error", fontsize=14, fontweight="bold")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(base_name + "_roll_heading_lateral.png", dpi=300)
    plt.close(fig)
    print(f"Roll-Heading-Lateral plot saved to {base_name}_roll_heading_lateral.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, steering_delta_filtered, "b-", linewidth=2, label="Steering [deg]")

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
                end = times_arr[i - 1]
                segments.append((start, end))
                in_seg = False
        if in_seg:
            segments.append((start, times_arr[-1]))
        for s, e in segments:
            ax.axvspan(s, e, color="gray", alpha=0.3)

    ax.set_title("Steering Angle", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=12)
    ax.set_ylabel("Steering [deg]", fontsize=12)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(base_name + "_steering.png", dpi=300)
    plt.close(fig)
    print(f"Steering plot saved to {base_name}_steering.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, wheel_speed_l_filtered, "b-", linewidth=2, label="Left Wheel")
    ax.plot(times, wheel_speed_r_filtered, "orange", linewidth=2, label="Right Wheel")

    if mask.any():
        segments = []
        in_seg = False
        for i, m in enumerate(mask):
            t = times_arr[i]
            if m and not in_seg:
                start = t
                in_seg = True
            elif not m and in_seg:
                end = times_arr[i - 1]
                segments.append((start, end))
                in_seg = False
        if in_seg:
            segments.append((start, times_arr[-1]))
        for s, e in segments:
            ax.axvspan(s, e, color="gray", alpha=0.3)

    ax.set_title("Wheel Speeds", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=12)
    ax.set_ylabel("Angular Velocity [rad/s]", fontsize=12)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(base_name + "_wheel_speeds.png", dpi=300)
    plt.close(fig)
    print(f"Wheel speeds plot saved to {base_name}_wheel_speeds.png")

    delta_left = []
    delta_right = []
    tan_delta = np.tan(np.radians(steering_delta))
    for td in tan_delta:
        dl = math.degrees(math.atan(2 * 0.6 * td / (2 * 0.6 - 0.68 * td)))
        dr = math.degrees(math.atan(2 * 0.6 * td / (2 * 0.6 + 0.68 * td)))
        delta_left.append(dl)
        delta_right.append(dr)

    delta_left_filtered = butter_lowpass_filter(delta_left, cutoff, fs, order)
    delta_right_filtered = butter_lowpass_filter(delta_right, cutoff, fs, order)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, delta_left_filtered, "b-", linewidth=2, label="Left Steering")
    ax.plot(times, delta_right_filtered, "r-", linewidth=2, label="Right Steering")

    if mask.any():
        segments = []
        in_seg = False
        for i, m in enumerate(mask):
            t = times_arr[i]
            if m and not in_seg:
                start = t
                in_seg = True
            elif not m and in_seg:
                end = times_arr[i - 1]
                segments.append((start, end))
                in_seg = False
        if in_seg:
            segments.append((start, times_arr[-1]))
        for s, e in segments:
            ax.axvspan(s, e, color="gray", alpha=0.3)

    ax.set_title("Wheel Steering Angles", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=12)
    ax.set_ylabel("Steering Angle [deg]", fontsize=12)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(base_name + "_steering_angles.png", dpi=300)
    plt.close(fig)
    print(f"Steering angles plot saved to {base_name}_steering_angles.png")


if __name__ == "__main__":
    file_path = "/home/yinyudream/robot/simulation/control/simulation_data.csv"
    plot_simulation_data(file_path)
    file_path = "/home/yinyudream/robot/simulation/control/simulation_can_data.csv"
    plot_simulation_data(file_path)
