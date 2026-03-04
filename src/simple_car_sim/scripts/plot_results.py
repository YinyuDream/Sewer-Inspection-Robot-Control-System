import csv
import matplotlib.pyplot as plt
import math

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

    # Plotting
    fig, axs = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Simulation Data Analysis')

    # Trajectory
    axs[0, 0].plot(x, y, label='Path')
    axs[0, 0].set_title('Robot Trajectory (X-Y)')
    axs[0, 0].set_xlabel('X [m]')
    axs[0, 0].set_ylabel('Y [m]')
    axs[0, 0].axis('equal')
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    # Velocity
    axs[0, 1].plot(times, velocity, label='Actual Velocity')
    axs[0, 1].plot(times, target_velocity, label='Target Velocity', linestyle='--')
    axs[0, 1].set_title('Velocity Tracking')
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('Velocity [m/s]')
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    # Roll & Roll Rate
    color = 'tab:blue'
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Roll [deg]', color=color)
    axs[1, 0].plot(times, roll, color=color, label='Roll')
    axs[1, 0].tick_params(axis='y', labelcolor=color)
    axs[1, 0].grid(True)
    
    ax2 = axs[1, 0].twinx()  # instantiate a second axes that shares the same x-axis
    color = 'tab:red'
    ax2.set_ylabel('Roll Rate [deg/s]', color=color)  # we already handled the x-label with ax1
    ax2.plot(times, roll_rate, color=color, linestyle='--', label='Roll Rate')
    ax2.tick_params(axis='y', labelcolor=color)
    axs[1, 0].set_title('Roll Stability')

    # Steering & Errors
    axs[1, 1].plot(times, steering_delta, label='Steering Angle [deg]')
    axs[1, 1].plot(times, heading_error, label='Heading Error [deg]', linestyle='--')
    axs[1, 1].set_title('Steering & Heading Error')
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
    axs[2, 1].plot(times, wheel_speed_l, label='Left Wheel')
    axs[2, 1].plot(times, wheel_speed_r, label='Right Wheel')
    axs[2, 1].set_title('Wheel Speeds (Encoder)')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel('Angular Vel [rad/s]')
    axs[2, 1].grid(True)
    axs[2, 1].legend()

    plt.tight_layout()
    # Save the plot
    plot_file = file_path.replace('.csv', '.png')
    plt.savefig(plot_file)
    print(f"Plot saved to {plot_file}")
    plt.show()

if __name__ == "__main__":
    file_path = '/home/yinyudream/robot/simulation_data.csv'
    plot_simulation_data(file_path)
