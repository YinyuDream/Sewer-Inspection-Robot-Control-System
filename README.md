# Four Wheel Robot Simulation (ROS 2 Humble + Gazebo Fortress)

This workspace contains a ROS 2 package that simulates a four-wheel mobile robot in Gazebo Fortress (Ignition Gazebo 6) and visualizes it in RViz2. A simple keyboard teleoperation node lets you drive the robot from a terminal.

## Workspace Layout

```
colcon_ws/
  src/
    four_wheel_robot/
```

## Build Instructions

1. Source your ROS 2 Humble environment:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Build the workspace with colcon:

   ```bash
   cd colcon_ws
   colcon build --symlink-install
   ```

3. Source the local workspace:

   ```bash
   source install/setup.bash
   ```

## Launch Simulation and Visualization

Run the main launch file to start Gazebo Fortress, spawn the robot, load controllers, and open RViz2 (set `use_rviz:=false` to skip RViz, `use_sim_time:=false` to disable simulated time, or `world:=<path>` to select a different world file):

```bash
ros2 launch four_wheel_robot sim.launch.py
```

## Teleoperate the Robot

In a separate terminal (after sourcing `install/setup.bash`), start the keyboard teleoperation node. Use `W/S` or the arrow keys to drive forward/backward, `A/D` or the arrow keys to rotate, space to stop, and `Q` to quit.

```bash
ros2 run four_wheel_robot keyboard_teleop
```

You can also launch it with custom speed limits:

```bash
ros2 launch four_wheel_robot teleop.launch.py linear_speed:=0.8 angular_speed:=1.5
```

## File Highlights

- `four_wheel_robot/urdf/four_wheel_robot.urdf.xacro`: Robot model with ros2_control integration for Ignition Gazebo.
- `four_wheel_robot/config/ros2_control.yaml`: Controller manager and diff drive controller configuration.
- `four_wheel_robot/launch/sim.launch.py`: Brings up Gazebo Fortress, spawns the robot, loads controllers, and optionally starts RViz2.
- `four_wheel_robot/four_wheel_robot/keyboard_teleop.py`: Publishes `cmd_vel` commands based on keyboard input.

## Testing Checklist

- [ ] Gazebo Fortress launches without errors.
- [ ] `joint_state_broadcaster` and `diff_drive_controller` report active state.
- [ ] Robot model appears in RViz2 and transforms update over time.
- [ ] Keyboard teleop moves the robot in Gazebo and RViz2 odometry matches the motion.
