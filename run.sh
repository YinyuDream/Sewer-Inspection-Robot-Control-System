pkill -f "ign gazebo"
pkill -f "ros2"
pkill -f "rviz"
pkill -f "parameter_bridge"
rm -rf build install log
colcon build
source install/setup.bash
ros2 launch simple_car_sim simulation.launch.py