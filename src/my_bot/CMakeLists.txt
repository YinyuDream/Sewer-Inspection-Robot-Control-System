cmake_minimum_required(VERSION 3.8)
project(my_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(xacro REQUIRED)

# Install resource files
install(DIRECTORY launch urdf config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install teleop script
install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
