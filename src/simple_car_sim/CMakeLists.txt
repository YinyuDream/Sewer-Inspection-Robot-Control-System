cmake_minimum_required(VERSION 3.8) # 定义 CMake 最低版本要求
project(simple_car_sim) # 定义项目名称

# 编译选项：如果是 GCC 或 Clang 编译器，启用额外的警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =======================
# 1. 查找依赖包
# =======================
# 查找 ament_cmake 构建工具
find_package(ament_cmake REQUIRED)
# 查找 ROS 2 Python 客户端库
find_package(rclpy REQUIRED)
# 查找几何消息库 (Twist 等)
find_package(geometry_msgs REQUIRED)
# 查找机器人状态发布器
find_package(robot_state_publisher REQUIRED)
# 查找 Xacro 模型解析工具
find_package(xacro REQUIRED)

# 注意: ros_gz_sim, ros_gz_bridge, rviz2 通常是运行时依赖，但在 CMake 中查找可以验证环境

# =======================
# 2. 测试配置 (可选)
# =======================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过代码版权检查 (在学习项目中通常不需要)
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =======================
# 3. 安装规则
# =======================
# 安装非代码资源 (launch文件, URDF模型, 世界文件, RViz配置, 脚本文件夹)
# 到 install/share/simple_car_sim 目录下
install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本
# 将 Python 脚本安装到 lib/simple_car_sim 目录下，使其可以通过 `ros2 run` 执行
install(PROGRAMS
  scripts/autonomous_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# =======================
# 4. 生成包配置
# =======================
ament_package()
