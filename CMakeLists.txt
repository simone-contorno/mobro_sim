cmake_minimum_required(VERSION 3.5)
project(mobro_sim)

# Build
set(CMAKE_BUILD_TYPE Release)

# C++ version
set(CMAKE_CXX_STANDARD 17)

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

# Executables
add_executable(simulate src/simulator.cpp src/robot.cpp)

# Directories
target_include_directories(simulate PRIVATE include ${EIGEN3_INCLUDE_DIRS})

# Dependencies
ament_target_dependencies(simulate rclcpp sensor_msgs tf2_ros nav_msgs urdf)

# Install
install(TARGETS simulate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY urdf launch maps params
  DESTINATION share/${PROJECT_NAME})

ament_package()
