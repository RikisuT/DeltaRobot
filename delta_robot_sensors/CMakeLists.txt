cmake_minimum_required(VERSION 3.8)
project(delta_robot_sensors)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Doxygen)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(deltarobot_interfaces REQUIRED)

include_directories(
  include/delta_robot_sensors
  ${deltarobot_interfaces_INCLUDE_DIRS}
)

# BN0055 Node
add_executable(BNO055_Sensor src/bno055_node.cpp src/bno055.cpp)
ament_target_dependencies(BNO055_Sensor rclcpp sensor_msgs)

# VL53L1X Node
add_executable(VL53L1X_Sensor src/vl53l1x_node.cpp src/vl53l1x.cpp)
ament_target_dependencies(VL53L1X_Sensor rclcpp sensor_msgs)

# Kalman Filter Node
add_executable(kalman src/kalman.cpp)
ament_target_dependencies(kalman rclcpp sensor_msgs deltarobot_interfaces)

install(TARGETS
  VL53L1X_Sensor
  BNO055_Sensor
  kalman
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build documentation" OFF)

if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  # set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ../README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen ${PROJECT_SOURCE_DIR}/include/ ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
