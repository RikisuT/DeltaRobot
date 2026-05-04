cmake_minimum_required(VERSION 3.8)
project(delta_robot_testing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/motor_slider_tester.py
  scripts/see_motors.py
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
