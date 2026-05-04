cmake_minimum_required(VERSION 3.8)
project(delta_robot_visualization)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/plot_ee_tf.py
  scripts/plotter3d.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
