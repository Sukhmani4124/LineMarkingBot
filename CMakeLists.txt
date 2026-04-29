cmake_minimum_required(VERSION 3.8)
project(robot_ros)

find_package(ament_cmake REQUIRED)

# Install the PID controller Python script as a runnable executable node
install(
  PROGRAMS
    scripts/pid_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_velocity_controller
)

install(
  DIRECTORY launch urdf meshes models world config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()