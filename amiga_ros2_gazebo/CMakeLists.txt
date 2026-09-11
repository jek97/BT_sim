cmake_minimum_required(VERSION 3.8)
project(amiga_ros2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  meshes
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_canbus_twist.py
  scripts/sim_twist_control.py
  scripts/sim_gps_shim.py
  scripts/sim_imu_shim.py
  scripts/sim_camera_shim.py
  scripts/sim_joint_state_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Sets IGN_GAZEBO_RESOURCE_PATH automatically whenever install/setup.bash is
# sourced, so `ign gazebo` run directly (not through gazebo.launch.py) can
# still resolve model://house_1, model://fence, etc. without a manual export.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/ign_gazebo_resource_path.sh")

ament_package()
