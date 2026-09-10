cmake_minimum_required(VERSION 3.8)
project(amiga_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LoRaFrame.msg
  srv/GetTreeInfo.srv
  srv/GenerateLTL.srv
  srv/InterpretAnomaly.srv
  srv/InterpretNote.srv
  srv/VerifyReplan.srv
  srv/PlanPath.srv
  srv/EvaluateCondition.srv
  srv/TakeSample.srv
  action/MoveTo.action
  action/InstallTool.action
  action/UninstallTool.action
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
