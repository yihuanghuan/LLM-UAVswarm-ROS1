cmake_minimum_required(VERSION 3.0.2)
project(uav_swarm_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  UAVStatus.msg
  ControlAdaptationLog.msg
  ControlTrackingDebug.msg
  ExecutionProfile.msg
  IAPFDebug.msg
  StartupEvent.msg
  TrajectoryMetrics.msg
  UAVExecutionCommand.msg

)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
)
