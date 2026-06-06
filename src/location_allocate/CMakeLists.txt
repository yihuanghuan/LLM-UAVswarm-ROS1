cmake_minimum_required(VERSION 3.0.2)
project(location_allocate)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  uav_swarm_interfaces
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    uav_swarm_interfaces
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/location_allocate_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
