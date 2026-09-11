cmake_minimum_required(VERSION 3.0.2)
project(location_allocate)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
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
  scripts/candidate_dispatch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY prompts config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ../schemas/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/schemas)
