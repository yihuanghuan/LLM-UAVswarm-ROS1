cmake_minimum_required(VERSION 3.0.2)
project(ladrc_controller)

# C++17 for std::make_unique and structured bindings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  uav_swarm_interfaces
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    mavros_msgs
    uav_swarm_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ---- LADRC 核心算法库 (不变) ----
add_library(ladrc_lib
  src/ladrc_core.cpp
  src/leso.cpp
  src/lsef.cpp
)

target_link_libraries(ladrc_lib
  ${catkin_LIBRARIES}
)

# ---- 主控制节点 ----
add_executable(ladrc_position_controller_node
  src/ladrc_position_controller_node.cpp
)

target_link_libraries(ladrc_position_controller_node
  ladrc_lib
  ${catkin_LIBRARIES}
)

# ---- 安装 ----
install(TARGETS ladrc_lib ladrc_position_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
