cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME ignition_platform)
set(NODE_NAME ignition_platform)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  Eigen3
  pluginlib
  as2_core
  as2_msgs
  std_srvs
  image_transport
  ignition-transport11
  ignition-msgs8
  ros_ign_bridge
  ros_ign_gazebo
  tf2
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SOURCE_CPP_FILES 
  lib/${NODE_NAME}.cpp
  lib/ignition_bridge.cpp
)
set(HEADER_HPP_FILES
  include/${NODE_NAME}/${NODE_NAME}.hpp
  include/${NODE_NAME}/ignition_bridge.hpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${NODE_NAME}_main.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  image_transport
  ignition-transport11
  ignition-msgs8
  ros_ign_bridge
  ros_ign_gazebo
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
