################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
# package name은 패키지 이름임
project(autonomous_driving)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ad_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)


## Add include directories
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface
    ${CMAKE_SOURCE_DIR}/../../bsw/system/bridge
)

## Declare a C++ executable
add_executable(autonomous_driving_node
    src/autonomous_driving_node.cpp
    # src/driving_way_node.cpp
)

add_executable(driving_way_node
    # src/autonomous_driving_node.cpp
    src/driving_way_node.cpp
)

add_executable(behavior_planner_node
    src/behavior_planner_node.cpp
)

## Link libraries to the executable
#------------------------ autonomous_driving_node-----------------------------#
target_link_libraries(autonomous_driving_node
    Eigen3::Eigen
)

## Add dependencies to the executable
ament_target_dependencies(autonomous_driving_node
    rclcpp
    std_msgs
    ad_msgs
    tf2
    tf2_ros
    Eigen3
    geometry_msgs
)

#------------------------ driving_way_node-----------------------------#
target_link_libraries(driving_way_node
    Eigen3::Eigen
)

ament_target_dependencies(driving_way_node
    rclcpp
    std_msgs
    ad_msgs
    Eigen3
    geometry_msgs
)

#------------------------ behavior_planner_node-----------------------------#
ament_target_dependencies(behavior_planner_node
    rclcpp
    std_msgs
    ad_msgs
)

## Install the executable
install(TARGETS autonomous_driving_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS driving_way_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS behavior_planner_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/simultaneous_inputs.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()