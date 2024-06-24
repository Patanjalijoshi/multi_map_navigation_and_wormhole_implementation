cmake_minimum_required(VERSION 2.8.3)
project(multi_map_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  roscpp
  std_msgs
  nav_msgs
)

## Find SQLite3
find_package(SQLite3 REQUIRED)

## Add action files
add_action_files(
  DIRECTORY action
  FILES
  NavigateToGoal.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    roscpp
    roslib
    std_msgs
    nav_msgs
)


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}  ## Add SQLite3 include directory
)

## Declare a C++ executable
add_executable(multi_map_server
  src/MultiMapServer.cpp
)

## Add dependencies to the executable
add_dependencies(multi_map_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries to the executable
target_link_libraries(multi_map_server
  ${catkin_LIBRARIES}
  ${SQLite3_LIBRARIES}  ## Link SQLite3 library
)

