cmake_minimum_required(VERSION 2.8.3)
project(headless_move_base)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(headless_move_base src/headless_move_base.cpp)
 add_dependencies(headless_move_base headless_move_base_generate_messages_cpp)
 target_link_libraries(headless_move_base
   ${catkin_LIBRARIES}
 )
