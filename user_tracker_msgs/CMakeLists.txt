cmake_minimum_required(VERSION 2.8.3)
project(user_tracker_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

add_message_files(
   FILES
   PlaneModel.msg
   UserPoseCloud.msg
   Blindage.msg
 )

generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs
 )

catkin_package(
)
