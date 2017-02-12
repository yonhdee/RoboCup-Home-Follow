cmake_minimum_required(VERSION 2.8.3)
project(user_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  user_tracker_msgs
)

catkin_package(
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)
 add_executable(user_tracker_laser src/user_tracker_laser.cpp)
  add_dependencies(user_tracker_laser user_tracker_generate_messages_cpp user_tracker_msgs_UserPoseCloud_h user_tracker_msgs_Blindage_h)
 target_link_libraries(user_tracker_laser
   ${catkin_LIBRARIES} 
 )

 add_executable(user_footprint_follow src/user_footprint_follow.cpp)
 add_dependencies(user_footprint_follow user_tracker_generate_messages_cpp user_tracker_msgs_UserPoseCloud_h user_tracker_msgs_Blindage_h)
 target_link_libraries(user_footprint_follow
   ${catkin_LIBRARIES} 
 )
 
  add_executable(user_trackNEO src/user_trackNEO.cpp)
  add_dependencies(user_trackNEO user_tracker_generate_messages_cpp user_tracker_msgs_UserPoseCloud_h user_tracker_msgs_Blindage_h)
 target_link_libraries(user_trackNEO
   ${catkin_LIBRARIES} 
 )

add_executable(show_user_footprint src/show_footprint.cpp)
add_dependencies(show_user_footprint user_tracker_generate_messages_cpp user_tracker_msgs_UserPoseCloud_h user_tracker_msgs_Blindage_h)
 target_link_libraries(show_user_footprint
   ${catkin_LIBRARIES} 
 )
 
 add_executable(show_user_trace src/show_trace.cpp)
add_dependencies(show_user_trace user_tracker_generate_messages_cppuser_tracker_msgs_UserPoseCloud_h user_tracker_msgs_Blindage_h)
 target_link_libraries(show_user_trace
   ${catkin_LIBRARIES} 
 )

