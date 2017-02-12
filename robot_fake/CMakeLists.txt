cmake_minimum_required(VERSION 2.8.3)
project(robot_fake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(base_controller_fake src/base_controller_fake.cpp)
add_dependencies(base_controller_fake robot_fake_generate_messages_cpp)
target_link_libraries(base_controller_fake
   ${catkin_LIBRARIES}
 )
add_executable(laserscan_fake_follow src/laserscan_fake_follow.cpp)
add_dependencies(laserscan_fake_follow robot_fake_generate_messages_cpp)
target_link_libraries(laserscan_fake_follow
   ${catkin_LIBRARIES}
 )
