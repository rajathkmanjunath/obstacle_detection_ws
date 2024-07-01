cmake_minimum_required(VERSION 3.0.2)
project(camera_lidar_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
   FILES
   emergencyStop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)