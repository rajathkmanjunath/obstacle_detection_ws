cmake_minimum_required(VERSION 3.0.2)
project(camera_od)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  CameraOD.msg
  CameraODArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/yolo_object_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
