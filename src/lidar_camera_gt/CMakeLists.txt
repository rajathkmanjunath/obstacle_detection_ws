cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_gt)

find_package(catkin REQUIRED COMPONENTS
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

catkin_install_python(PROGRAMS
  scripts/publish_lidar_points_gt.py
  scripts/calibrate_point_cloud_threshold.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
