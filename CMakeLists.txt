cmake_minimum_required(VERSION 3.0.2)
project(gopro_camera_ros1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gopro_camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
