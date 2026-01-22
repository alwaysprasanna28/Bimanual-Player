cmake_minimum_required(VERSION 3.0.2)
project(fr3_trajectory_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  controller_manager_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/single_arm_recorder.py
  scripts/bimanual_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
