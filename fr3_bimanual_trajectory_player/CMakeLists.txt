cmake_minimum_required(VERSION 3.0.2)
project(fr3_bimanual_trajectory_player)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  control_msgs
  trajectory_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bimanual_trajectory_player.py
  scripts/single_trajectory_player.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
