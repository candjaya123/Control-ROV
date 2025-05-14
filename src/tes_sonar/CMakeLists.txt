cmake_minimum_required(VERSION 3.0.2)
project(tes_sonar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/sonar.py
  src/join.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
