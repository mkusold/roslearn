cmake_minimum_required(VERSION 3.0.2)
project(simulation)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY robot_description
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)