cmake_minimum_required(VERSION 2.8.3)
project(magni_nav)
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/corridor_centering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
