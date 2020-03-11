cmake_minimum_required(VERSION 2.8.3)
project(fmi_adapter_examples)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fmi_adapter_examples
#  CATKIN_DEPENDS fmi_adapter roscpp
#  DEPENDS system_lib
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
)
