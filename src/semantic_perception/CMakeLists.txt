cmake_minimum_required(VERSION 3.0.2)
project(semantic_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_filters
  cv_bridge
  perception_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_filters cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

catkin_install_python(PROGRAMS
  src/semantic_cloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(red_light_detection_node src/red_light_detection_node.cpp)

target_link_libraries(red_light_detection_node ${catkin_LIBRARIES})