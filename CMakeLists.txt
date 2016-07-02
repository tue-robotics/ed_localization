cmake_minimum_required(VERSION 2.8.3)
project(ed_localization)

find_package(catkin REQUIRED COMPONENTS
  ed
  geometry_msgs
  sensor_msgs
  tf
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES bla
  # DEPENDS system_lib
  CATKIN_DEPENDS ed geometry_msgs sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_localization_plugin
  src/laser_model.cpp
  src/localization_plugin.cpp
  src/odom_model.cpp
  src/particle_filter.cpp
)
# target_link_libraries(library_name ${catkin_LIBRARIES})
add_dependencies(ed_localization_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_tf_plugin
  src/localization_tf_plugin.cpp
  src/localization_tf_plugin.h
)
# target_link_libraries(library_name ${catkin_LIBRARIES})
add_dependencies(ed_localization_tf_plugin ${catkin_EXPORTED_TARGETS})
