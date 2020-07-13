cmake_minimum_required(VERSION 3.0.2)
project(ed_localization)

find_package(catkin REQUIRED COMPONENTS
  ed
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  LIBRARIES ed_localization_plugin ed_localization_tf_plugin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_localization_plugin
  src/kdtree.cpp
  src/kdtree.h
  src/laser_model.cpp
  src/laser_model.h
  src/localization_plugin.cpp
  src/localization_plugin.h
  src/odom_model.cpp
  src/odom_model.h
  src/particle_filter.cpp
  src/particle_filter.h
  src/transform.h
)
target_link_libraries(ed_localization_plugin ${catkin_LIBRARIES})
add_dependencies(ed_localization_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_tf_plugin
  src/localization_tf_plugin.cpp
  src/localization_tf_plugin.h
)
target_link_libraries(ed_localization_tf_plugin ${catkin_LIBRARIES})
add_dependencies(ed_localization_tf_plugin ${catkin_EXPORTED_TARGETS})
