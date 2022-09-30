cmake_minimum_required(VERSION 3.0.2)
project(ed_localization)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  ed
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ed_localization_pf ed_localization_odom ed_localization_laser ed_localization_plugin ed_localization_tf_plugin
  CATKIN_DEPENDS ed geometry_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros
  DEPENDS
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_localization_pf
  src/kdtree.cpp
  src/kdtree.h
  src/particle_filter.cpp
  src/particle_filter.h
)

add_library(ed_localization_odom
  src/odom_model.cpp
  src/odom_model.h
)
target_link_libraries(ed_localization_odom ed_localization_pf)

add_library(ed_localization_laser
  src/laser_model.cpp
  src/laser_model.h
)
target_link_libraries(ed_localization_laser ed_localization_pf)

add_library(ed_localization_plugin
  src/localization_plugin.cpp
  src/localization_plugin.h
)
target_link_libraries(ed_localization_plugin ed_localization_pf ed_localization_odom ed_localization_laser ${catkin_LIBRARIES})
add_dependencies(ed_localization_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_tf_plugin
  src/localization_tf_plugin.cpp
  src/localization_tf_plugin.h
)
target_link_libraries(ed_localization_tf_plugin ${catkin_LIBRARIES})
add_dependencies(ed_localization_tf_plugin ${catkin_EXPORTED_TARGETS})
