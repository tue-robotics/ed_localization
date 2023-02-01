cmake_minimum_required(VERSION 3.0.2)
project(ed_localization)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ed
  geolib2
  geometry_msgs
  rgbd
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tue_config
  tue_filesystem
  tue_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ed_localization_pf ed_localization_odom ed_localization_laser ed_localization_plugin_base ed_localization_plugin ed_localization_tf_plugin
  CATKIN_DEPENDS ed geolib2 geometry_msgs roscpp tf2_ros tue_config
  DEPENDS Boost
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(ed_localization_pf
  include/ed_localization/kdtree.h
  include/ed_localization/particle_filter.h
  src/kdtree.cpp
  src/particle_filter.cpp
)
target_compile_definitions(ed_localization_pf PRIVATE BOOST_FILESYSTEM_NO_DEPRECATED)
target_link_libraries(ed_localization_pf ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(ed_localization_odom
  include/ed_localization/odom_model.h
  src/odom_model.cpp
)
target_link_libraries(ed_localization_odom ed_localization_pf ${catkin_LIBRARIES})

add_library(ed_localization_laser
  src/laser_model.cpp
  src/laser_model.h
)
target_link_libraries(ed_localization_laser ed_localization_pf ${catkin_LIBRARIES})

add_library(ed_localization_plugin_base
  include/ed_localization/localization_plugin_base.h
  src/localization_plugin_base.cpp
)
target_link_libraries(ed_localization_plugin_base ed_localization_pf ed_localization_odom ${catkin_LIBRARIES})
add_dependencies(ed_localization_plugin_base ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_plugin
  src/localization_plugin.cpp
  src/localization_plugin.h
)
target_link_libraries(ed_localization_plugin ed_localization_plugin_base ed_localization_laser ${catkin_LIBRARIES})
add_dependencies(ed_localization_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_laser_test_plugin
  src/localization_laser_test_plugin.cpp
  src/localization_laser_test_plugin.h
)
target_link_libraries(ed_localization_laser_test_plugin ed_localization_plugin_base ed_localization_laser ${catkin_LIBRARIES})
add_dependencies(ed_localization_laser_test_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_rgbd
  src/rgbd_model.cpp
  src/rgbd_model.h
)
target_link_libraries(ed_localization_rgbd ed_localization_pf ${catkin_LIBRARIES})

add_library(ed_localization_rgbd_plugin
  src/localization_rgbd_plugin.cpp
  src/localization_rgbd_plugin.h
)
target_link_libraries(ed_localization_rgbd_plugin ed_localization_plugin_base ed_localization_rgbd ${catkin_LIBRARIES})
add_dependencies(ed_localization_rgbd_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_rgbd_test_plugin
  src/localization_rgbd_test_plugin.cpp
  src/localization_rgbd_test_plugin.h
)
target_link_libraries(ed_localization_rgbd_test_plugin ed_localization_plugin_base ed_localization_rgbd ${catkin_LIBRARIES})
add_dependencies(ed_localization_rgbd_test_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_localization_tf_plugin
  src/localization_tf_plugin.cpp
  src/localization_tf_plugin.h
)
target_link_libraries(ed_localization_tf_plugin ${catkin_LIBRARIES})
add_dependencies(ed_localization_tf_plugin ${catkin_EXPORTED_TARGETS})

add_executable(particles_analyzer tools/particles_analyzer.cpp)
target_link_libraries(particles_analyzer ed_localization_pf ${catkin_LIBRARIES})
