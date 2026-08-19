cmake_minimum_required(VERSION 3.8)
project(ed_localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()

find_package(ament_cmake REQUIRED)

find_package(ed REQUIRED)
find_package(geolib2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tue_config REQUIRED)

# Non-ROS dependencies
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                        PARTICLE FILTER
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_pf SHARED
  src/kdtree.cpp
  src/kdtree.h
  src/particle_filter.cpp
  src/particle_filter.h
)
target_link_libraries(${PROJECT_NAME}_pf PUBLIC
  geolib2::geolib
  rclcpp::rclcpp
  tue_config::tue_config
)

# ------------------------------------------------------------------------------------------------
#                                            MODELS
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_odom SHARED
  src/odom_model.cpp
  src/odom_model.h
)
target_link_libraries(${PROJECT_NAME}_odom PUBLIC ${PROJECT_NAME}_pf)

add_library(${PROJECT_NAME}_laser SHARED
  src/laser_model.cpp
  src/laser_model.h
)
target_link_libraries(${PROJECT_NAME}_laser PUBLIC
  ${PROJECT_NAME}_pf
  ed::ed_core
  ${sensor_msgs_TARGETS}
)

# ------------------------------------------------------------------------------------------------
#                                            PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_plugin SHARED
  src/localization_plugin.cpp
  src/localization_plugin.h
)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC
  ${PROJECT_NAME}_laser
  ${PROJECT_NAME}_odom
  ${PROJECT_NAME}_pf
  ed::ed_core
  geolib2::geolib
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)
# Only the visualization helper includes <opencv2/*>; keep it off the link interface.
target_link_libraries(${PROJECT_NAME}_plugin PRIVATE ${OpenCV_LIBRARIES})

add_library(${PROJECT_NAME}_tf_plugin SHARED
  src/localization_tf_plugin.cpp
  src/localization_tf_plugin.h
)
target_link_libraries(${PROJECT_NAME}_tf_plugin PUBLIC
  ed::ed_core
  geolib2::geolib
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(ed plugins.xml)

# ------------------------------------------------------------------------------------------------
#                                            INSTALL
# ------------------------------------------------------------------------------------------------

install(
  TARGETS
    ${PROJECT_NAME}_laser
    ${PROJECT_NAME}_odom
    ${PROJECT_NAME}_pf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ${PROJECT_NAME}_plugin
    ${PROJECT_NAME}_tf_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------------------------
#                                             TESTS
# ------------------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(tue_lint_config REQUIRED)

  ament_clang_format(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format --clang-format-version=21)
  ament_clang_tidy(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy --clang-tidy-version=21
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/compile_commands.json)
  ament_lint_cmake(MAX_LINE_LENGTH 120)
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()

ament_package()
