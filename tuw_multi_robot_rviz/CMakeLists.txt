cmake_minimum_required(VERSION 3.8)
project(tuw_multi_robot_rviz)

# Add support for C++17, supported in RVIZ2
set(CMAKE_CXX_STANDARD 14)

##################
## Dependencies ##
##################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz_default_plugins REQUIRED)

# Find custom dependencies
find_package(tuw_multi_robot_msgs REQUIRED)


##################
## Source files ##
##################

# Set the headers to moc
set(plugin_headers_to_moc
  include/tuw_multi_robot_rviz/MultiRobotGoalSelector.hpp
  include/tuw_multi_robot_rviz/MultiRobotInfoDisplay.hpp
  include/tuw_multi_robot_rviz/MultiRobotInfoVisual.hpp
  include/tuw_multi_robot_rviz/RobotGoalsArrayDisplay.hpp
  include/tuw_multi_robot_rviz/RobotGoalsArrayVisual.hpp
  include/tuw_multi_robot_rviz/TextVisual.hpp
  include/tuw_multi_robot_rviz/VoronoiGraphDisplay.hpp
  include/tuw_multi_robot_rviz/VoronoiGraphVisual.hpp
  include/tuw_multi_robot_rviz/VoronoiSegmentDisplay.hpp
  include/tuw_multi_robot_rviz/VoronoiSegmentVisual.hpp
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  ${plugin_headers_to_moc}
  src/MultiRobotGoalSelector.cpp
  src/MultiRobotInfoDisplay.cpp  
  src/MultiRobotInfoVisual.cpp
  src/RobotGoalsArrayDisplay.cpp  
  src/RobotGoalsArrayVisual.cpp
  src/VoronoiGraphDisplay.cpp
  src/VoronoiGraphVisual.cpp
  src/VoronoiSegmentDisplay.cpp
  src/VoronoiSegmentVisual.cpp
)

set(dependencies
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  nav_msgs
  tuw_multi_robot_msgs
)


ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(FILES
#  plugin_description.xml
#  DESTINATION share
#)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY media
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
  tuw_multi_robot_msgs
)

ament_package()
