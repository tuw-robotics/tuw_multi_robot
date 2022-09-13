cmake_minimum_required(VERSION 3.8)
project(tuw_multi_robot_rviz)

# Add support for C++17, supported in RVIZ2
set(CMAKE_CXX_STANDARD 14)

##################
## Dependencies ##
##################

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Find custom dependencies
find_package(tuw_multi_robot_msgs REQUIRED)


##################
## Source files ##
##################

# Set the headers to moc
set(plugin_headers_to_moc
  include/tuw_multi_robot_rviz/VoronoiGraphDisplay.hpp
  include/tuw_multi_robot_rviz/VoronoiGraphVisual.hpp
  include/tuw_multi_robot_rviz/VoronoiSegmentDisplay.hpp
  include/tuw_multi_robot_rviz/VoronoiSegmentVisual.hpp
)

# Moc header files
foreach(header "${plugin_headers_to_moc}")
  qt5_wrap_cpp(plugin_headers_moc_files "${header}")
endforeach()

# Set the source files
set(plugin_src_files
  src/VoronoiGraphDisplay.cpp
  src/VoronoiGraphVisual.cpp
  src/VoronoiSegmentDisplay.cpp
  src/VoronoiSegmentVisual.cpp
)

add_library(tuw_multi_robot_rviz
  ${plugin_src_files}
 #${voronoi_rviz_plugin_headers_to_moc} 
  ${plugin_headers_moc_files}
)


# Define the included directories
target_include_directories(${PROJECT_NAME} PRIVATE include/${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)


ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  tuw_multi_robot_msgs
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
)

#############
## Testing ##
#############


#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
## Mark executables and/or libraries for installation
#install(TARGETS 
#  ${PROJECT_NAME} 
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION share
)

install(
  DIRECTORY icons
  DESTINATION share
)

install(
  DIRECTORY media
  DESTINATION share
)


ament_package()