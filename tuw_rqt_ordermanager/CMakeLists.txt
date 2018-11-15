cmake_minimum_required(VERSION 2.8.3)
project(tuw_rqt_ordermanager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
  nav_msgs
  geometry_msgs
  tuw_multi_robot_msgs
  tf
)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

roslint_cpp()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${tuw_rqt_ordermanager_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs nav_msgs geometry_msgs tuw_multi_robot_msgs tf
#  DEPENDS system_lib
)

SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTOMOC ON)

set (tuw_rqt_ordermanager_SRCS
  src/tuw_rqt_ordermanager/rqtordermanager.cpp
  src/tuw_rqt_ordermanager/item_robot.cpp
  src/tuw_rqt_ordermanager/item_order.cpp
  src/tuw_rqt_ordermanager/item_station.cpp
  src/tuw_rqt_ordermanager/map_transformation.cpp
)

set(tuw_rqt_ordermanager_HDRS
  include/tuw_rqt_ordermanager/rqtordermanager.h
  include/tuw_rqt_ordermanager/graphicsview.h
  include/tuw_rqt_ordermanager/robotdialog.h
  include/tuw_rqt_ordermanager/stationdialog.h
  include/tuw_rqt_ordermanager/orderdialog.h
  include/tuw_rqt_ordermanager/item_robot.h
  include/tuw_rqt_ordermanager/item_order.h
  include/tuw_rqt_ordermanager/item_station.h
  include/tuw_rqt_ordermanager/map_transformation.h
)

set(tuw_rqt_ordermanager_UIS
  src/tuw_rqt_ordermanager/rqtordermanager.ui
  src/tuw_rqt_ordermanager/robotdialog.ui
  src/tuw_rqt_ordermanager/stationdialog.ui
  src/tuw_rqt_ordermanager/orderdialog.ui
)

set(tuw_rqt_ordermanager_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(tuw_rqt_ordermanager_MOCS ${tuw_rqt_ordermanager_SRCS})
  qt5_wrap_ui(tuw_rqt_ordermanager_UIS_H ${tuw_rqt_ordermanager_HDRS})
  set(qt_LIBRARIES Qt5::Widgets)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(tuw_rqt_ordermanager_MOCS ${tuw_rqt_ordermanager_SRCS})
  qt4_wrap_ui(tuw_rqt_ordermanager_UIS_H ${tuw_rqt_ordermanager_UIS})
  set(qt_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tuw_rqt_ordermanager_INCLUDE_DIRECTORIES}
)

add_library(graphicsview SHARED src/tuw_rqt_ordermanager/graphicsview.cpp)
add_library(robotdialog SHARED src/tuw_rqt_ordermanager/robotdialog.cpp)
add_library(stationdialog SHARED src/tuw_rqt_ordermanager/stationdialog.cpp)
add_library(orderdialog SHARED src/tuw_rqt_ordermanager/orderdialog.cpp)
add_library(item_robot SHARED src/tuw_rqt_ordermanager/item_robot.cpp)
add_library(item_order SHARED src/tuw_rqt_ordermanager/item_order.cpp)
add_library(item_station SHARED src/tuw_rqt_ordermanager/item_station.cpp)
add_library(map_transformation SHARED src/tuw_rqt_ordermanager/map_transformation.cpp)

#add_dependencies(${PROJECT_NAME} nav_msgs_generate_messages_cpp)

target_link_libraries(
  graphicsview
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)
target_link_libraries(
  robotdialog
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)
target_link_libraries(
  stationdialog
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)
target_link_libraries(
  orderdialog
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)
target_link_libraries(
  item_robot
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)
target_link_libraries(
  item_order
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)
target_link_libraries(
  item_station
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${tuw_rqt_ordermanager_SRCS}
  ${tuw_rqt_ordermanager_MOCS}
  ${tuw_rqt_ordermanager_UIS_H}
)

## Declare a C++ executable
# add_executable(tuw_rqt_ordermanager_node src/tuw_rqt_ordermanager_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tuw_rqt_ordermanager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  graphicsview
  robotdialog
  stationdialog
  orderdialog
  item_robot
  item_order
  item_station
  map_transformation
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tuw_rqt_ordermanager tuw_rqt_ordermanager_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tuw_rqt_ordermanager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
