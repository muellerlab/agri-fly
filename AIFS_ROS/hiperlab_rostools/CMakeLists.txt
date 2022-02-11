cmake_minimum_required(VERSION 2.8.3)
project(hiperlab_rostools)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)


set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "-g -O0") #enable all warnings, treat warnings as errors
set(CMAKE_CXX_FLAGS "-O3")





## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hiperlab_common
  hiperlab_components
  roscpp
  rospy
  message_generation
  std_msgs
  image_transport
  cv_bridge
  geometry_msgs
  tf
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time program_options)
include_directories(${Boost_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


#find_package(OpenCV 3)
#include_directories(${OpenCV_INCLUDE_DIRS})
#message(STATUS "Found OpenCV at ${OpenCV_INCLUDE_DIRS}")
#target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV at ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})





## Setup the AirSim Library
set(AIRSIM_ROOT /home/$ENV{USER}/Documents/Repos/LabCode/agri-fly/AIFS_AirSim)  
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")



include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${AIRSIM_ROOT}/MavLinkCom/mavlink
  ${AIRSIM_ROOT}/MavLinkCom/MavLinkComGenerator
  ${AIRSIM_ROOT}/MavLinkCom/MavLinkTest
  ${AIRSIM_ROOT}/MavLinkCom/src
  ${OpenCV_INCLUDE_DIRS}
)




## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

add_message_files(
  FILES
	estimator_output.msg
	mocap_output.msg  
	simulator_truth.msg
	radio_command.msg
	telemetry.msg
	joystick_values.msg
	controller_diagnostics.msg
	controller_input.msg
	controller_output.msg
	planner_diagnostics.msg
	planner_input.msg
	planner_output.msg
	planner_statistics.msg
	polynomial_trajectory.msg
)

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

# Generate added messages and services with any dependencies listed here
  generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs # Or other packages containing msgs
)


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
#  INCLUDE_DIRS include
#  LIBRARIES hiperlab_rostools
  CATKIN_DEPENDS hiperlab_common hiperlab_components roscpp rospy message_runtime image_transport cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hiperlab_rostools.cpp
# )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hiperlab_rostools_node.cpp)

add_executable(quad_mocap_rates_control 
	src/QuadMocapRatesControl/main.cpp
	src/QuadMocapRatesControl/ExampleVehicleStateMachine.cpp
)
target_link_libraries(quad_mocap_rates_control ${catkin_LIBRARIES})
add_dependencies(quad_mocap_rates_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simulator src/Simulator/main.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} AirLib MavLinkCom)
add_dependencies(simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(air_sim_bridge src/AirSimBridge/main.cpp)
target_link_libraries(air_sim_bridge ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} AirLib MavLinkCom)
add_dependencies(air_sim_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_rappids_planner_controller
	src/QuadRappidsPlannerAndController/main.cpp
	src/QuadRappidsPlannerAndController/ExampleVehicleStateMachine.cpp)
target_link_libraries(quad_rappids_planner_controller ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_dependencies(quad_rappids_planner_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sync_simulator src/SyncSimulator/main.cpp)
target_link_libraries(sync_simulator ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} AirLib MavLinkCom)
add_dependencies(sync_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(vehicle_monitor 
    src/VehicleMonitor/main.cpp
    src/VehicleMonitor/VehicleMonitor.cpp
    src/VehicleMonitor/JoystickMonitor.cpp
)
target_link_libraries(vehicle_monitor ${catkin_LIBRARIES})
add_dependencies(vehicle_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hiperlab_rostools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
