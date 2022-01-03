project(Simulator CXX)

set(APPLICATION_NAME RappidsSimulator )

file(GLOB_RECURSE PROJECT_SOURCE 
	"*.cpp")
	
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time program_options)

add_executable(${APPLICATION_NAME} ${PROJECT_SOURCE})
set_target_properties(${APPLICATION_NAME} PROPERTIES OUTPUT_NAME_DEBUG ${APPLICATION_NAME}D)
set_target_properties(${APPLICATION_NAME} PROPERTIES OUTPUT_NAME_RELEASE ${APPLICATION_NAME})

add_dependencies(${APPLICATION_NAME} Common Components)

# Swap this for your root for the airsim library
set(AIRSIM_ROOT /home/clark/Documents/Repos/LabCode/agri-fly/AIFS_AirSim) 

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)


set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "-g -O0") #enable all warnings, treat warnings as errors
set(CMAKE_CXX_FLAGS "-O3")

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

find_package( OpenCV 3.2.0 REQUIRED )
message(STATUS "Found OpenCV at ${OpenCV_INCLUDE_DIRS}")
include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)

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
  #${OPEN_VINS_ROOT}/ov_msckf/src
)

target_link_libraries(${APPLICATION_NAME} 
	Common
	Components
	AirLib
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	MavLinkCom
	#ov_msckf_lib
)
