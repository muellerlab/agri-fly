cmake_minimum_required(VERSION 3.2)

project(AgriFly CXX)

set(CMAKE_CXX_STANDARD 11) 
#set(CMAKE_VERBOSE_MAKEFILE ON)  #enable this to see exactly which build commands are invoked.

#set(CMAKE_CXX_FLAGS "-g -Werror -Wall -Wextra -Wpedantic") #enable all warnings, treat warnings as errors
set(CMAKE_CXX_FLAGS "-O0") #enable this for max optimization (faster simulation, e.g.)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/Common)
set(COMPONENTS_DIR ${CMAKE_CURRENT_LIST_DIR}/Components)

add_subdirectory(Common)
add_subdirectory(Components)
add_subdirectory(Simulator)
