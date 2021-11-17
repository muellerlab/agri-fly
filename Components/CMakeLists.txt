# CMake useful variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")



project(Components_lib_project CXX)
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(
    ${PROJECT_SOURCE_DIR}
)

include_directories(	
	${COMMON_DIR}
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE LIB_SOURCES
	"Components/*.cpp"
)

set(LIBRARY_NAME Components)
add_library(${LIBRARY_NAME} STATIC ${LIB_SOURCES})
target_link_libraries(${LIBRARY_NAME} Common)

set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME_DEBUG ${LIBRARY_NAME}D )
set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME_RELEASE ${LIBRARY_NAME})

target_link_libraries( ${LIBRARY_NAME} Common )