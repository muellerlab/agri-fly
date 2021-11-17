# CMake useful variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

project(Common CXX)

include_directories(
    ${PROJECT_SOURCE_DIR}
)

include_directories(	
)

file(GLOB_RECURSE LIB_SOURCES 
	"./Common/*.cpp"
)

set(LIBRARY_NAME Common)

add_library(${LIBRARY_NAME} SHARED ${LIB_SOURCES})

set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME_DEBUG ${LIBRARY_NAME}d )
set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME_RELEASE ${LIBRARY_NAME})

