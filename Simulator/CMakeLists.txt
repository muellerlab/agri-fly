project(Simulator CXX)

#Common includes for all Apps:
include_directories(	
	${COMMON_DIR}
	${COMPONENTS_DIR}
)

add_subdirectory(Rappids_Simulator)

