# set(SCOUT_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/scout_sdk)
add_subdirectory(scout_sdk)

px4_add_module(
	MODULE drivers__rover_interface
	MAIN rover_interface
	STACK_MAIN 4096
	COMPILE_FLAGS
	INCLUDES
		${SCOUT_SDK_DIR}/include
	SRCS
		RoverInterface.cpp
		RoverInterface.hpp
	DEPENDS
		scout_sdk
)
