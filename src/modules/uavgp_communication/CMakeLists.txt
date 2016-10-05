px4_add_module(
	MODULE modules__uavgp_communication
	MAIN uavgp_communication
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS -Os
	SRCS
		uavgp_communication_main.cpp
	DEPENDS
		platforms__common
)
