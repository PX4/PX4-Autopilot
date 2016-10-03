px4_add_module(
	MODULE modules__uavgp_commucation
	MAIN uavgp_commucation
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS -Os
	SRCS
		uavgp_commucation_main.cpp
	DEPENDS
		platforms__common
)
