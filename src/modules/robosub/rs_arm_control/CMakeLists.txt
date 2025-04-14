px4_add_module(
	MODULE modules__robosub__rs_arm_control
	MAIN rs_arm_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_arm_control.cpp
	)
