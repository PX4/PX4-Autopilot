px4_add_module(
	MODULE modules__robosub__rs_pos_control
	MAIN rs_pos_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_pos_control.cpp
	)
