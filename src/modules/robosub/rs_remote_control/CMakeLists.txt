px4_add_module(
	MODULE modules__robosub__rs_remote_control
	MAIN rs_remote_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_remote_control.cpp
	)
