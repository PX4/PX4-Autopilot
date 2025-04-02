px4_add_module(
	MODULE modules__rs_motor_control
	MAIN rs_motor_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_motor_control.cpp
	)
