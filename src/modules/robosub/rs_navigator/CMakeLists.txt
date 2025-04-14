px4_add_module(
	MODULE modules__robosub__rs_navigator
	MAIN rs_navigator
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_navigator.cpp
	)
