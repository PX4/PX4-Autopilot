px4_add_module(
	MODULE modules__robosub__rs_object_detection
	MAIN rs_object_detection
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_object_detection.cpp
	)
