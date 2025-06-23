px4_add_module(
	MODULE modules__robosub__rs_remote_control
	MAIN rs_remote_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_remote_control.cpp
		rs_remote_control.hpp
	MODULE_CONFIG
		rs_remote_control.yaml
	DEPENDS
		px4_work_queue
	)
