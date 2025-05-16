px4_add_module(
	MODULE modules__robosub__rs_canfd_receiver
	MAIN rs_canfd_receiver
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_canfd_receiver.cpp
	)
