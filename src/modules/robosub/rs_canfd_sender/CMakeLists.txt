px4_add_module(
	MODULE modules__robosub__rs_canfd_sender
	MAIN rs_canfd_sender
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_canfd_sender.cpp
	)
