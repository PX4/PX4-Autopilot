px4_add_module(
	MODULE modules__robosub__rs_canfd_receiver
	MAIN rs_canfd_receiver
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_canfd_receiver.cpp
		rs_canfd_receiver.hpp
		../rs_canfd_common/rs_canfd_common.hpp
	)
