px4_add_module(
	MODULE modules__robosub__rs_canfd_socket
	MAIN rs_canfd_socket
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_canfd_socket.cpp
		rs_canfd_socket.hpp
	)
