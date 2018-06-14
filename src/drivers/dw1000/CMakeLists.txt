px4_add_module(
	MODULE drivers__dw1000
	MAIN dw1000
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		dw1000.cpp
	DEPENDS
		platforms__common
	)