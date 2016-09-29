px4_add_module(
	MODULE modules__example_lz
	MAIN example_lz
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS -Os
	SRCS
		example_lz_main.cpp
	DEPENDS
		platforms__common
)