px4_add_module(
	MODULE examples__px4_my_hello
	MAIN px4_my_hello
	STACK_MAIN 2000
	SRCS
		px4_my_hello.c
	DEPENDS
	)
