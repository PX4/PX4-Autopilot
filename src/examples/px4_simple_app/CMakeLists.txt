px4_add_module(
	MODULE examples__px4_simple_app
	MAIN px4_simple_app
	STACK_MAIN 2000
	SRCS
		px4_simple_app.c
	DEPENDS
	)
