############################################################################
px4_add_module(
	MODULE modules__batman
	MAIN batman
	STACK_MAIN 2000
	SRCS
		batman.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
