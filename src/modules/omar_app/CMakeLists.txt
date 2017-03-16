############################################################################
px4_add_module(
	MODULE modules__omar_app
	MAIN omar_app
	STACK_MAIN 2000
	SRCS
		omar_app.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
