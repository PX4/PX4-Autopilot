px4_add_module(
	MODULE drivers__pmw3901
	MAIN pmw3901
	COMPILE_FLAGS
		-Wno-sign-compare
	SRCS
		pmw3901.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
