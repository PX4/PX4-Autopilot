
px4_add_module(
	MODULE modules__navigator__navigator_tests
	MAIN navigator_tests
	SRCS
		navigator_tests.cpp
		tracker_test.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
