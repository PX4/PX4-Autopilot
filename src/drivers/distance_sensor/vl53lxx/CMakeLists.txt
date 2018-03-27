px4_add_module(
	MODULE drivers__vl53lxx
	MAIN vl53lxx
	COMPILE_FLAGS
		-Wno-sign-compare
	SRCS
		vl53lxx.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :