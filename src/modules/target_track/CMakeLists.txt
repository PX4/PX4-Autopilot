############################################################################
#
#holdhigh module
#cetc-3
#############################################################################
px4_add_module(
	MODULE modules__target_track
	MAIN target_track
	STACK_MAIN 1500
	SRCS
		target_track.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
