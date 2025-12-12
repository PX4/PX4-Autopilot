px4_add_module(
	MODULE modules__cdus_backstepping_attitude
	MAIN cdus_backstepping_attitude
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CdusBacksteppingAttitude.cpp
		CdusBacksteppingAttitude.hpp
	DEPENDS
		circuit_breaker
		mathlib
		px4_work_queue
	)