px4_add_module(
	MODULE modules__cdus_backstepping
	MAIN cdus_backstepping
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CdusBackstepping.cpp
		CdusBackstepping.hpp
	DEPENDS
		circuit_breaker
		mathlib
		px4_work_queue
	)