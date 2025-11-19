px4_add_module(
	MODULE modules__cdus_rate_control
	MAIN cdus_rate_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CdusRateControl.cpp
		CdusRateControl.hpp
	DEPENDS
		circuit_breaker
		mathlib
		RateControl
		px4_work_queue
	)