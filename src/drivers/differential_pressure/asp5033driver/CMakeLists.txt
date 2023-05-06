

px4_add_module(
	MODULE drivers__differential_pressure__asp5033driver
	MAIN asp5033driver
	COMPILE_FLAGS
		-DDEBUG_BUILD
	SRCS
		asp5033driver_main.cpp
		ASP5033Driver.cpp
		ASP5033Driver.hpp
	DEPENDS
		px4_work_queue
	)
