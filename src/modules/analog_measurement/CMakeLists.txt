
px4_add_module(
	MODULE modules__analog_measurement
	MAIN analog_measurement
	COMPILE_FLAGS
	SRCS
		analog_measurement.cpp
		analog_measurement.hpp
	DEPENDS
		px4_work_queue
	)
