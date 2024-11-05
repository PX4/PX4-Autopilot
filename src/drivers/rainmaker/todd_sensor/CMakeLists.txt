px4_add_module(
	MODULE drivers__todd_sensor
	MAIN todd_sensor
	SRCS
		ToddSensor.cpp
		ToddSensor.hpp
	DEPENDS
		px4_work_queue
	)
