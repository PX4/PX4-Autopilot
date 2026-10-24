px4_add_module(
	MODULE drivers__imu__st__lsm6dsr
	MAIN lsm6dsr
	COMPILE_FLAGS
	SRCS
		LSM6DSR.cpp
		LSM6DSR.hpp
		lsm6dsr_main.cpp
		ST_LSM6DSR_Registers.hpp
	DEPENDS
		drivers_accelerometer
		drivers_gyroscope
		px4_work_queue
	)
