px4_add_module(
	MODULE drivers__imu__st__lsm6dso
	MAIN lsm6dso
	COMPILE_FLAGS
	SRCS
		LSM6DSO.cpp
		LSM6DSO.hpp
		lsm6dso_main.cpp
		ST_LSM6DSO_Registers.hpp
	DEPENDS
		drivers_accelerometer
		drivers_gyroscope
		px4_work_queue
	)
