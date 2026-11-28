px4_add_module(
	MODULE drivers__imu__tdk__tdk_icm42x
	MAIN tdk_icm42x
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		tdk_icm42x_main.cpp
		TdkIcm42x.cpp
		TdkIcm42xConfigs.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)
