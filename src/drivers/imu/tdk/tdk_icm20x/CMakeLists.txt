px4_add_module(
	MODULE drivers__imu__tdk__tdk_icm20x
	MAIN tdk_icm20x
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		tdk_icm20x_main.cpp
		TdkIcm20x.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)
