px4_add_module(
	MODULE drivers__imu__tdk__tdk_direct
	MAIN tdk_direct
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		tdk_direct_main.cpp
		TdkDirect.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)
