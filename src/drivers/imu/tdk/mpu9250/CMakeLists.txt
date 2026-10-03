px4_add_module(
	MODULE drivers__imu__tdk__mpu9250
	MAIN mpu9250
	SRCS
		mpu9250_main.cpp
		TdkMpu9250.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)

# Keep acquisition optimized for throughput; CLI and configuration tables use
# the board's default optimization level, including debug/sanitizer builds.
set_source_files_properties(TdkMpu9250.cpp PROPERTIES COMPILE_FLAGS "${MAX_CUSTOM_OPT_LEVEL}")
