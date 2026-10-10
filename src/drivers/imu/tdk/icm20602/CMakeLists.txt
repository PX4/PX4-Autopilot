px4_add_module(
	MODULE drivers__imu__tdk__icm20602
	MAIN icm20602
	SRCS
		icm20602_main.cpp
		TdkIcm20602.cpp
		TdkIcm20602Initialization.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)

# Keep acquisition optimized for throughput; CLI, startup and configuration tables use
# the board's default optimization level, including debug/sanitizer builds.
set_source_files_properties(TdkIcm20602.cpp PROPERTIES COMPILE_FLAGS "${MAX_CUSTOM_OPT_LEVEL}")
