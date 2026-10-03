px4_add_module(
	MODULE drivers__imu__tdk__icm45686
	MAIN icm45686
	SRCS
		icm45686_main.cpp
		TdkIcm45686.cpp
		TdkIcm45686Initialization.cpp
		TdkIcm45686Configs.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)

# Keep acquisition optimized for throughput; CLI, startup and configuration tables use
# the board's default optimization level, including debug/sanitizer builds.
set_source_files_properties(TdkIcm45686.cpp PROPERTIES COMPILE_FLAGS "${MAX_CUSTOM_OPT_LEVEL}")
