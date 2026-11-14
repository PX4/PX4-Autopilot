px4_add_module(
	MODULE drivers__imu__tdk__icm42670p
	MAIN icm42670p
	SRCS
		icm42670p_main.cpp
		TdkIcm42670P.cpp
		TdkIcm42670PInitialization.cpp
		TdkIcm42670PConfigs.cpp
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)

# Keep acquisition optimized for throughput; CLI, startup and configuration tables use
# the board's default optimization level, including debug/sanitizer builds.
set_source_files_properties(TdkIcm42670P.cpp PROPERTIES COMPILE_FLAGS "${MAX_CUSTOM_OPT_LEVEL}")
