
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

add_definitions(
	-D__PX4_POSIX_BEBOP
	-D__DF_LINUX # Define needed DriverFramework
	-D__DF_BEBOP # Define needed DriverFramework
	)

set(config_module_list

	#
	# Board support modules
	#
	drivers/linux_sbus
	modules/sensors
	platforms/posix/drivers/df_ms5607_wrapper
	platforms/posix/drivers/df_mpu6050_wrapper
	platforms/posix/drivers/df_ak8963_wrapper
	platforms/posix/drivers/df_bebop_bus_wrapper
	platforms/posix/drivers/df_bebop_rangefinder_wrapper
	platforms/posix/drivers/bebop_flow

	#
	# System commands
	#
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/topic_listener
	systemcmds/perf

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
	modules/landing_target_estimator
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/vtol_att_control

	#
	# Library modules
	#
	modules/sdlog2
	modules/logger
	modules/commander
	modules/dataman
	modules/land_detector
	modules/navigator
	modules/mavlink

	#
	# PX4 drivers
	#
	drivers/gps
)

set(config_df_driver_list
	ms5607
	mpu6050
	ak8963
	bebop_bus
	bebop_rangefinder
	mt9v117
)
