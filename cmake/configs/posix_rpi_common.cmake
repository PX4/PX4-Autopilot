# This file is shared between posix_rpi_native.cmake
# and posix_rpi_cross.cmake.

include(posix/px4_impl_posix)

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the RPi.
add_definitions(
	-D__PX4_POSIX_RPI
	-D__DF_LINUX # For DriverFramework
)


set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_lsm9ds1_wrapper
	platforms/posix/drivers/df_ms5611_wrapper
	platforms/posix/drivers/df_hmc5883_wrapper
	platforms/posix/drivers/df_trone_wrapper
	platforms/posix/drivers/df_isl29501_wrapper

	#
	# System commands
	#
	systemcmds/param
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/perf

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
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
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/dataman
	modules/land_detector
	modules/navigator
	modules/mavlink

	#
	# PX4 drivers
	#
	drivers/gps
	drivers/navio_sysfs_rc_in
	drivers/navio_sysfs_pwm_out
	drivers/navio_gpio
	drivers/navio_rgbled
	drivers/pwm_out_sim

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/ecl
	lib/geo_lookup
	lib/launchdetection
	lib/led
	lib/external_lgpl
	lib/conversion
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/version
	lib/DriverFramework/framework

	#
	# POSIX
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
)

#
# DriverFramework driver
#
set(config_df_driver_list
	mpu9250
	lsm9ds1
	ms5611
	hmc5883
	trone
	isl29501
)
