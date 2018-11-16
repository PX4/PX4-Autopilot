# This file is shared between posix_bbblue_native.cmake
# and posix_bbblue_cross.cmake.


# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the bbblue.
add_definitions(
	-D__PX4_POSIX_BBBLUE
	-D__PX4_POSIX
	-D__DF_LINUX        # For DriverFramework
	-D__DF_BBBLUE       # For DriverFramework
	-DRC_AUTOPILOT_EXT  # Enable extensions in Robotics Cape Library
#	-DDEBUG_BUILD
)

#optional __DF_BBBLUE_USE_RC_BMP280_IMP
add_definitions(
	-D__DF_BBBLUE_USE_RC_BMP280_IMP
)

#optional __PX4_BBBLUE_DEFAULT_MAVLINK_WIFI, default is "SoftAp"
#add_definitions(
#	-D__PX4_BBBLUE_DEFAULT_MAVLINK_WIFI="wlan"
#)


set(config_module_list
	#
	# Board support modules
	#
	#drivers/barometer
	drivers/batt_smbus
	drivers/differential_pressure
	drivers/distance_sensor
	#drivers/telemetry
	#drivers/boards

	modules/sensors

	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_bmp280_wrapper
		
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
	systemcmds/tune_control
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
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/gnd_att_control
	modules/gnd_pos_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Library modules
	#
	modules/logger
	modules/commander
	modules/dataman
	modules/land_detector
	modules/navigator
	modules/mavlink

	#
	# PX4 drivers
	#
	drivers/linux_sbus
	drivers/gps
	drivers/bbblue_adc
	drivers/linux_gpio
	drivers/linux_pwm_out
	drivers/pwm_out_sim

)

#
# DriverFramework driver
#
set(config_df_driver_list
	mpu9250
	bmp280
)
