include(configs/qcom/qurt_eagle_common)

add_definitions(
   -D__USING_SNAPDRAGON_LEGACY_DRIVER
   )

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_bmp280_wrapper

	#
	# System commands
	#
	systemcmds/param

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

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/commander
	modules/land_detector

	#
	# PX4 drivers
	#
	drivers/gps
	drivers/pwm_out_rc_in
	drivers/qshell/qurt

	#
	# FC_ADDON drivers
	#
	platforms/qurt/fc_addon/rc_receiver
	platforms/qurt/fc_addon/uart_esc

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/ecl
	lib/geo_lookup
	lib/conversion
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/version
	lib/DriverFramework/framework

	#
	# QuRT port
	#
	platforms/common
	platforms/qurt/px4_layer
	platforms/posix/work_queue

	#
	# sources for muorb over fastrpc
	#
	modules/muorb/adsp
	)

set(config_df_driver_list
	mpu9250
	bmp280
	)
