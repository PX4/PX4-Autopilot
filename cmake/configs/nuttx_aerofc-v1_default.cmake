
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_module_list
	#
	# Board support modules
	#
	drivers/aerofc_adc
	drivers/distance_sensor
	drivers/gps
	drivers/barometer/ms5611
	drivers/magnetometer/hmc5883
	drivers/magnetometer/ist8310
	drivers/imu/mpu9250
	drivers/px4fmu
	drivers/stm32
	drivers/pwm_out_sim
	drivers/rc_input
	drivers/tap_esc
	modules/sensors

	#
	# System commands
	#
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/motor_test
	systemcmds/reboot
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/dumpfile
	systemcmds/ver

	#
	# General system control
	#
	modules/commander
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/land_detector

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/local_position_estimator
	modules/landing_target_estimator
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control # FIXME: only required for params needed by Navigator

	#
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	modules/dataman
)
