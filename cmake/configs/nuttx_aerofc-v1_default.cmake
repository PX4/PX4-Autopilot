
px4_nuttx_configure(HWCLASS m4)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/stm32
	drivers/led
	drivers/px4fmu
	drivers/tap_esc
	drivers/mpu9250
	drivers/ms5611
	drivers/hmc5883
	drivers/gps
	drivers/ist8310
	drivers/ll40ls
	drivers/aerofc_adc
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
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control

	#
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	modules/dataman
)
