
px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 1)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	drivers/magnetometer
	drivers/telemetry

	drivers/batt_smbus
	drivers/camera_trigger
	drivers/gps
	drivers/imu/adis16477
	#drivers/imu/adis16497
	drivers/irlock
	drivers/mkblctrl
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/rc_input
	drivers/stm32
	drivers/stm32/adc
	drivers/tap_esc
	drivers/vmount
	modules/sensors

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/config
	systemcmds/dumpfile
	systemcmds/esc_calib
	systemcmds/hardfault_log
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/motor_ramp
	systemcmds/mtd
	systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	systemcmds/sd_bench
	systemcmds/top
	systemcmds/topic_listener
	systemcmds/tune_control
	systemcmds/ver

	#
	# Testing
	#
	drivers/distance_sensor/sf0x/sf0x_tests
	drivers/test_ppm
	#lib/rc/rc_tests
	modules/commander/commander_tests
	lib/controllib/controllib_test
	modules/mavlink/mavlink_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	#
	# General system control
	#
	modules/camera_feedback
	modules/commander
	modules/events
	modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/uavcan

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/landing_target_estimator
	modules/local_position_estimator
	modules/position_estimator_inav
	modules/wind_estimator

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
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	modules/dataman

	#
	# OBC challenge
	#
	#examples/bottle_drop

	#
	# Rover apps
	#
	#examples/rover_steering_control

	#
	# Segway
	#
	#examples/segway

	#
	# Demo apps
	#

	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	#examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)
