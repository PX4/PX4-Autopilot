
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	#drivers/magnetometer
	#drivers/telemetry

	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/px4fmu
	drivers/imu/lsm303d
	drivers/imu/l3gd20
	drivers/gps
	drivers/pwm_out_sim
	modules/sensors
	#drivers/pwm_input
	#drivers/camera_trigger
	drivers/rc_input

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/config
	#systemcmds/dumpfile
	#systemcmds/esc_calib
	systemcmds/mixer
	#systemcmds/motor_ramp
	systemcmds/mtd
	systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	#systemcmds/sd_bench
	systemcmds/top
	#systemcmds/topic_listener
	systemcmds/ver

	#
	# Testing
	#
	#drivers/distance_sensor/sf0x/sf0x_tests
	#drivers/test_ppm
	#lib/rc/rc_tests
	#modules/commander/commander_tests
	#lib/controllib/controllib_test
	#modules/mavlink/mavlink_tests
	#modules/unit_test
	#modules/uORB/uORB_tests
	#systemcmds/tests

	#
	# General system control
	#
	modules/commander
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/uavcan
	modules/land_detector

	#
	# Estimation modules
	#
	#modules/attitude_estimator_q
	#modules/position_estimator_inav
	#modules/local_position_estimator
	modules/ekf2

	#
	# Vehicle Control
	#
	#modules/fw_att_control
	#modules/fw_pos_control_l1
	#modules/gnd_att_control
	#modules/gnd_pos_control
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
	# Demo apps
	#
	#examples/math_demo
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
