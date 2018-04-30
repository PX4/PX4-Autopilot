
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

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
	drivers/blinkm
	drivers/camera_trigger
	drivers/gps
	#drivers/hott
	drivers/imu/l3gd20
	drivers/imu/lsm303d
	#drivers/mkblctrl
	drivers/imu/mpu6000
	drivers/imu/mpu9250
	#drivers/oreoled
	drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/rgbled
	#drivers/rgbled_pwm
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/vmount
	modules/sensors

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
	systemcmds/led_control
	systemcmds/hardfault_log
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/tune_control
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver
	systemcmds/sd_bench
	systemcmds/motor_ramp

	#
	# Tests
	#
	drivers/distance_sensor/sf0x/sf0x_tests
	drivers/test_ppm
	modules/commander/commander_tests
	modules/mc_pos_control/mc_pos_control_tests
	lib/controllib/controllib_test
	modules/mavlink/mavlink_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	#
	# General system control
	#
	modules/commander
	modules/events
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/gpio_led
	modules/uavcan
	modules/land_detector
	modules/camera_feedback

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
	# Logging
	#
	modules/sdlog2
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
