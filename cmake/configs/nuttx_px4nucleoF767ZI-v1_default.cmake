
px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	drivers/magnetometer
	drivers/telemetry

	drivers/imu/adis16448
	drivers/blinkm
	drivers/imu/bma180
	drivers/imu/bmi160
	drivers/camera_trigger
	drivers/gps
	drivers/mkblctrl
	drivers/imu/mpu6000
	drivers/imu/mpu9250
	drivers/oreoled
	drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/rgbled
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/tap_esc
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
	# General system control
	#
	modules/commander
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/gpio_led
	modules/uavcan
	modules/land_detector

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
	# modules/segway # XXX Needs GCC 4.7 fix
	modules/fw_pos_control_l1
	modules/fw_att_control
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
	examples/bottle_drop

	#
	# Rover apps
	#
	examples/rover_steering_control

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)
