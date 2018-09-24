
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer/bmp280
	#drivers/differential_pressure
	#drivers/distance_sensor
	drivers/magnetometer/hmc5883
	drivers/telemetry/frsky_telemetry
	drivers/imu/mpu6000

	#drivers/batt_smbus
	#drivers/blinkm
	#drivers/camera_trigger
	drivers/gps
	drivers/px4flow
	drivers/px4fmu
	drivers/rc_input
	drivers/rgbled
	drivers/stm32
	drivers/stm32/adc
	#drivers/stm32/tone_alarm
	modules/sensors

	#
	# System commands
	#
	#systemcmds/bl_update
	systemcmds/config
	systemcmds/esc_calib
	systemcmds/hardfault_log
	systemcmds/led_control
	systemcmds/mixer
	#systemcmds/motor_ramp
	#systemcmds/mtd
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

	##
	## Testing
	##
	#drivers/distance_sensor/sf0x/sf0x_tests
	#drivers/test_ppm
	##lib/rc/rc_tests
	#modules/commander/commander_tests
	#lib/controllib/controllib_test
	#modules/mavlink/mavlink_tests
	#modules/mc_pos_control/mc_pos_control_tests
	#modules/uORB/uORB_tests
	#systemcmds/tests

	#
	# General system control
	#
	#modules/camera_feedback
	modules/commander
	modules/events
	modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/landing_target_estimator
	modules/local_position_estimator
	#modules/wind_estimator

	#
	# Vehicle Control
	#
	modules/fw_att_control
	modules/fw_pos_control_l1
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

	##
	## OBC challenge
	##
	#examples/bottle_drop

	##
	## Rover apps
	##
	#examples/rover_steering_control

	##
	## Segway
	##
	#examples/segway

	##
	## Demo apps
	##

	## Tutorial code from
	## https://px4.io/dev/px4_simple_app
	#examples/px4_simple_app

	## Tutorial code from
	## https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	## Tutorial code from
	## https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	## Hardware test
	#examples/hwtest
)
