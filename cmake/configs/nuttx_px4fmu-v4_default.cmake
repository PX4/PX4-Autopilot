
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

	drivers/airspeed
	drivers/blinkm
	drivers/bma180
	drivers/bmi055
	drivers/bmi160
	drivers/boards
	drivers/bst
	drivers/camera_trigger
	drivers/device
	drivers/frsky_telemetry
	drivers/gps
	drivers/hott
	drivers/iridiumsbd
	drivers/irlock
	drivers/led
	drivers/mkblctrl
	drivers/mpu6000
	drivers/mpu9250
	drivers/oreoled
	drivers/pca9685
	drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/rgbled
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
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
	modules/mc_pos_control/mc_pos_control_tests
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
	modules/sdlog2

	#
	# Library modules
	#
	modules/systemlib/param
	modules/systemlib
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	lib/led
	lib/mathlib
	lib/mathlib/math/filter
	lib/mixer
	lib/rc
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/terrain_estimation
	lib/version

	#
	# Platform
	#
	platforms/common
	platforms/nuttx
	platforms/nuttx/px4_layer

	#
	# OBC challenge
	#
	examples/bottle_drop

	#
	# Rover apps
	#
	examples/rover_steering_control

	#
	# Segway
	#
	examples/segway

	#
	# Demo apps
	#

	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	examples/fixedwing_control

	# Hardware test
	examples/hwtest
)