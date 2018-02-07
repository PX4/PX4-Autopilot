px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common IO px4io-v2)

#set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	#drivers/barometer
	drivers/differential_pressure
	#drivers/magnetometer

	#drivers/adis16448
	drivers/airspeed
	drivers/barometer/ms5611
	#drivers/blinkm
	#drivers/bmi160
	#drivers/bmp280
	drivers/boards
	#drivers/bst
	drivers/camera_trigger
	drivers/device
	#drivers/frsky_telemetry
	drivers/gps
	#drivers/hott
	#drivers/iridiumsbd
	#drivers/irlock
	drivers/l3gd20
	drivers/led
	drivers/lsm303d
	drivers/magnetometer/hmc5883
	drivers/magnetometer/lis3mdl
	#drivers/mb12xx
	#drivers/mkblctrl
	drivers/mpu6000
	drivers/mpu9250
	#drivers/oreoled
	#drivers/protocol_splitter
	drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/px4io
	drivers/rgbled
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	#drivers/tap_esc
	drivers/vmount

	# distance sensors
	drivers/distance_sensor/ll40ls
	drivers/distance_sensor/mb12xx
	drivers/distance_sensor/sf0x
	drivers/distance_sensor/sf1xx
	drivers/distance_sensor/srf02
	drivers/distance_sensor/srf02_i2c
	drivers/distance_sensor/teraranger
	drivers/distance_sensor/tfmini
	drivers/distance_sensor/ulanding
	modules/sensors

	#
	# System commands
	#
	#systemcmds/bl_update
	#systemcmds/config
	#systemcmds/dumpfile
	#systemcmds/esc_calib
	systemcmds/hardfault_log
	#systemcmds/led_control
	systemcmds/mixer
	#systemcmds/motor_ramp
	#systemcmds/motor_test
	systemcmds/mtd
	#systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	#systemcmds/sd_bench
	systemcmds/top
	#systemcmds/topic_listener
	systemcmds/tune_control
	systemcmds/ver

	#
	# Testing
	#
	#drivers/distance_sensor/sf0x/sf0x_tests
	#drivers/test_ppm
	#lib/controllib/controllib_test
	#lib/rc/rc_tests
	#modules/commander/commander_tests
	#modules/mavlink/mavlink_tests
	#modules/mc_pos_control/mc_pos_control_tests
	#modules/uORB/uORB_tests
	#systemcmds/tests

	#
	# General system control
	#
	modules/camera_feedback
	modules/commander
	modules/events
	#modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	#modules/uavcan

	#
	# Estimation modules
	#
	#modules/attitude_estimator_q
	modules/ekf2
	#modules/local_position_estimator
	#modules/position_estimator_inav
	#modules/landing_target_estimator

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
	#modules/sdlog2

	#
	# Library modules
	#
	modules/dataman
	modules/systemlib
	modules/systemlib/param
	modules/uORB

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/led
	lib/mathlib
	lib/mixer
	#lib/terrain_estimation
	lib/tunes
	lib/version

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
