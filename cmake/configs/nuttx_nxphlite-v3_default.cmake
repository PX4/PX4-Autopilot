
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(UAVCAN_PLATFORM kinetis CACHE STRING "uavcan platform")
set(UAVCAN_TIMER 1)
set(config_uavcan_num_ifaces 2)

# user-configurable UART ports
set(board_serial_ports
	GPS1:/dev/ttyS3
	TEL1:/dev/ttyS4)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	drivers/magnetometer
	drivers/telemetry

	drivers/barometer/mpl3115a2
	drivers/batt_smbus
	drivers/blinkm
	drivers/camera_trigger
	drivers/imu/fxas21002c
	drivers/imu/fxos8701cq
	drivers/gps
	drivers/kinetis
	drivers/kinetis/adc
	drivers/kinetis/tone_alarm
	drivers/imu/l3gd20
	drivers/mkblctrl
	drivers/imu/mpu6000
	drivers/imu/mpu9250
	drivers/oreoled
# NOT Portable YET drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/rc_input
	drivers/rgbled
	drivers/rgbled_pwm
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
## Needs bbsrm 	systemcmds/hardfault_log
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/motor_ramp
	systemcmds/motor_test
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
	systemcmds/usb_connected
	systemcmds/ver

	#
	# Testing
	#
	drivers/distance_sensor/sf0x/sf0x_tests
### NOT Portable YET 	drivers/test_ppm
	#lib/rc/rc_tests
	modules/commander/commander_tests
	lib/controllib/controllib_test
	modules/mavlink/mavlink_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	#
	# General system control
	#
	modules/commander
	modules/events
	modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/uavcan
	modules/camera_feedback

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
	# https://px4.io/dev/debug_values
	examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	examples/fixedwing_control

	# Hardware test
	examples/hwtest
)
