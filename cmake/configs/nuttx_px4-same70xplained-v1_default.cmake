
px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

##set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	drivers/magnetometer

	drivers/device
	drivers/samv7
#WIP 	drivers/samv7/adc
	drivers/samv7/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/boards
	drivers/rgbled
	drivers/mpu6000
	drivers/mpu9250
	drivers/lsm303d
	drivers/l3gd20
	drivers/gps
#WIP 	drivers/pwm_out_sim
	drivers/hott
	drivers/blinkm
	drivers/airspeed
	drivers/frsky_telemetry
	modules/sensors
	#drivers/mkblctrl
	drivers/px4flow
	drivers/oreoled
##	drivers/gimbal
#WIP  	drivers/pwm_input
#WIP  	drivers/camera_trigger
	drivers/bst

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
#WIP systemcmds/hardfault_log
	systemcmds/reboot
	#systemcmds/topic_listener
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
#	systemcmds/mtd Excluded until TWIHS works
	systemcmds/dumpfile
	systemcmds/ver

	#
	# General system control
	#
	modules/commander
	modules/navigator
	modules/mavlink
	modules/gpio_led
##WIP	modules/uavcan
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
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
	modules/sdlog2
##	modules/logger

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
	lib/led
	lib/mathlib
	lib/mixer
	lib/rc
	lib/terrain_estimation
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
