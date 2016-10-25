include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_uavcan_num_ifaces 1)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/kinetis
	drivers/kinetis/adc
	drivers/kinetis/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/boards/nxphlite-v1
    drivers/rgbled
##SPACE	drivers/mpu6000
##SPACE	drivers/mpu9250
##SPACE	drivers/hmc5883
##SPACE	drivers/ms5611
##SPACE	drivers/mb12xx
##SPACE	drivers/srf02
##SPACE	drivers/sf0x
##SPACE	drivers/sf1xx
##SPACE	drivers/ll40ls
	drivers/trone
	drivers/gps
##SPACE	drivers/pwm_out_sim
##SPACE	drivers/hott
##SPACE	drivers/hott/hott_telemetry
##SPACE	drivers/hott/hott_sensors
##SPACE	drivers/blinkm
	drivers/airspeed
##SPACE	drivers/ets_airspeed
##SPACE	drivers/meas_airspeed
	drivers/frsky_telemetry
	modules/sensors
##SPACE	drivers/mkblctrl
##SPACE	drivers/px4flow
##SPACE	drivers/oreoled
##SPACE	drivers/vmount
# NOT Portable YET drivers/pwm_input
	drivers/camera_trigger
##SPACE	drivers/bst
##SPACE	drivers/snapdragon_rc_pwm
##SPACE	drivers/lis3mdl
##SPACE	drivers/bmp280
#No External SPI drivers/bma180
#No External SPI 	drivers/bmi160
##SPACE	drivers/tap_esc

	#
	# System commands
	#
##SPACE	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
##SPACE	systemcmds/esc_calib
## Needs bbsrm 	systemcmds/hardfault_log
	systemcmds/reboot
##SPACE	systemcmds/topic_listener
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
##SPACE	systemcmds/mtd
##SPACE	systemcmds/dumpfile
	systemcmds/ver
##SPACE	systemcmds/sd_bench
##SPACE	systemcmds/motor_ramp

	#
	# Testing
	#
##SPACE	drivers/sf0x/sf0x_tests
### NOT Portable YET drivers/test_ppm
##SPACE	modules/commander/commander_tests
##SPACE	modules/controllib_test
##SPACE	modules/mavlink/mavlink_tests
##SPACE	modules/unit_test
##SPACE	modules/uORB/uORB_tests
##SPACE	systemcmds/tests

	#
	# General system control
	#
##SPACE	modules/commander
	modules/load_mon
##SPACE	modules/navigator
	modules/mavlink
	modules/gpio_led
##NO CAN YET	modules/uavcan
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
##SPACE	modules/local_position_estimator
##SPACE	modules/ekf2

	#
	# Vehicle Control
	#
	# modules/segway # XXX Needs GCC 4.7 fix
##SPACE	modules/fw_pos_control_l1
	modules/fw_att_control
##SPACE modules/mc_att_control
	modules/mc_pos_control
##SPACE modules/vtol_att_control

	#
	# Logging
	#
#	modules/sdlog2
	modules/logger

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/rc
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/version
	lib/DriverFramework/framework
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common
	platforms/nuttx/px4_layer

	#
	# OBC challenge
	#
##SPACE	modules/bottle_drop

	#
	# Rover apps
	#
##SPACE	examples/rover_steering_control

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
##SPACE	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	#examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest

	# EKF
##SPACE	examples/ekf_att_pos_estimator
)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_extra_libs
##NO CAN YET	uavcan
##NO CAN YET	uavcan_stm32_driver
	)

set(config_io_extra_libs
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	PRIORITY "SCHED_PRIORITY_DEFAULT"
	MAIN "sercon"
	STACK_MAIN "2048"
	COMPILE_FLAGS "-Os")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	PRIORITY "SCHED_PRIORITY_DEFAULT"
	MAIN "serdis"
	STACK_MAIN "2048"
	COMPILE_FLAGS "-Os")
