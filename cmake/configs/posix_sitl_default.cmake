
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	#drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor

	drivers/airspeed
	drivers/boards
	drivers/camera_trigger
	drivers/device
	drivers/gps
	drivers/linux_gpio
	drivers/pwm_out_sim
	drivers/vmount

	modules/sensors
	platforms/posix/drivers/accelsim
	platforms/posix/drivers/adcsim
	platforms/posix/drivers/airspeedsim
	platforms/posix/drivers/barosim
	platforms/posix/drivers/gpssim
	platforms/posix/drivers/gyrosim
	platforms/posix/drivers/ledsim
	platforms/posix/drivers/tonealrmsim

	#
	# System commands
	#
	#systemcmds/bl_update
	#systemcmds/config
	#systemcmds/dumpfile
	systemcmds/esc_calib
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/motor_ramp
	#systemcmds/mtd
	#systemcmds/nshterm
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
	#drivers/test_ppm
	lib/rc/rc_tests
	modules/commander/commander_tests
	lib/controllib/controllib_test
	modules/mavlink/mavlink_tests
	modules/mc_pos_control/mc_pos_control_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	platforms/posix/tests/hello
	platforms/posix/tests/hrt_test
	platforms/posix/tests/muorb
	platforms/posix/tests/vcdev_test

	#
	# General system control
	#
	modules/commander
	modules/events
	#modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/replay
	modules/simulator
	#modules/uavcan

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
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
	modules/dataman
	modules/systemlib/param
	modules/systemlib
	modules/uORB
	modules/landing_target_estimator

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
	lib/mixer
	lib/mathlib
	lib/rc
	lib/terrain_estimation
	lib/tunes
	lib/version

	#
	# OBC challenge
	#
	examples/bottle_drop

	#
	# Rover apps
	#
	examples/rover_steering_control

	#
	# HippoCampus example (AUV from TUHH)
	#
	examples/uuv_example_app

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

	# Template Module
	templates/module

	# Hardware test
	#examples/hwtest
)

# Default config_sitl_rcS_dir (posix_sitl_default), this is overwritten later
# for the config posix_sitl_efk2 and set again, explicitly, for posix_sitl_lpe,
# which are based on posix_sitl_default.
set(config_sitl_rcS_dir posix-configs/SITL/init/ekf2 CACHE INTERNAL "init script dir for sitl")

set(config_sitl_viewer jmavsim CACHE STRING "viewer for sitl")
set_property(CACHE config_sitl_viewer PROPERTY STRINGS "jmavsim;none")

set(config_sitl_debugger disable CACHE STRING "debugger for sitl")
set_property(CACHE config_sitl_debugger PROPERTY STRINGS "disable;gdb;lldb")

# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message("Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)
endif()
