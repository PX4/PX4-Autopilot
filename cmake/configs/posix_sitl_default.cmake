include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/boards/sitl
	drivers/device
	drivers/gps
	drivers/pwm_out_sim
	drivers/vmount

	platforms/common
	platforms/posix/drivers/accelsim
	platforms/posix/drivers/adcsim
	platforms/posix/drivers/airspeedsim
	platforms/posix/drivers/barosim
	platforms/posix/drivers/gpssim
	platforms/posix/drivers/gyrosim
	platforms/posix/drivers/ledsim
	platforms/posix/drivers/rgbledsim
	platforms/posix/drivers/tonealrmsim
	platforms/posix/px4_layer
	platforms/posix/work_queue

	systemcmds/esc_calib
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/reboot
	systemcmds/sd_bench
	systemcmds/topic_listener
	systemcmds/ver
	systemcmds/top
	systemcmds/motor_ramp

	modules/attitude_estimator_q
	modules/commander
	modules/dataman
	modules/ekf2
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/land_detector
	modules/logger
	modules/mavlink
	modules/mc_att_control
	modules/mc_pos_control
	modules/navigator
	modules/param
	modules/position_estimator_inav
	modules/local_position_estimator
	modules/replay
	modules/sdlog2
	modules/sensors
	modules/simulator
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/vtol_att_control

	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	lib/mathlib
	lib/mathlib/math/filter
	lib/rc
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/terrain_estimation
	lib/version

	examples/px4_simple_app
	examples/mc_att_control_multiplatform
	examples/mc_pos_control_multiplatform
	examples/ekf_att_pos_estimator
	examples/attitude_estimator_ekf
	examples/fixedwing_control

	#
	# Testing
	#
	drivers/sf0x/sf0x_tests
	lib/rc/rc_tests
	modules/commander/commander_tests
	modules/mc_pos_control/mc_pos_control_tests
	modules/controllib_test
	modules/mavlink/mavlink_tests
	modules/unit_test
	modules/uORB/uORB_tests
	systemcmds/tests

	)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

# Default config_sitl_rcS_dir (posix_sitl_default), this is overwritten later
# for the config posix_sitl_efk2 and set again, explicitly, for posix_sitl_lpe,
# which are based on posix_sitl_default.
set(config_sitl_rcS_dir
	posix-configs/SITL/init/lpe
	CACHE INTERNAL "init script dir for sitl"
	)

set(config_sitl_viewer
	jmavsim
	CACHE STRING "viewer for sitl"
	)
set_property(CACHE config_sitl_viewer
	PROPERTY STRINGS "jmavsim;none")

set(config_sitl_debugger
	disable
	CACHE STRING "debugger for sitl"
	)
set_property(CACHE config_sitl_debugger
	PROPERTY STRINGS "disable;gdb;lldb")

# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message("Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)
endif()

