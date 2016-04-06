include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/device
	drivers/boards/sitl
	drivers/pwm_out_sim
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	platforms/posix/drivers/adcsim
	platforms/posix/drivers/gpssim
	drivers/gps
	platforms/posix/drivers/tonealrmsim
	platforms/posix/drivers/accelsim
	platforms/posix/drivers/airspeedsim
	platforms/posix/drivers/barosim
	platforms/posix/drivers/gyrosim
	platforms/posix/drivers/rgbledsim
	platforms/posix/drivers/ledsim
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/perf
	modules/uORB
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/sensors
	modules/simulator
	modules/mavlink
	modules/attitude_estimator_ekf
	modules/attitude_estimator_q
	modules/ekf2
	modules/ekf_att_pos_estimator
	modules/position_estimator_inav
	modules/navigator
	modules/vtol_att_control
	modules/mc_pos_control
	modules/mc_att_control
	modules/mc_pos_control_multiplatform
	modules/mc_att_control_multiplatform
	modules/land_detector
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/dataman
	modules/sdlog2
	modules/commander
	modules/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	examples/px4_simple_app
	)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_sitl_rcS
	posix-configs/SITL/init/rcS
	CACHE FILEPATH "init script for sitl"
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



add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
