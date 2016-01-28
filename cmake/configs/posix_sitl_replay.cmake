include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/device
	drivers/boards/sitl
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	systemcmds/param
	systemcmds/ver
	systemcmds/perf
	modules/uORB
	modules/param
	modules/systemlib
	modules/ekf2
	modules/ekf2_replay
	modules/sdlog2
	modules/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
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
	replay
	CACHE STRING "viewer for sitl"
	)
set_property(CACHE config_sitl_viewer
	PROPERTY STRINGS "replay;none")

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
