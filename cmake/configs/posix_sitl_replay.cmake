
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

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
	modules/systemlib/param
	modules/systemlib
	modules/ekf2
	modules/ekf2_replay
	modules/sdlog2
	modules/logger
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/version
	lib/DriverFramework/framework
	lib/micro-CDR
	)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_sitl_rcS_dir
	posix-configs/SITL/init/replay
	CACHE INTERNAL "init script dir for sitl"
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
