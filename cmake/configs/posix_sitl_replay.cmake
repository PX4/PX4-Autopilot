
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/boards/sitl
	systemcmds/param
	systemcmds/ver
	systemcmds/perf
	modules/ekf2
	modules/ekf2_replay
	modules/logger
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
