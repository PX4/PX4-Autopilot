include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchains/Toolchain-cbmc.cmake)

set(config_module_list
	modules/commander2
	platforms/common
	)

set(config_firmware_options
	)

set(config_extra_builtin_cmds
	)

set(config_firmware)
