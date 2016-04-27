include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m3 CONFIG nsh ROMFS y ROMFSROOT mavstation_common)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list

	#
	# Board support modules
	#

	drivers/device
	drivers/stm32
	drivers/boards/mavstation

	#
	# Library modules
	#
	modules/param
	modules/systemlib

	#
	# Libraries
	#
	# had to add for cmake, not sure why wasn't in original config
	platforms/nuttx
	platforms/common
	platforms/nuttx/px4_layer
)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	PRIORITY "SCHED_PRIORITY_DEFAULT"
	MAIN "sercon" STACK_MAIN "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	PRIORITY "SCHED_PRIORITY_DEFAULT"
	MAIN "serdis" STACK_MAIN "2048")
