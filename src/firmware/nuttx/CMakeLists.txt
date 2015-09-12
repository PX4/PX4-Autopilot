px4_nuttx_generate_builtin_commands(
	OUT builtin_commands.c
	MODULE_LIST
		${module_libraries}
		${config_extra_builtin_cmds}
	)

px4_nuttx_generate_romfs(OUT romfs.o
	ROOT ROMFS/px4fmu_common)

px4_generate_parameters()

# add executable
add_executable(firmware_nuttx builtin_commands.c ${CMAKE_BINARY_DIR}/px4_parameters.c romfs.o)
set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)
set(main_link_flags
	"-T${nuttx_export_dir}/build/ld.script"
	"-Wl,-Map=${CMAKE_BINARY_DIR}/${BOARD}/main.map"
	)
px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
set_target_properties(firmware_nuttx PROPERTIES LINK_FLAGS ${main_link_flags})

set(fw_file ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.px4)

target_link_libraries(firmware_nuttx
	-Wl,--start-group
	${module_libraries}
	${config_extra_libs}
	apps nuttx nosys m gcc
	-Wl,--end-group)

px4_nuttx_add_firmware(OUT ${fw_file}
	BOARD ${BOARD}
	EXE ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
	${config_firmware_options}
	)

px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD}
	BUNDLE ${fw_file})

install(FILES ${fw_file} DESTINATION .)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
