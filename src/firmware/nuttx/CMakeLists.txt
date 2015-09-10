px4_add_extra_builtin_cmds(OUT extra_builtins)

px4_nuttx_generate_builtin_commands(
	OUT builtin_commands.c
	MODULE_LIST ${module_libraries} ${extra_builtins}
	)

px4_nuttx_generate_romfs(OUT romfs.o
	ROOT ROMFS/px4fmu_common)

# add executable
add_executable(firmware_nuttx builtin_commands.c romfs.o)
set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)
set(main_link_flags
	"-T${nuttx_export_dir}/build/ld.script"
	"-Wl,-Map=${CMAKE_BINARY_DIR}/main.map"
	)
px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
set_target_properties(firmware_nuttx PROPERTIES LINK_FLAGS ${main_link_flags})

set(cmsis_lib ${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM4lf_math.a)

target_link_libraries(firmware_nuttx
	-Wl,--start-group
	${module_libraries}
	${cmsis_lib}
	apps nuttx nosys m gcc
	-Wl,--end-group)

px4_get_config(OUT_FW_OPTS fw_opts)
px4_nuttx_add_firmware(OUT ${CMAKE_CURRENT_BINARY_DIR}/fw_main.px4
	EXE ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
	${fw_opts}
	)

px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD}
	BUNDLE ${CMAKE_CURRENT_BINARY_DIR}/fw_main.px4)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
