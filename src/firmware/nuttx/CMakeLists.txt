px4_nuttx_generate_builtin_commands(
	OUT builtin_commands.c
	MODULE_LIST
		${module_libraries}
		${config_extra_builtin_cmds}
	)

# add executable
add_executable(firmware_nuttx
	builtin_commands.c)


set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)

set(link_libs
	romfs apps nuttx m gcc
	)

if(NOT ${BOARD} STREQUAL "sim")
	list(APPEND link_libs nosys)
	set(main_link_flags
		"-T${nuttx_export_dir}/build/ld.script"
		"-Wl,-Map=${CMAKE_BINARY_DIR}/${BOARD}/main.map"
		)
	px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
	set_target_properties(firmware_nuttx PROPERTIES LINK_FLAGS ${main_link_flags})
endif()

target_link_libraries(firmware_nuttx
	-Wl,--start-group
	${module_libraries}
	${config_extra_libs}
	${link_libs}
	-Wl,--end-group)

add_custom_target(check_weak
	COMMAND ${NM} firmware_nuttx | ${GREP} " w " | cat
	DEPENDS firmware_nuttx
	VERBATIM
	)

if(NOT ${BOARD} STREQUAL "sim")

	if(PX4_NUTTX_ROMFS)
		px4_nuttx_add_romfs(OUT romfs
			ROOT ${PX4_NUTTX_ROMFS}
			)
	else()
		if (config_io_board)
			set(extras "${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}.bin")
		endif()

		px4_nuttx_add_romfs(OUT romfs
			ROOT ROMFS/px4fmu_common
			EXTRAS ${extras}
			)
		if (config_io_board)
			add_dependencies(romfs fw_io)
		endif()
	endif()

	set(fw_file
		${CMAKE_CURRENT_BINARY_DIR}/${OS}-${BOARD}-${LABEL}.px4)

	px4_nuttx_add_firmware(OUT ${fw_file}
		BOARD ${BOARD}
		EXE ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
		PARAM_XML ${CMAKE_BINARY_DIR}/parameters.xml
		AIRFRAMES_XML ${CMAKE_BINARY_DIR}/airframes.xml
		)

	configure_file(gdbinit.in .gdbinit)

	add_custom_target(debug
		COMMAND ${GDB} ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_tui
		COMMAND ${GDBTUI} ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_ddd
		COMMAND ${DDD} --debugger ${GDB} ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_io
		COMMAND ${GDB}
			${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_io_tui
		COMMAND ${GDBTUI}
			${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_io_ddd
		COMMAND ${DDD} --debugger ${GDB}
			${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	if ("${BOARD}" STREQUAL "aerofc-v1")
		px4_add_upload_aero(OUT upload OS ${OS} BOARD ${BOARD}
			BUNDLE ${fw_file})
	else()
		px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD}
			BUNDLE ${fw_file})
	endif()
endif()

install(FILES ${fw_file} DESTINATION .)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
