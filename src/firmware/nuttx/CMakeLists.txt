px4_nuttx_generate_builtin_commands(
	OUT builtin_commands.c
	MODULE_LIST
		${module_libraries}
		${config_extra_builtin_cmds}
	)

# add executable
add_executable(firmware_nuttx
	builtin_commands.c)


set(nuttx_export_dir ${PX4_BINARY_DIR}/${BOARD}/NuttX/nuttx/nuttx-export)

set(link_libs
	apps nuttx m gcc
	)

if("${romfs_used}" STREQUAL "y")
	list(APPEND link_libs romfs)
endif()

#
# Bootloaders use a different ld file, a seperate startup file
# and need to override the os version of sched_process_timer
#
set(startup_libs)

if("${config_nuttx_config}" STREQUAL "bootloader")
	set(nuttx_ld_prefix "bootloader")
	set(nuttx_startup_files ${nuttx_export_dir}/startup/${nuttx_startup_files})
	set(nuttx_bootloader_wrapers "-Wl,-wrap,sched_process_timer -Wl,-wrap,sem_post -Wl,-wrap,sem_wait")

	add_custom_command(OUTPUT
		${nuttx_startup_files}
		DEPENDS nuttx_export_${BOARD})

	set_source_files_properties(
		${nuttx_startup_files}
		PROPERTIES
		EXTERNAL_OBJECT true
		GENERATED true)

	add_library(startup
		${nuttx_startup_files})
	set_target_properties(startup
		PROPERTIES
		LINKER_LANGUAGE C)
	list(APPEND startup_libs startup)

endif()

if(NOT ${BOARD} STREQUAL "sim")
	set(main_link_flags
		"-T${nuttx_export_dir}/build/${nuttx_ld_prefix}ld.script"
		"-Wl,-Map=${PX4_BINARY_DIR}/${BOARD}/main.map"
		)
	px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
	set_target_properties(firmware_nuttx PROPERTIES LINK_FLAGS ${main_link_flags})
endif()

target_link_libraries(firmware_nuttx
	-Wl,--warn-common
	-Wl,--gc-sections
	-Wl,--start-group
	${startup_libs}
	${module_libraries}
	${df_driver_libs}
	${config_extra_libs}
	${nuttx_bootloader_wrapers}
	${link_libs}
	-Wl,--end-group)

add_custom_target(check_weak
	COMMAND ${NM} firmware_nuttx | ${GREP} " w " | cat
	DEPENDS firmware_nuttx
	VERBATIM
	)

if(NOT ${BOARD} STREQUAL "sim")

	if ("${romfs_used}" STREQUAL "y")

		if (config_io_board)
			set(extras "${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}.bin")
		endif()

		message(STATUS "Adding ROMFS on ${BOARD}")
		px4_nuttx_add_romfs(OUT romfs
			ROOT ROMFS/${config_romfs_root}
			EXTRAS ${extras}
			)
		if (config_io_board)
			add_dependencies(romfs fw_io)
		endif()
	endif()

	set(fw_file
		${CMAKE_CURRENT_BINARY_DIR}/${BOARD}_${LABEL}.px4)

#
# Bootloaders do not need .px4 or xml
#
	if(NOT "${config_nuttx_config}" STREQUAL "bootloader")
		px4_nuttx_add_firmware(OUT ${fw_file}
			BOARD ${BOARD}
			EXE ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
			PARAM_XML ${PX4_BINARY_DIR}/parameters.xml
			AIRFRAMES_XML ${PX4_BINARY_DIR}/airframes.xml
			)
	endif()

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
			${PX4_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_io_tui
		COMMAND ${GDBTUI}
			${PX4_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	add_custom_target(debug_io_ddd
		COMMAND ${DDD} --debugger ${GDB}
			${PX4_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
		DEPENDS firmware_nuttx
			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
		)

	if(NOT "${config_nuttx_config}" STREQUAL "bootloader")
		if ("${BOARD}" STREQUAL "aerofc-v1")
			px4_add_upload_aero(OUT upload OS ${OS} BOARD ${BOARD}
				BUNDLE ${fw_file})
		else()
			px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD}
				BUNDLE ${fw_file})
		endif()
	endif()
endif()

if(NOT "${config_nuttx_config}" STREQUAL "bootloader")
	install(FILES ${fw_file} DESTINATION .)
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
