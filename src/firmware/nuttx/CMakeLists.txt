include(common/px4_upload)

# add executable
set(fw_name ${CONFIG}.elf)
add_executable(${fw_name} ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD}/src/empty.c)

get_property(module_libraries GLOBAL PROPERTY PX4_LIBRARIES)

# build NuttX
add_subdirectory(${PX4_SOURCE_DIR}/platforms/nuttx/NuttX ${PX4_BINARY_DIR}/NuttX)

# Bootloaders use a different ld file, a separate startup file
# and need to override the os version of sched_process_timer
if ("${nuttx_config_type}" STREQUAL "bootloader")
	target_link_libraries(${fw_name}
		-T${PX4_BINARY_DIR}/NuttX/nuttx/configs/${BOARD}/scripts/bootloaderld.script
		-Wl,-Map=${PX4_BINARY_DIR}/${CONFIG}.map
		-Wl,-wrap,sched_process_timer -Wl,-wrap,sem_post -Wl,-wrap,sem_wait
		-Wl,--warn-common
		-Wl,--gc-sections
		-Wl,--start-group
		${module_libraries}
		nuttx_arch
		nuttx_board
		nuttx_c
		nuttx_sched
		nuttx_startup
		-Wl,--end-group
		)
else()
	target_link_libraries(${fw_name}
		-T${PX4_BINARY_DIR}/NuttX/nuttx/configs/${BOARD}/scripts/ld.script
		-Wl,-Map=${PX4_BINARY_DIR}/${CONFIG}.map
		-Wl,--warn-common
		-Wl,--gc-sections
		-Wl,--start-group
		${module_libraries}
		nuttx_apps
		nuttx_arch
		nuttx_binfmt
		nuttx_binfmt
		nuttx_board
		nuttx_c
		nuttx_configs
		nuttx_cxx
		nuttx_drivers
		nuttx_fs
		nuttx_mm
		nuttx_sched
		-Wl,--end-group
		m
		gcc
		)
endif()

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	target_link_libraries(${fw_name} romfs)
	if (config_io_board)
		add_dependencies(romfs copy_px4io_bin)
	endif()
endif()

# create px4 file (combined firmware and metadata)
#
# Bootloaders do not need .px4 or xml
if("${nuttx_config_type}" STREQUAL "nsh" AND NOT ${CONFIG} MATCHES "px4io")
	set(fw_file ${PX4_BINARY_DIR}/${CONFIG}.px4)

	add_custom_command(OUTPUT ${BOARD}.bin
		COMMAND ${OBJCOPY} -O binary ${PX4_BINARY_DIR}/${fw_name} ${BOARD}.bin
		DEPENDS ${fw_name}
		)

	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/${CONFIG}.px4
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${PX4_SOURCE_DIR}/Images/${BOARD}.prototype
			--git_identity ${PX4_SOURCE_DIR}
			--parameter_xml ${PX4_BINARY_DIR}/parameters.xml
			--airframe_xml ${PX4_BINARY_DIR}/airframes.xml
			--image ${BOARD}.bin > ${fw_file}
		DEPENDS ${BOARD}.bin parameters_xml airframes_xml
		COMMENT "Creating ${CONFIG}.px4"
		)

	add_custom_target(px4 ALL DEPENDS ${PX4_BINARY_DIR}/${CONFIG}.px4)

	# upload helper
	if (${BOARD} STREQUAL "aerofc-v1")
		px4_add_upload_aero(OUT upload OS ${OS} BOARD ${BOARD} BUNDLE ${fw_file})
	else()
		px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD} BUNDLE ${fw_file})
	endif()
endif()

# print size
add_custom_target(size
	COMMAND size ${fw_name}
	DEPENDS ${fw_name}
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)

# print weak symbols
add_custom_target(check_weak
	COMMAND ${NM} ${fw_name} | ${GREP} " w " | cat
	DEPENDS ${fw_name}
	VERBATIM
	)

# debugger helpers
configure_file(gdbinit.in .gdbinit)

add_custom_target(debug
	COMMAND ${GDB} $<TARGET_FILE:${fw_name}>
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_tui
	COMMAND ${GDBTUI} $<TARGET_FILE:${fw_name}>
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_ddd
	COMMAND ${DDD} --debugger ${GDB} $<TARGET_FILE:${fw_name}>
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io
	COMMAND ${GDB} ${fw_io_path}
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io_tui
	COMMAND ${GDBTUI} ${fw_io_path}
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io_ddd
	COMMAND ${DDD} --debugger ${GDB} ${fw_io_path}}
	DEPENDS ${fw_io} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)
