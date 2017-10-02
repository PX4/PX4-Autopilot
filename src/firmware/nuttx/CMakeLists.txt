include(common/px4_upload)

# add executable
set(fw_name ${CONFIG}.elf)
add_executable(${fw_name} ${PX4_SOURCE_DIR}/src/platforms/empty.c)
add_dependencies(${fw_name} git_nuttx nuttx_build)

get_property(module_libraries GLOBAL PROPERTY PX4_LIBRARIES)

# build NuttX
add_subdirectory(${PX4_SOURCE_DIR}/platforms/nuttx/NuttX ${PX4_BINARY_DIR}/NuttX)

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

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	target_link_libraries(${fw_name} romfs)
	if (config_io_board)
		add_dependencies(romfs copy_px4io_bin)
	endif()
endif()

# create px4 file (combined firmware and metadata)
set(fw_file ${PX4_BINARY_DIR}/${BOARD}_${LABEL}.px4)

add_custom_command(OUTPUT ${BOARD}.bin
	COMMAND ${OBJCOPY} -O binary ${PX4_BINARY_DIR}/${fw_name} ${BOARD}.bin
	DEPENDS ${fw_name}
	)

if (TARGET parameters_xml AND TARGET airframes_xml)
	add_custom_command(OUTPUT ${fw_file}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${PX4_SOURCE_DIR}/Images/${BOARD}.prototype
			--git_identity ${PX4_SOURCE_DIR}
			--parameter_xml ${PX4_BINARY_DIR}/parameters.xml
			--airframe_xml ${PX4_BINARY_DIR}/airframes.xml
			--image ${BOARD}.bin > ${fw_file}
		DEPENDS ${BOARD}.bin parameters_xml airframes_xml
		COMMENT "Creating ${fw_file}"
		)

	add_custom_target(px4 ALL DEPENDS ${fw_file})

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
