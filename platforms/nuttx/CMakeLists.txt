include(common/px4_upload)
include(cygwin_cygpath)

if (NOT FW_NAME)
	set(FW_NAME ${CONFIG}.elf)
endif()

add_executable(${FW_NAME} ${PX4_SOURCE_DIR}/src/platforms/empty.c)
add_dependencies(${FW_NAME} git_nuttx nuttx_build)

get_property(module_libraries GLOBAL PROPERTY PX4_LIBRARIES)

# build NuttX
add_subdirectory(NuttX ${PX4_BINARY_DIR}/NuttX)

set(nuttx_libs)
list(APPEND nuttx_libs
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
	)

if (CONFIG_NET)
	list(APPEND nuttx_libs nuttx_net)
endif()

if (NOT LD_SCRIPT)
	set(LD_SCRIPT ld.script)
endif()

file(RELATIVE_PATH PX4_BINARY_DIR_REL ${CMAKE_CURRENT_BINARY_DIR} ${PX4_BINARY_DIR})

# only in the cygwin environment: convert absolute linker script path to mixed windows (C:/...)
# because even relative linker script paths are different for linux, mac and windows
CYGPATH(PX4_BINARY_DIR PX4_BINARY_DIR_CYG)

target_link_libraries(${FW_NAME}

	-Wl,--script=${PX4_BINARY_DIR_CYG}/NuttX/nuttx/configs/${BOARD}/scripts/${LD_SCRIPT}
	-Wl,-Map=${CONFIG}.map
	-Wl,--warn-common,--gc-sections

	-Wl,--start-group
		${module_libraries}
		${nuttx_libs}
	-Wl,--end-group

	m
	gcc
	)

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	target_link_libraries(${FW_NAME} romfs)
	if (config_io_board)
		add_dependencies(romfs copy_px4io_bin)
	endif()
endif()

# create px4 file (combined firmware and metadata)
set(fw_file ${PX4_BINARY_DIR}/${FW_NAME})
string(REPLACE ".elf" ".px4" fw_file ${fw_file})
string(REPLACE "nuttx_" "" fw_file ${fw_file})

add_custom_command(OUTPUT ${BOARD}.bin
	COMMAND ${OBJCOPY} -O binary ${PX4_BINARY_DIR_REL}/${FW_NAME} ${BOARD}.bin
	DEPENDS ${FW_NAME}
	)

if (NOT FW_PROTOTYPE)
	set(FW_PROTOTYPE ${BOARD})
endif()

if (TARGET parameters_xml AND TARGET airframes_xml)
	add_custom_command(OUTPUT ${fw_file}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${PX4_SOURCE_DIR}/platforms/nuttx/Images/${FW_PROTOTYPE}.prototype
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
	COMMAND size ${FW_NAME}
	DEPENDS ${FW_NAME}
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)

# print weak symbols
add_custom_target(check_weak
	COMMAND ${NM} ${FW_NAME} | ${GREP} " w " | cat
	DEPENDS ${FW_NAME}
	VERBATIM
	)

# debugger helpers
configure_file(gdbinit.in .gdbinit)

add_custom_target(debug
	COMMAND ${GDB} $<TARGET_FILE:${FW_NAME}>
	DEPENDS ${FW_NAME} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_tui
	COMMAND ${GDBTUI} $<TARGET_FILE:${FW_NAME}>
	DEPENDS ${FW_NAME} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_ddd
	COMMAND ${DDD} --debugger ${GDB} $<TARGET_FILE:${FW_NAME}>
	DEPENDS ${FW_NAME} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io
	COMMAND ${GDB} ${fw_io_path}
	DEPENDS ${FW_NAME} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io_tui
	COMMAND ${GDBTUI} ${fw_io_path}
	DEPENDS ${FW_NAME} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io_ddd
	COMMAND ${DDD} --debugger ${GDB} ${fw_io_path}}
	DEPENDS ${fw_io} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)
