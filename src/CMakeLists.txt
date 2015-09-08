#=============================================================================
# module subdirectories, need to include first
#
add_subdirectory(./lib)
add_subdirectory(./drivers)
add_subdirectory(./platforms)
add_subdirectory(./systemcmds)
add_subdirectory(./examples)
add_subdirectory(./modules)

#=============================================================================
# executable
#

# a list of modules that will be linked to main
set(module_list)

if (${LABEL} STREQUAL "simple")
	list(APPEND module_list
		drivers__led
		drivers__device
		platforms__common
		modules__systemlib
		modules__uORB
		examples__px4_simple_app
		lib__mathlib__math__filter
		lib__conversion
	)
endif()

if (${OS} STREQUAL "nuttx")

	if (${LABEL} STREQUAL "simple")
		list(APPEND module_list ${module_list_simple})
	endif()

	list(APPEND module_list
		platforms__nuttx
		platforms__nuttx__px4_layer
		drivers__boards__px4fmu-v2
		drivers__stm32
	)

	px4_nuttx_generate_builtin_commands(
		OUT builtin_commands.c
		MODULE_LIST ${module_list})

	px4_nuttx_generate_romfs(OUT romfs.o
		ROOT ${CMAKE_SOURCE_DIR}/ROMFS/px4fmu_common)

	# add executable
	add_executable(main builtin_commands.c romfs.o)
	set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)
	set(main_link_flags
		"-T${nuttx_export_dir}/build/ld.script"
		"-Wl,-Map=${CMAKE_BINARY_DIR}/main.map"
		)
	px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
	set_target_properties(main PROPERTIES LINK_FLAGS ${main_link_flags})

	target_link_libraries(main
		-Wl,--start-group
		${module_list}
		apps nuttx nosys m gcc
		-Wl,--end-group)

	px4_nuttx_add_firmware(OUT ${CMAKE_CURRENT_BINARY_DIR}/fw_main.px4
		EXE ${CMAKE_CURRENT_BINARY_DIR}/main)

	px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD}
		BUNDLE ${CMAKE_CURRENT_BINARY_DIR}/fw_main.px4)

endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
