#=============================================================================
# module subdirectories, need to include first
#
add_subdirectory(./lib)
add_subdirectory(./drivers)
add_subdirectory(./platforms)
add_subdirectory(./systemcmds)
add_subdirectory(./examples)
add_subdirectory(./modules)
add_subdirectory(./firmware)
#=============================================================================
# executable
#

function(px4_mangle_name dirname newname)
	set(tmp)
	string(REPLACE "/" "__" tmp ${dirname})
	set(${newname} ${tmp} PARENT_SCOPE)
endfunction()

if (${OS} STREQUAL "nuttx")

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
		ROOT ROMFS/px4fmu_common)

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

elseif(${OS} STREQUAL "qurt")
	set(module_dir_list)
	set(module_list)
	px4_qurt_add_modules(module_dir_list ${BOARD})
	message(STATUS "module list: ${module_dir_list}")

	foreach(directory ${module_dir_list})
		message(STATUS "directory: ${directory}")
		px4_mangle_name(${directory} mangled_name)
		list(APPEND module_list
			${mangled_name})
	endforeach()
	px4_qurt_generate_builtin_commands(
		OUT builtin_commands.cpp
		MODULE_LIST ${module_list})

	# FIXME @jgoppert - how to work around issues like this?
        # Without changing global variables?
        # Clear -rdynamic flag which fails for hexagon
        set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
        set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

	add_executable(main builtin_commands.cpp)
	target_link_libraries(main
		-Wl,--start-group
		${module_list}
		apps nuttx nosys m gcc
		-Wl,--end-group)
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
