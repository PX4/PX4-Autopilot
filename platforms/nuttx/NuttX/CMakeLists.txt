cmake_minimum_required(VERSION 3.2)

if(NOT BOARD)
	message(FATAL_ERROR "BOARD must be set (eg px4fmu-v2)")
endif()

if(NOT nuttx_config_type)
	# default to nsh if not specified
	set(nuttx_config_type "nsh")
endif()

project(NuttX_${BOARD} LANGUAGES ASM C CXX)
message(STATUS "NuttX: " ${BOARD} " " ${nuttx_config_type} " " ${CMAKE_SYSTEM_PROCESSOR})

if (CMAKE_HOST_APPLE OR CMAKE_HOST_WIN32)
	# copy with rsync and create file dependencies
	set(cp_cmd "rsync")
	set(cp_opts)
	list(APPEND cp_opts
		-rp
		--inplace
	)
else()
	# copy with hard links
	# archive, recursive, force, link (hardlinks)
	set(cp_cmd "cp")
	set(cp_opts "-aRfl")
endif()

file(GLOB_RECURSE copy_nuttx_files
	LIST_DIRECTORIES false
	${CMAKE_CURRENT_SOURCE_DIR}/nuttx/*)

file(GLOB_RECURSE copy_apps_files
	LIST_DIRECTORIES false
	${CMAKE_CURRENT_SOURCE_DIR}/apps/*)

# copy nuttx to build directory
add_custom_command(OUTPUT nuttx_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CMAKE_CURRENT_SOURCE_DIR}/nuttx ${CMAKE_CURRENT_BINARY_DIR}
	COMMAND cmake -E touch nuttx_copy.stamp
	DEPENDS ${copy_nuttx_files}
	COMMENT "Copying NuttX/nuttx to ${CMAKE_CURRENT_BINARY_DIR}"
	)

set(NUTTX_DIR ${CMAKE_CURRENT_BINARY_DIR}/nuttx)
set(NUTTX_CONFIG_DIR ${PX4_SOURCE_DIR}/nuttx-configs)

# copy apps to build directory
add_custom_command(OUTPUT apps_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CMAKE_CURRENT_SOURCE_DIR}/apps ${CMAKE_CURRENT_BINARY_DIR}
	COMMAND cmake -E touch apps_copy.stamp
	DEPENDS ${copy_apps_files}
	COMMENT "Copying NuttX/apps to ${CMAKE_CURRENT_BINARY_DIR}"
	)
set(APPS_DIR ${PX4_BINARY_DIR}/NuttX/apps)

# copy PX4 board config into nuttx
file(GLOB_RECURSE board_config_files ${NUTTX_CONFIG_DIR}/${BOARD})
add_custom_command(OUTPUT
					${NUTTX_DIR}/PX4_Config.mk
					${NUTTX_DIR}/PX4_Warnings.mk
					${NUTTX_DIR}/.config
					${NUTTX_DIR}/Make.defs
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/PX4_Config.mk ${NUTTX_DIR}/PX4_Config.mk
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/PX4_Warnings.mk ${NUTTX_DIR}/PX4_Warnings.mk
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/Make.defs ${NUTTX_DIR}/Make.defs
	COMMAND ${cp_cmd} ${cp_opts} ${NUTTX_CONFIG_DIR}/${BOARD} ${NUTTX_DIR}/configs/
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/include/nuttx/config.h
	DEPENDS
		${NUTTX_CONFIG_DIR}/PX4_Config.mk
		${NUTTX_CONFIG_DIR}/PX4_Warnings.mk
		${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
		${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/Make.defs
		${board_config_files}
		nuttx_copy.stamp
		apps_copy.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}/tools
	COMMENT "Copying NuttX config ${BOARD} and configuring"
	)
add_custom_target(nuttx_configure DEPENDS ${NUTTX_DIR}/.config)

# verbose build settings (V=1 or VERBOSE=1)
option(PX4_NUTTX_VERBOSE "PX4 NuttX verbose build" off)

if (($ENV{V} MATCHES "1") OR ($ENV{VERBOSE} MATCHES "1"))
	message(STATUS "NuttX verbose build enabled")
	set(PX4_NUTTX_VERBOSE on)
endif()

if (PX4_NUTTX_VERBOSE)
	set(nuttx_build_options)
	set(nuttx_build_output)
	set(nuttx_build_uses_terminal "USES_TERMINAL")
else()
	set(nuttx_build_options "--quiet")
	set(nuttx_build_output ">nuttx_build.log")
	set(nuttx_build_uses_terminal)
endif()

# context
add_custom_command(OUTPUT ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h
	COMMAND make ${nuttx_build_options} --no-print-directory context ${nuttx_build_output}
	DEPENDS nuttx_configure ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	${nuttx_build_uses_terminal}
	)
add_custom_target(nuttx_context DEPENDS ${NUTTX_DIR}/include/nuttx/version.h)

# library of NuttX libraries
add_library(nuttx_build INTERFACE)
add_dependencies(nuttx_build nuttx_context)

# builtins
add_custom_target(nuttx_builtins)
if (CONFIG_NSH_LIBRARY)
	# add additional commands to nuttx builtins
	set(builtin_registry ${APPS_DIR}/builtin/registry)
	foreach(module ${module_libraries})
		get_target_property(MAIN ${module} MAIN)
		get_target_property(STACK_MAIN ${module} STACK_MAIN)
		get_target_property(PRIORITY ${module} PRIORITY)

		if(MAIN)
			add_custom_command(OUTPUT ${builtin_registry}/${MAIN}_main.bdat
				COMMAND echo "{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main }," > ${builtin_registry}/${MAIN}_main.bdat
				DEPENDS nuttx_context
				VERBATIM
				)

			add_custom_command(OUTPUT ${builtin_registry}/${MAIN}_main.pdat
				COMMAND echo "int ${MAIN}_main(int argc, char *argv[]);" > ${builtin_registry}/${MAIN}_main.pdat
				DEPENDS nuttx_context
				VERBATIM
				)

			add_custom_target(nuttx_builtins_${MAIN} DEPENDS ${builtin_registry}/${MAIN}_main.bdat ${builtin_registry}/${MAIN}_main.pdat)
			add_dependencies(nuttx_builtins nuttx_builtins_${MAIN})
		endif()
	endforeach()
endif()
add_dependencies(nuttx_build nuttx_builtins)

# APPS

# libapps.a
add_custom_command(OUTPUT ${APPS_DIR}/libapps.a ${APPS_DIR}/platform/.built
	COMMAND find ${APPS_DIR} -name \*.o -delete
	COMMAND make ${nuttx_build_options} --no-print-directory -C ../apps TOPDIR="${NUTTX_DIR}" libapps.a ${nuttx_build_output}
	DEPENDS nuttx_context nuttx_builtins
	WORKING_DIRECTORY ${NUTTX_DIR}
	${nuttx_build_uses_terminal}
	)
add_custom_target(nuttx_apps_build DEPENDS ${APPS_DIR}/libapps.a)
add_library(nuttx_apps STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_apps PROPERTY IMPORTED_LOCATION ${APPS_DIR}/libapps.a)
add_dependencies(nuttx_build nuttx_apps_build)
target_link_libraries(nuttx_build INTERFACE nuttx_apps)

# helper for all targets
function(add_nuttx_dir nuttx_lib nuttx_lib_dir kernel extra)
	file(GLOB_RECURSE nuttx_lib_files
		LIST_DIRECTORIES false
		${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*)

	add_custom_command(OUTPUT ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND find ${nuttx_lib_dir} -type f -name *.o -delete
		COMMAND make -C ${nuttx_lib_dir} ${nuttx_build_options} --no-print-directory lib${nuttx_lib}.a TOPDIR=${NUTTX_DIR} KERNEL=${kernel} EXTRADEFINES=${extra} ${nuttx_build_output}
		DEPENDS ${nuttx_lib_files} nuttx_context
		WORKING_DIRECTORY ${NUTTX_DIR}
		${nuttx_build_uses_terminal}
		)
	add_custom_target(nuttx_${nuttx_lib}_build DEPENDS ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_library(nuttx_${nuttx_lib} STATIC IMPORTED GLOBAL)
	set_property(TARGET nuttx_${nuttx_lib} PROPERTY IMPORTED_LOCATION ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_dependencies(nuttx_build nuttx_${nuttx_lib}_build)
	target_link_libraries(nuttx_build INTERFACE nuttx_${nuttx_lib})
endfunction()

# add_nuttx_dir(NAME DIRECTORY KERNEL EXTRA)
add_nuttx_dir(arch arch/arm/src y -D__KERNEL__)
add_nuttx_dir(binfmt binfmt y -D__KERNEL__)
add_nuttx_dir(configs configs y -D__KERNEL__)
add_nuttx_dir(drivers drivers y -D__KERNEL__)
add_nuttx_dir(fs fs y -D__KERNEL__)
add_nuttx_dir(sched sched y -D__KERNEL__)
add_nuttx_dir(c libc n "")
add_nuttx_dir(cxx libxx n "")
add_nuttx_dir(mm mm n "")

# oldconfig helper
add_custom_target(oldconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${BOARD} oldconfig
	COMMAND cp ${NUTTX_DIR}/.config ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
	COMMAND ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/tools/nuttx_defconf_tool.sh ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
	DEPENDS nuttx_configure
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${BOARD} with ${nuttx_config_type}"
	USES_TERMINAL
	)

# menuconfig helper
add_custom_target(menuconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${BOARD} menuconfig
	COMMAND cp ${NUTTX_DIR}/.config ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
	COMMAND cd ${PX4_SOURCE_DIR} && platforms/nuttx/NuttX/tools/nuttx_defconf_tool.sh ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
	DEPENDS nuttx_configure
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${BOARD} with ${nuttx_config_type}"
	USES_TERMINAL
	)

# qconfig helper
add_custom_target(qconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${BOARD} qconfig
	COMMAND cp .config ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
	DEPENDS nuttx_configure
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make qconfig for ${BOARD} with ${nuttx_config_type}"
	USES_TERMINAL
	)
