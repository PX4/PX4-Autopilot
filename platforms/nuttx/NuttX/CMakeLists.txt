cmake_minimum_required(VERSION 3.2)

px4_add_git_submodule(TARGET git_nuttx PATH "nuttx")
px4_add_git_submodule(TARGET git_nuttx_apps PATH "apps")

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

set(NUTTX_DIR ${CMAKE_CURRENT_BINARY_DIR}/nuttx)
set(NUTTX_CONFIG_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/nuttx-configs)
configure_file(${NUTTX_CONFIG_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs)

# copy nuttx to build directory
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/nuttx)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
	DEPENDS ${copy_nuttx_files} git_nuttx
	COMMENT "Copying NuttX/nuttx to ${CMAKE_CURRENT_BINARY_DIR}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)

# copy apps to build directory
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/apps)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
	DEPENDS ${copy_apps_files} git_nuttx_apps
	COMMENT "Copying NuttX/apps to ${CMAKE_CURRENT_BINARY_DIR}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
set(APPS_DIR ${CMAKE_CURRENT_BINARY_DIR}/apps)

# copy PX4 board config into nuttx
file(GLOB_RECURSE board_config_files ${NUTTX_CONFIG_DIR}/${BOARD})
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_CONFIG_DIR}/${BOARD})
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${NUTTX_DIR}/configs)
add_custom_command(
	OUTPUT ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/include/nuttx/config.h
	COMMAND ${CMAKE_COMMAND} -E make_directory ${NUTTX_DIR}/configs/${BOARD}/src
	COMMAND cd ${CMAKE_SOURCE_DIR} && ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	DEPENDS
		${NUTTX_CONFIG_DIR}/${BOARD}/${nuttx_config_type}/defconfig
		${board_config_files}
		${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
		${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
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
set(nuttx_builtin_list)
if (CONFIG_NSH_LIBRARY)
	# force builtins regeneration and apps rebuild if nuttx or px4 configuration have changed
	add_custom_command(OUTPUT builtins_clean.stamp
		COMMAND find ${APPS_DIR}/builtin/registry -name px4_\*.bdat -delete
		COMMAND find ${APPS_DIR}/builtin/registry -name px4_\*.pdat -delete
		COMMAND rm -f ${APPS_DIR}/builtin/builtin_list.h
		COMMAND ${CMAKE_COMMAND} -E touch builtins_clean.stamp
		DEPENDS
			nuttx_configure
			nuttx_context
			${PX4_SOURCE_DIR}/cmake/configs/${CONFIG}.cmake
		)

	foreach(module ${module_libraries})
		get_target_property(MAIN ${module} MAIN)
		get_target_property(STACK_MAIN ${module} STACK_MAIN)
		get_target_property(PRIORITY ${module} PRIORITY)

		if(MAIN)
			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND echo "{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main }," > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS nuttx_context builtins_clean.stamp
				VERBATIM
				)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat)

			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND echo "int ${MAIN}_main(int argc, char *argv[]);" > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS nuttx_context builtins_clean.stamp
				VERBATIM
				)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat)

		endif()
	endforeach()
endif()

# APPS

# libapps.a
add_custom_command(OUTPUT ${APPS_DIR}/libapps.a ${APPS_DIR}/platform/.built
	COMMAND find ${APPS_DIR} -name \*.o -delete
	COMMAND make ${nuttx_build_options} --no-print-directory -C ../apps TOPDIR="${NUTTX_DIR}" libapps.a ${nuttx_build_output}
	DEPENDS nuttx_context ${nuttx_builtin_list}
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
		COMMAND make -C ${nuttx_lib_dir} ${nuttx_build_options} --no-print-directory all TOPDIR=${NUTTX_DIR} KERNEL=${kernel} EXTRADEFINES=${extra} ${nuttx_build_output}
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

if (CONFIG_NET)
	add_nuttx_dir(net net y -D__KERNEL__)
endif()

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
