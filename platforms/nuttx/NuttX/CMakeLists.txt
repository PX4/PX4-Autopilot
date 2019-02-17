
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

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config)

###############################################################################
# NuttX: copy to build directory
###############################################################################
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/nuttx)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)

# setup custom command to copy changes later
file(GLOB_RECURSE copy_nuttx_files LIST_DIRECTORIES false ${NUTTX_SRC_DIR}/nuttx/*)
add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
	DEPENDS ${copy_nuttx_files} git_nuttx
	COMMENT "Copying NuttX/nuttx to ${CP_DST}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

###############################################################################
# NuttX apps: copy to build directory
###############################################################################
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/apps)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)

# setup custom command to copy changes later
file(GLOB_RECURSE copy_apps_files LIST_DIRECTORIES false ${NUTTX_SRC_DIR}/apps/*)
add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
	DEPENDS ${copy_apps_files} git_nuttx_apps
	COMMENT "Copying NuttX/apps to ${CP_DST}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
set(APPS_DIR ${CMAKE_CURRENT_BINARY_DIR}/apps)


# If the board provides a Kconfig Use it or create an empty one
if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
	add_custom_command(
		OUTPUT ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/configs/dummy/Kconfig
		DEPENDS
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
			${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
		)
else()
	add_custom_command(
		OUTPUT ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/configs/dummy/Kconfig
		DEPENDS
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
			${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
		)
endif()

###############################################################################
# NuttX configure
###############################################################################

if ("x${config_expanded}" STREQUAL "x")
	# copy PX4 board Compressed config into nuttx and inflate it
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/.config
			${NUTTX_DIR}/arch/arm/include/math.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND kconfig-tweak --set-str CONFIG_APPS_DIR "../apps"
		COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/include/nuttx/config.h
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CONFIG_DIR}/ ${PX4_BINARY_DIR}/NuttX/nuttx-config
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/math.h ${NUTTX_DIR}/arch/arm/include/math.h # copy arm math.h into NuttX source
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
		COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y olddefconfig
		DEPENDS
			${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig
			${NUTTX_CONFIG_DIR}/include/board.h
			${NUTTX_CONFIG_DIR}/scripts/ld.script
			${NUTTX_SRC_DIR}/math.h
			${NUTTX_SRC_DIR}/nsh_romfsimg.h
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX config ${NUTTX_CONFIG} and configuring"
		)
else()
	# copy PX4 board config into nuttx
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/.config
			${NUTTX_DIR}/arch/arm/include/math.h
		COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig ${NUTTX_DIR}/.config
		COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/include/nuttx/config.h
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CONFIG_DIR}/ ${CMAKE_CURRENT_BINARY_DIR}/nuttx-config
		COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/nuttx-config/src
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/math.h ${NUTTX_DIR}/arch/arm/include/ # copy arm math.h into NuttX source
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/nsh_romfsimg.h ${CMAKE_CURRENT_BINARY_DIR}/nuttx-config/include/
		DEPENDS
			${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig
			${NUTTX_CONFIG_DIR}/include/board.h
			${NUTTX_CONFIG_DIR}/scripts/ld.script
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
			${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
			${NUTTX_DIR}/configs/dummy/Kconfig
			${CMAKE_CURRENT_SOURCE_DIR}/math.h
			${CMAKE_CURRENT_SOURCE_DIR}/nsh_romfsimg.h
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX uncompressed config ${NUTTX_CONFIG} and configuring"
		)
endif()

###############################################################################
# NuttX build
###############################################################################

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs)

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
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	${nuttx_build_uses_terminal}
	)
add_custom_target(nuttx_context DEPENDS ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h)

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
			${NUTTX_DIR}/.config
			${NUTTX_DIR}/include/nuttx/version.h
			${NUTTX_DIR}/include/nuttx/config.h
			${PX4_CONFIG_FILE}
		)

	foreach(module ${module_libraries})
		get_target_property(MAIN ${module} MAIN)
		get_target_property(STACK_MAIN ${module} STACK_MAIN)
		get_target_property(PRIORITY ${module} PRIORITY)

		if(MAIN)
			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND echo "{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main }," > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS builtins_clean.stamp ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h
				VERBATIM
				)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat)

			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND echo "int ${MAIN}_main(int argc, char *argv[]);" > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS builtins_clean.stamp ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h
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
	DEPENDS ${nuttx_builtin_list} ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h
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
		DEPENDS ${nuttx_lib_files} ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h
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
add_nuttx_dir(c libs/libc n "")
add_nuttx_dir(xx libs/libxx n "")
add_nuttx_dir(mm mm n "")

if (CONFIG_NET)
	add_nuttx_dir(net net y -D__KERNEL__)
endif()

# oldconfig helper
add_custom_target(oldconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${PX4_BOARD} oldconfig
	COMMAND cp ${NUTTX_DIR}/.config ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# olddefconfig helper
add_custom_target(olddefconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${PX4_BOARD} olddefconfig
	COMMAND cp ${NUTTX_DIR}/.config ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make olddefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# menuconfig helper
add_custom_target(menuconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${PX4_BOARD} menuconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# qconfig helper
add_custom_target(qconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${PX4_BOARD} qconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make qconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# savedefconfig helper
add_custom_target(savedefconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${PX4_BOARD} savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/defconfig ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig
	DEPENDS ${NUTTX_DIR}/.config
	COMMENT "Compressing .config and saving back to ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig"
	WORKING_DIRECTORY ${NUTTX_DIR}
)

# menuconfig and save
add_custom_target(menuconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces menuconfig to run before savedefconfig
	DEPENDS menuconfig
)

# oldconfig and save
add_custom_target(oldconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces oldconfig to run before savedefconfig
	DEPENDS oldconfig
)

# qconfig and save
add_custom_target(qconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces qconfig to run before savedefconfig
	DEPENDS qconfig
)
