############################################################################
#
#   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config)
set(APPS_DIR ${NUTTX_SRC_DIR}/apps)

###############################################################################
# NuttX build
###############################################################################

configure_file(${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/Make.defs.in ${CMAKE_CURRENT_BINARY_DIR}/nuttx/Make.defs)

# inflate .config
add_custom_command(
	OUTPUT ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/nuttx/Make.defs ${NUTTX_DIR}/Make.defs
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/defconfig
	COMMAND ${NUTTX_SRC_DIR}/tools/px4_nuttx_make_olddefconfig.sh > ${CMAKE_CURRENT_BINARY_DIR}/nuttx_olddefconfig.log
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${CMAKE_CURRENT_BINARY_DIR}/nuttx/.config
	DEPENDS
		${NUTTX_DEFCONFIG}
		${NUTTX_DIR}/defconfig
		${CMAKE_CURRENT_BINARY_DIR}/nuttx/Make.defs
	WORKING_DIRECTORY ${NUTTX_DIR}
	#USES_TERMINAL
)

# context (.config -> include/nuttx/config.h, include/nuttx/version.h, dirlinks)
add_custom_command(
	OUTPUT ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/arch/chip
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/nuttx/.config ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/nuttx/Make.defs ${NUTTX_DIR}/Make.defs
	COMMAND make --no-print-directory --silent clean_context
	COMMAND make --no-print-directory --silent pass1dep > ${CMAKE_CURRENT_BINARY_DIR}/nuttx_context.log
	DEPENDS
		${CMAKE_CURRENT_BINARY_DIR}/nuttx/Make.defs
		${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	#USES_TERMINAL
)
add_custom_target(nuttx_context DEPENDS ${NUTTX_DIR}/include/nuttx/config.h)


# builtins
set(builtin_apps_string)
set(builtin_apps_decl_string)
if(CONFIG_NSH_LIBRARY)
	list(SORT module_libraries)
	foreach(module ${module_libraries})
		get_target_property(MAIN ${module} MAIN)
		get_target_property(STACK_MAIN ${module} STACK_MAIN)
		get_target_property(PRIORITY ${module} PRIORITY)

		if(MAIN)
			set(builtin_apps_string "${builtin_apps_string}{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main },\n")
			set(builtin_apps_decl_string "${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")
		endif()
	endforeach()
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/px4.bdat.in ${CMAKE_CURRENT_BINARY_DIR}/px4.bdat)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/px4.pdat.in ${CMAKE_CURRENT_BINARY_DIR}/px4.pdat)

# APPS

# libapps.a
file(GLOB_RECURSE nuttx_apps_files LIST_DIRECTORIES false
	${APPS_DIR}/*.c
	${APPS_DIR}/*.h
)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a
	COMMAND ${CMAKE_COMMAND} -E remove -f ${APPS_DIR}/libapps.a ${APPS_DIR}/builtin/builtin_list.h ${APPS_DIR}/builtin/builtin_proto.h
	COMMAND find ${APPS_DIR} -type f -name \*.o -delete
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/px4.bdat ${APPS_DIR}/builtin/registry/px4.bdat
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/px4.pdat ${APPS_DIR}/builtin/registry/px4.pdat
	COMMAND ${CMAKE_COMMAND} -E touch_nocreate ${APPS_DIR}/builtin/registry/.updated
	COMMAND make --no-print-directory --silent TOPDIR="${NUTTX_DIR}" > ${CMAKE_CURRENT_BINARY_DIR}/nuttx_apps.log
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${APPS_DIR}/libapps.a ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a
	DEPENDS ${nuttx_apps_files} nuttx_context ${NUTTX_DIR}/include/nuttx/config.h
	WORKING_DIRECTORY ${APPS_DIR}
	#USES_TERMINAL
)
add_custom_target(nuttx_apps_build DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a)
add_library(nuttx_apps STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_apps PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a)
add_dependencies(nuttx_apps nuttx_apps_build)

# helper for all targets
function(add_nuttx_dir nuttx_lib nuttx_lib_dir kernel extra)
	file(GLOB_RECURSE nuttx_lib_files LIST_DIRECTORIES false
		${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*.h
	)

	add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND find ${nuttx_lib_dir} -type f -name \*.o -delete
		COMMAND make -C ${nuttx_lib_dir} --no-print-directory --silent all TOPDIR="${NUTTX_DIR}" KERNEL=${kernel} EXTRAFLAGS=${extra}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a
		DEPENDS
			${nuttx_lib_files}
			nuttx_context ${NUTTX_DIR}/include/nuttx/config.h
		WORKING_DIRECTORY ${NUTTX_DIR}
		#USES_TERMINAL
	)
	add_custom_target(nuttx_${nuttx_lib}_build DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a)

	add_library(nuttx_${nuttx_lib} STATIC IMPORTED GLOBAL)
	set_property(TARGET nuttx_${nuttx_lib} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_dependencies(nuttx_${nuttx_lib} nuttx_${nuttx_lib}_build)
endfunction()

# add_nuttx_dir(NAME DIRECTORY KERNEL EXTRA)
add_nuttx_dir(arch arch/${CONFIG_ARCH}/src y -D__KERNEL__)
add_nuttx_dir(binfmt binfmt y -D__KERNEL__)
add_nuttx_dir(boards boards y -D__KERNEL__)
add_nuttx_dir(drivers drivers y -D__KERNEL__)
add_nuttx_dir(fs fs y -D__KERNEL__)
add_nuttx_dir(sched sched y -D__KERNEL__)
add_nuttx_dir(c libs/libc n "")
add_nuttx_dir(xx libs/libxx n "")
add_nuttx_dir(mm mm n "")
add_nuttx_dir(crypto crypto y -D__KERNEL__)

if(CONFIG_NET)
	add_nuttx_dir(net net y -D__KERNEL__)
endif()

###############################################################################
# NuttX oldconfig: Update a configuration using a provided .config as base
add_custom_target(oldconfig_nuttx
	COMMAND make --no-print-directory --silent oldconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + savedefconfig back to PX4
add_custom_target(oldconfig
	COMMAND make --no-print-directory --silent savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS oldconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make oldconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX olddefconfig: same as oldconfig, but additionally update deps and sets new symbols to their default value
add_custom_target(olddefconfig_nuttx
	COMMAND make --no-print-directory --silent olddefconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make olddefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX olddefconfig + savedefconfig back to PX4
add_custom_target(olddefconfig
	COMMAND make --no-print-directory --silent savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS olddefconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make olddefconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX menuconfig
add_custom_target(menuconfig_nuttx
	COMMAND make --no-print-directory --silent menuconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX menuconfig + savedefconfig back to PX4
add_custom_target(menuconfig
	COMMAND make --no-print-directory --silent savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS menuconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make nuttx_menuconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX qconfig
add_custom_target(qconfig_nuttx
	COMMAND make --no-print-directory --silent qconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make qconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX qconfig + savedefconfig back to PX4
add_custom_target(qconfig
	COMMAND make --no-print-directory --silent savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS qconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make qconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)
