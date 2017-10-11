############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

#=============================================================================
#
#	Defined functions in this file
#
# 	OS Specific Functions
#
#		* px4_posix_generate_builtin_commands
#
# 	Required OS Interface Functions
#
# 		* px4_os_add_flags
#		* px4_os_prebuild_targets
#

include(common/px4_base)

#=============================================================================
#
#	px4_posix_generate_builtin_commands
#
#	This function generates the builtin_commands.c src for posix
#
#	Usage:
#		px4_posix_generate_builtin_commands(
#			MODULE_LIST <in-list>
#			OUT <file>)
#
#	Input:
#		MODULE_LIST	: list of modules
#
#	Output:
#		OUT	: stem of generated apps.cpp/apps.h ("apps")
#
#	Example:
#		px4_posix_generate_builtin_commands(
#			OUT <generated-src> MODULE_LIST px4_simple_app)
#
function(px4_posix_generate_builtin_commands)
	px4_parse_function_args(
		NAME px4_posix_generate_builtin_commands
		ONE_VALUE OUT
		MULTI_VALUE MODULE_LIST
		REQUIRED MODULE_LIST OUT
		ARGN ${ARGN})

	set(builtin_apps_string)
	set(builtin_apps_decl_string)
	set(command_count 0)
	foreach(module ${MODULE_LIST})
		# default
		set(MAIN_DEFAULT MAIN-NOTFOUND)
		set(STACK_DEFAULT 1024)
		set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)
		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()
		if (MAIN)
			set(builtin_apps_string
				"${builtin_apps_string}\tapps[\"${MAIN}\"] = ${MAIN}_main;\n")
			set(builtin_apps_decl_string
				"${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")
			math(EXPR command_count "${command_count}+1")
		endif()
	endforeach()
	configure_file(${PX4_SOURCE_DIR}/src/platforms/apps.cpp.in ${OUT}.cpp)
	configure_file(${PX4_SOURCE_DIR}/src/platforms/apps.h.in ${OUT}.h)
endfunction()

#=============================================================================
#
#	px4_os_add_flags
#
#	Set the posix build flags.
#
#
function(px4_os_add_flags)

	include_directories(src/platforms/posix/include)

	# This block sets added_definitions and added_cxx_flags.
	if(UNIX AND APPLE)
		add_definitions(
			-D__PX4_POSIX
			-D__PX4_DARWIN
			-D__DF_DARWIN
			-Dnoreturn_function=__attribute__\(\(noreturn\)\)
			)

		if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
			message(FATAL_ERROR "PX4 Firmware requires XCode 8 or newer on Mac OS. Version installed on this system: ${CMAKE_CXX_COMPILER_VERSION}")
		endif()

		exec_program(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
		string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
		if (DARWIN_VERSION LESS 16)
			add_definitions(
				-DCLOCK_MONOTONIC=1
				-DCLOCK_REALTIME=0
				-D__PX4_APPLE_LEGACY
				)
		endif()
	else()
		add_definitions(
			-D__PX4_POSIX
			-D__PX4_LINUX
			-D__DF_LINUX
			-Dnoreturn_function=__attribute__\(\(noreturn\)\)
			)
	endif()

	if (${BOARD} STREQUAL "eagle")
		# TODO: move this to toolchain
		
		set(cpu_flags -mcpu=cortex-a9 -mfpu=neon-vfpv3 -mthumb-interwork -mfloat-abi=hard)

		# need to manually specify flags until compiler id is properly detected by the toolchain
		add_compile_options("$<$<COMPILE_LANGUAGE:C>:-std=gnu99>")
		add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++11>")

		if ("$ENV{HEXAGON_ARM_SYSROOT}" STREQUAL "")
			message(FATAL_ERROR "HEXAGON_ARM_SYSROOT not set")
		else()
			set(HEXAGON_ARM_SYSROOT $ENV{HEXAGON_ARM_SYSROOT})
		endif()

		set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

		# Add the toolchain specific flags
		add_compile_options(--sysroot=${HEXAGON_ARM_SYSROOT})

		link_libraries(
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/usr/lib
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib
			--sysroot=${HEXAGON_ARM_SYSROOT}
			)
	elseif (${BOARD} STREQUAL "excelsior")
		# TODO: move this to toolchain
		
		set(cpu_flags -mcpu=cortex-a9 -mfpu=neon-vfpv3 -mthumb-interwork -mfloat-abi=hard)

		# need to manually specify flags until compiler id is properly detected by the toolchain
		add_compile_options("$<$<COMPILE_LANGUAGE:C>:-std=gnu99>")
		add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++11>")

		if ("$ENV{HEXAGON_ARM_SYSROOT}" STREQUAL "")
			message(FATAL_ERROR "HEXAGON_ARM_SYSROOT not set")
		else()
			set(HEXAGON_ARM_SYSROOT $ENV{HEXAGON_ARM_SYSROOT})
		endif()

		set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

		# Add the toolchain specific flags
		add_compile_options(--sysroot=${HEXAGON_ARM_SYSROOT}/lib32-apq8096 -mfloat-abi=softfp -mfpu=neon -mthumb-interwork)

		link_libraries(
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib32-apq8096/usr/lib
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib32-apq8096/lib
			--sysroot=${HEXAGON_ARM_SYSROOT}/lib32-apq8096 -mfloat-abi=softfp -mfpu=neon -mthumb-interwork

			)
	elseif (${BOARD} STREQUAL "rpi")
		# TODO: move this to toolchain
		add_compile_options(-mcpu=cortex-a53 -mfpu=neon -mfloat-abi=hard)

		find_program(CXX_COMPILER_PATH ${CMAKE_CXX_COMPILER})
		get_filename_component(CXX_COMPILER_PATH ${CXX_COMPILER_PATH} DIRECTORY)
		get_filename_component(CXX_COMPILER_PATH "${CXX_COMPILER_PATH}/../" ABSOLUTE)

		if (${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
			add_compile_options(
				--target=arm-pc-linux-gnueabihf
				-ccc-gcc-name arm-linux-gnueabihf-gcc
				--sysroot=${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc
				-I${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/include/
			)

			link_libraries(
				-B${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/lib
				-L${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/lib
			)
		endif()
	endif()

endfunction()

#=============================================================================
#
#	px4_os_prebuild_targets
#
#	This function generates os dependent targets
#
#	Usage:
#		px4_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			)
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		px4_os_prebuild_targets(OUT target_list)
#
function(px4_os_prebuild_targets)
	px4_parse_function_args(
			NAME px4_os_prebuild_targets
			ONE_VALUE OUT
			REQUIRED OUT
			ARGN ${ARGN})

	add_custom_target(${OUT} DEPENDS uorb_headers)
endfunction()
