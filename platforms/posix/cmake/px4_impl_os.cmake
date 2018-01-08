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
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake/posix)

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
#	Usage:
#		px4_os_add_flags()
#
function(px4_os_add_flags)

	include_directories(src/platforms/posix/include)

	# This block sets added_definitions and added_cxx_flags.
	add_definitions(
		-D__PX4_POSIX
		-Dnoreturn_function=__attribute__\(\(noreturn\)\)
		)
		
	include_directories(platforms/posix/include)

	if(UNIX AND APPLE)
		add_definitions(
			-D__PX4_DARWIN
			-D__DF_DARWIN
			)

		if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
			message(FATAL_ERROR "PX4 Firmware requires XCode 8 or newer on Mac OS. Version installed on this system: ${CMAKE_CXX_COMPILER_VERSION}")
		endif()

		EXEC_PROGRAM(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
		STRING(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
		# message(STATUS "PX4 Darwin Version: ${DARWIN_VERSION}")
		if (DARWIN_VERSION LESS 16)
			add_definitions(
				-DCLOCK_MONOTONIC=1
				-DCLOCK_REALTIME=0
				-D__PX4_APPLE_LEGACY
				)
		endif()

	elseif(CYGWIN)
		add_definitions(
			-D__PX4_CYGWIN
			-D_GNU_SOURCE
			-D__USE_LINUX_IOCTL_DEFS
			-U __CUSTOM_FILE_IO__
			)

	else()
		add_definitions(
			-D__PX4_LINUX
			-D__DF_LINUX
			)

	endif()

	# This block sets added_c_flags (appends to others).
	if ("${BOARD}" STREQUAL "eagle")

		if ("$ENV{HEXAGON_ARM_SYSROOT}" STREQUAL "")
			message(FATAL_ERROR "HEXAGON_ARM_SYSROOT not set")
		else()
			set(HEXAGON_ARM_SYSROOT $ENV{HEXAGON_ARM_SYSROOT})
		endif()

		# Add the toolchain specific flags
		add_compile_options(--sysroot=${HEXAGON_ARM_SYSROOT})

		list(APPEND added_exe_linker_flags
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/usr/lib
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib
			--sysroot=${HEXAGON_ARM_SYSROOT}
			)

	# This block sets added_c_flags (appends to others).
	elseif ("${BOARD}" STREQUAL "excelsior")

		if ("$ENV{HEXAGON_ARM_SYSROOT}" STREQUAL "")
			message(FATAL_ERROR "HEXAGON_ARM_SYSROOT not set")
		else()
			set(HEXAGON_ARM_SYSROOT $ENV{HEXAGON_ARM_SYSROOT})
		endif()
		
		set(excelsior_flags --sysroot=${HEXAGON_ARM_SYSROOT}/lib32-apq8096 -mfloat-abi=softfp -mfpu=neon -mthumb-interwork)

		# Add the toolchain specific flags
		add_compile_options(--sysroot=${HEXAGON_ARM_SYSROOT}/lib32-apq8096 -mfloat-abi=softfp -mfpu=neon -mthumb-interwork)

		list(APPEND added_cxx_flags
			--sysroot=${HEXAGON_ARM_SYSROOT}/lib32-apq8096 -mfloat-abi=softfp -mfpu=neon -mthumb-interwork
			)

		list(APPEND added_exe_linker_flags
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib32-apq8096/usr/lib
			-Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib32-apq8096/lib

			--sysroot=${HEXAGON_ARM_SYSROOT}/lib32-apq8096 -mfloat-abi=softfp -mfpu=neon -mthumb-interwork

			)

	elseif ("${BOARD}" STREQUAL "rpi")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-a53" PARENT_SCOPE)
		#add_compile_options(-mcpu=cortex-a53 -mfpu=neon -mfloat-abi=hard)

		find_program(CXX_COMPILER_PATH ${CMAKE_CXX_COMPILER})

		GET_FILENAME_COMPONENT(CXX_COMPILER_PATH ${CXX_COMPILER_PATH} DIRECTORY)
		GET_FILENAME_COMPONENT(CXX_COMPILER_PATH "${CXX_COMPILER_PATH}/../" ABSOLUTE)

		IF ("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
			set(CLANG_COMPILE_FLAGS
				--target=arm-pc-linux-gnueabihf
				-ccc-gcc-name arm-linux-gnueabihf-gcc
				--sysroot=${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc
				-I${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/include/
			)

			list(APPEND added_c_flags ${CLANG_COMPILE_FLAGS})
			list(APPEND added_cxx_flags ${CLANG_COMPILE_FLAGS})
			list(APPEND added_exe_linker_flags ${POSIX_CMAKE_EXE_LINKER_FLAGS} ${CLANG_COMPILE_FLAGS}
				-B${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/lib
				-L${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/lib
			)
		ENDIF()
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
#	Input:
#		BOARD 		: board
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

	add_library(${OUT} INTERFACE)
	add_dependencies(${OUT} DEPENDS uorb_headers)
endfunction()
