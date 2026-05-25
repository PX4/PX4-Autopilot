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

# Homebrew's qt@5 is keg-only on macOS. gz-gui8 (pulled in by gz-sim8
# from the Gazebo simulation modules) links against Qt5 and CMake
# cannot locate it without a prefix hint. This file is included early
# from the root CMakeLists.txt, before any add_subdirectory descends
# into find_package(gz-sim8). Only runs when the user has not already
# set Qt5_DIR.
if(APPLE AND NOT DEFINED Qt5_DIR)
	execute_process(
		COMMAND brew --prefix qt@5
		OUTPUT_VARIABLE _brew_qt5_prefix
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)
	if(_brew_qt5_prefix AND EXISTS "${_brew_qt5_prefix}/lib/cmake/Qt5")
		list(APPEND CMAKE_PREFIX_PATH "${_brew_qt5_prefix}")
	endif()
endif()

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
# 		* px4_os_determine_build_chip
#		* px4_os_prebuild_targets
#

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
	configure_file(${PX4_SOURCE_DIR}/platforms/common/apps.cpp.in ${OUT}.cpp)
	configure_file(${PX4_SOURCE_DIR}/platforms/common/apps.h.in ${OUT}.h)
endfunction()


#=============================================================================
#
#	px4_posix_generate_alias
#
#	This function generates the px4-alias.sh script containing the command
#	aliases for all modules and commands.
#
#	Usage:
#		px4_posix_generate_alias(
#			MODULE_LIST <in-list>
#			OUT <file>
#			PREFIX <prefix>)
#
#	Input:
#		MODULE_LIST	: list of modules
#		PREFIX	: command prefix (e.g. "px4-")
#
#	Output:
#		OUT	: px4-alias.sh file path
#
#	Example:
#		px4_posix_generate_alias(
#			OUT <generated-src> MODULE_LIST px4_simple_app PREFIX px4-)
#
function(px4_posix_generate_alias)
	px4_parse_function_args(
		NAME px4_posix_generate_alias
		ONE_VALUE OUT PREFIX
		MULTI_VALUE MODULE_LIST
		REQUIRED OUT PREFIX MODULE_LIST
		ARGN ${ARGN})

	set(alias_string)
	foreach(module ${MODULE_LIST})
		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()
		if (MAIN)
			set(alias_string
				"${alias_string}alias ${MAIN}='${PREFIX}${MAIN} --instance $px4_instance'\n"
			)
		endif()
	endforeach()
	configure_file(${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/px4-alias.sh_in ${OUT})
endfunction()


#=============================================================================
#
#	px4_posix_generate_symlinks
#
#	This function generates symlinks for all modules/commands.
#
#	Usage:
#		px4_posix_generate_symlinks(
#			TARGET <target>
#			MODULE_LIST <in-list>
#			PREFIX <prefix>)
#
#	Input:
#		MODULE_LIST	: list of modules
#		PREFIX	: command prefix (e.g. "px4-")
#		TARGET	: cmake target for which the symlinks should be created
#
#	Example:
#		px4_posix_generate_symlinks(
#			TARGET px4 MODULE_LIST px4_simple_app PREFIX px4-)
#
function(px4_posix_generate_symlinks)
	px4_parse_function_args(
		NAME px4_posix_generate_symlinks
		ONE_VALUE TARGET PREFIX
		MULTI_VALUE MODULE_LIST
		REQUIRED TARGET PREFIX MODULE_LIST
		ARGN ${ARGN})

	foreach(module ${MODULE_LIST})

		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()

		if (MAIN)
			set(ln_name "${PREFIX}${MAIN}")
			if(WIN32 OR MINGW)
				set(alias_executable "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${ln_name}.exe")

				# Windows symlinks require extra privileges. Hard links keep the
				# px4-<module>.exe entry points without duplicating the MSVC exe.
				add_custom_command(TARGET ${TARGET}
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E remove -f
						${alias_executable}
					COMMAND ${CMAKE_COMMAND} -E create_hardlink
						$<TARGET_FILE:${TARGET}>
						${alias_executable}
				)
			else()
				add_custom_command(TARGET ${TARGET}
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E create_symlink ${TARGET} ${ln_name}
					WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
				)
			endif()
		endif()
	endforeach()
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

	add_definitions(-D__PX4_POSIX)

	if(MSVC)
		add_definitions(-Dnoreturn_function=)
	else()
		add_definitions(-Dnoreturn_function=__attribute__\(\(noreturn\)\))
	endif()

	include_directories(platforms/posix/include)

	# When cross-compiling for Windows (MinGW) the shim headers have to
	# sit *before* the MinGW sysroot in the search path so our termios.h,
	# sys/mman.h, etc. are picked up instead of "no such file". -idirafter
	# is not enough — sysroot headers win otherwise.
	if(WIN32 OR MINGW)
		include_directories(BEFORE platforms/posix/include/windows_shim)
	endif()

	if ("${PX4_BOARD}" MATCHES "sitl")

		if(WIN32 OR MINGW)
			add_definitions(
				-D__PX4_WINDOWS
				-D_GNU_SOURCE
				-D_WIN32_WINNT=0x0A00
				-DWINVER=0x0A00
				-DNOMINMAX
				-DWIN32_LEAN_AND_MEAN
				-D_USE_MATH_DEFINES
				)

			if(MSVC)
				add_definitions(
					-D_CRT_SECURE_NO_WARNINGS
					-D_CRT_NONSTDC_NO_DEPRECATE
					-D_WINSOCK_DEPRECATED_NO_WARNINGS
				)
			else()
				add_definitions(
					# Strip __declspec(dllimport) from WinSock prototypes so PX4
					# call sites resolve to the bare `bind`/`socket`/etc. symbol
					# (a thunk emitted into the executable) rather than going
					# through `__imp_<name>`. Required for the linker --wrap
					# flags configured in platforms/posix/cmake/windows.cmake (which
					# route those bare symbols through the errno-translating
					# wrappers in src/px4/windows/posix/net/socket_wrap.cpp) to actually fire.
					-DWINSOCK_API_LINKAGE=
				)

				# Relax a few warning classes that fire in every translation
				# unit on MinGW and drown real diagnostics. Note: *do not* use
				# -Wno-format here — PX4 globally enables -Wformat-security /
				# -Wformat=1 and disabling the parent -Wformat makes those
				# flags "ignored without -Wformat", which becomes an error
				# under -Wfatal-errors.
				add_compile_options(
					-Wno-format-nonliteral
					-Wno-format-truncation
					-Wno-format-zero-length
					-Wno-pedantic-ms-format
					-Wno-cast-function-type
					-Wno-unknown-pragmas
					-Wno-attributes
					-Wno-unused-variable
					-Wno-unused-function
					-Wno-unused-but-set-variable
					-Wno-missing-field-initializers
					-Wno-deprecated-declarations
					# Even with __USE_MINGW_ANSI_STDIO=1, GCC's -Wformat still
					# checks printf arguments against the ms_printf attribute
					# that MinGW bakes into its stdio.h prototype. PX4 uses
					# PRIu64/%llu/%zu heavily (via inttypes.h) — downgrade
					# format mismatches from error to warning rather than
					# mechanically rewriting thousands of call sites.
					-Wno-error=format
					-Wno-error=format-extra-args
					# Cross-compile GCC flags uninitialized-use false positives
					# where native GCC doesn't — downgrade, don't rewrite.
					-Wno-error=uninitialized
					-Wno-error=maybe-uninitialized
					# WinSock typedefs SOCKET = unsigned long long; PX4 stores
					# sockets in `int _fd` like everyone else. Downgrade the
					# resulting narrowing warnings instead of rewriting every
					# call site.
					-Wno-error=narrowing
					# MinGW LLP64: `unsigned long` is 32-bit but a pointer
					# is 64-bit. A handful of PX4 call sites cast int->void*
					# through `unsigned long`. Accept the warning rather
					# than rewriting every cast with uintptr_t.
					-Wno-error=int-to-pointer-cast
					-Wno-error=pointer-to-int-cast
					# Same LLP64 mismatch for C++'s stricter cast-from-pointer
					# check. `(unsigned long)&p` truncates on Win64. Let the
					# warning through without aborting the build.
					$<$<COMPILE_LANGUAGE:CXX>:-fpermissive>
					-Wno-error=shadow
					)
			endif()

		elseif(UNIX AND APPLE)
			add_definitions(-D__PX4_DARWIN)

			# Silence Apple ld warning about duplicate static libs. CMake intentionally
			# re-emits them to resolve circular deps (px4_layer, px4_work_queue,
			# px4_daemon, lockstep_scheduler). See also CMAKE_*_ARCHIVE_FINISH override
			# at the top of the root CMakeLists.txt for the matching ranlib silence.
			add_link_options(LINKER:-no_warn_duplicate_libraries)

		elseif(CYGWIN)
			add_definitions(
				-D__PX4_CYGWIN
				-D_GNU_SOURCE
				-D__USE_LINUX_IOCTL_DEFS
				-U__CUSTOM_FILE_IO__
				)
		else()
			add_definitions(-D__PX4_LINUX)
		endif()

	endif()

	# -Wbad-function-cast is a C-only warning but it trips on the winsock
	# socket() return type vs int on MinGW. Exclude it on Windows.
	if(NOT (WIN32 OR MINGW))
		add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wbad-function-cast>)
	endif()

endfunction()

#=============================================================================
#
#	px4_os_determine_build_chip
#
#	Sets PX4_CHIP and PX4_CHIP_MANUFACTURER.
#
#	Usage:
#		px4_os_determine_build_chip()
#
function(px4_os_determine_build_chip)

	# always use generic chip and chip manufacturer
	set(PX4_CHIP "generic" CACHE STRING "PX4 Chip" FORCE)
	set(PX4_CHIP_MANUFACTURER "generic" CACHE STRING "PX4 Chip Manufacturer" FORCE)

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
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		px4_os_prebuild_targets(OUT target_list BOARD px4_fmu-v2)
#
function(px4_os_prebuild_targets)
	px4_parse_function_args(
			NAME px4_os_prebuild_targets
			ONE_VALUE OUT BOARD
			REQUIRED OUT
			ARGN ${ARGN})

	add_library(prebuild_targets INTERFACE)
	target_link_libraries(prebuild_targets INTERFACE px4_layer drivers_board)
	add_dependencies(prebuild_targets DEPENDS uorb_headers)

endfunction()
