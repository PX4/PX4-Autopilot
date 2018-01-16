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
# 	utility functions
#
#		* px4_parse_function_args
#		* px4_join
#		* px4_add_module
#		* px4_add_common_flags
#		* px4_add_optimization_flags_for_target
#		* px4_add_executable
#		* px4_add_library
#

include(CMakeParseArguments)

#=============================================================================
#
#	px4_parse_function_args
#
#	This function simplifies usage of the cmake_parse_arguments module.
#	It is intended to be called by other functions.
#
#	Usage:
#		px4_parse_function_args(
#			NAME <name>
#			[ OPTIONS <list> ]
#			[ ONE_VALUE <list> ]
#			[ MULTI_VALUE <list> ]
#			REQUIRED <list>
#			ARGN <ARGN>)
#
#	Input:
#		NAME		: the name of the calling function
#		OPTIONS		: boolean flags
#		ONE_VALUE	: single value variables
#		MULTI_VALUE	: multi value variables
#		REQUIRED	: required arguments
#		ARGN		: the function input arguments, typically ${ARGN}
#
#	Output:
#		The function arguments corresponding to the following are set:
#		${OPTIONS}, ${ONE_VALUE}, ${MULTI_VALUE}
#
#	Example:
#		function test()
#			px4_parse_function_args(
#				NAME TEST
#				ONE_VALUE NAME
#				MULTI_VALUE LIST
#				REQUIRED NAME LIST
#				ARGN ${ARGN})
#			message(STATUS "name: ${NAME}")
#			message(STATUS "list: ${LIST}")
#		endfunction()
#
#		test(NAME "hello" LIST a b c)
#
#		OUTPUT:
#			name: hello
#			list: a b c
#
function(px4_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			if (NOT "${OUT_${arg}}" STREQUAL "0")
				message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
			endif()
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

#=============================================================================
#
#	px4_join
#
#	This function joins a list with a given separator. If list is not
#	passed, or is sent "", this will return the empty string.
#
#	Usage:
#		px4_join(OUT ${OUT} [ LIST ${LIST} ] GLUE ${GLUE})
#
#	Input:
#		LIST		: list to join
#		GLUE		: separator to use
#
#	Output:
#		OUT			: joined list
#
#	Example:
#		px4_join(OUT test_join LIST a b c GLUE ";")
#		test_join would then be:
#			"a;b;c"
#
function(px4_join)
	px4_parse_function_args(
		NAME px4_join
		ONE_VALUE OUT GLUE
		MULTI_VALUE LIST
		REQUIRED GLUE OUT
		ARGN ${ARGN})
	string (REPLACE ";" "${GLUE}" _TMP_STR "${LIST}")
	set(${OUT} ${_TMP_STR} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_add_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			[ MAIN <string> ]
#			[ STACK <string> ] !!!!!DEPRECATED, USE STACK_MAIN INSTEAD!!!!!!!!!
#			[ STACK_MAIN <string> ]
#			[ STACK_MAX <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			[ EXTERNAL ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point, if not given, assumed to be library
#		STACK			: deprecated use stack main instead
#		STACK_MAIN		: size of stack for main function
#		STACK_MAX		: maximum stack size of any frame
#		COMPILE_FLAGS		: compile flags
#		LINK_FLAGS		: link flags
#		SRCS			: source files
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#		EXTERNAL		: flag to indicate that this module is out-of-tree
#
#	Output:
#		Static library with name matching MODULE.
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			STACK_MAIN 1024
#			DEPENDS
#				git_nuttx
#			)
#
function(px4_add_module)

	px4_parse_function_args(
		NAME px4_add_module
		ONE_VALUE MODULE MAIN STACK STACK_MAIN STACK_MAX PRIORITY
		MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS
		OPTIONS EXTERNAL
		REQUIRED MODULE
		ARGN ${ARGN})

	px4_add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})

	# set defaults if not set
	set(MAIN_DEFAULT MAIN-NOTFOUND)
	set(STACK_MAIN_DEFAULT 1024)
	set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)

	foreach(property MAIN STACK_MAIN PRIORITY)
		if(NOT ${property})
			set(${property} ${${property}_DEFAULT})
		endif()
		set_target_properties(${MODULE} PROPERTIES ${property} ${${property}})
	endforeach()

	# default stack max to stack main
	if(NOT STACK_MAX)
		set(STACK_MAX ${STACK_MAIN})
	endif()
	set_target_properties(${MODULE} PROPERTIES STACK_MAX ${STACK_MAX})

	if(${OS} STREQUAL "qurt" )
		set_property(TARGET ${MODULE} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	elseif(${OS} STREQUAL "nuttx" )
		target_compile_options(${MODULE} PRIVATE -Wframe-larger-than=${STACK_MAX})
	endif()

	if(MAIN)
		target_compile_definitions(${MODULE} PRIVATE PX4_MAIN=${MAIN}_app_main)
		target_compile_definitions(${MODULE} PRIVATE MODULE_NAME="${MAIN}")
	else()
		target_compile_definitions(${MODULE} PRIVATE MODULE_NAME="${MODULE}")
	endif()

	if(COMPILE_FLAGS)
		target_compile_options(${MODULE} PRIVATE ${COMPILE_FLAGS})
	endif()

	if(INCLUDES)
		target_include_directories(${MODULE} PRIVATE ${INCLUDES})
	endif()

	if(DEPENDS)
		add_dependencies(${MODULE} ${DEPENDS})
	endif()

	# join list variables to get ready to send to compiler
	foreach(prop LINK_FLAGS)
		if(${prop})
			px4_join(OUT ${prop} LIST ${${prop}} GLUE " ")
		endif()
	endforeach()

	# store module properties in target
	# COMPILE_FLAGS and LINK_FLAGS are passed to compiler/linker by cmake
	# STACK_MAIN, MAIN, PRIORITY are PX4 specific
	if(COMPILE_FLAGS AND ${_no_optimization_for_target})
		px4_strip_optimization(COMPILE_FLAGS ${COMPILE_FLAGS})
	endif()
	foreach (prop LINK_FLAGS STACK_MAIN MAIN PRIORITY)
		if (${prop})
			set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
		endif()
	endforeach()
endfunction()

#=============================================================================
#
#	px4_add_common_flags
#
#	Set the default build flags.
#
#	Usage:
#		px4_add_common_flags(
#			BOARD <in-string>
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
#			OPTIMIZATION_FLAGS <inout-variable>
#			EXE_LINKER_FLAGS <inout-variable>
#			INCLUDE_DIRS <inout-variable>
#			LINK_DIRS <inout-variable>
#			DEFINITIONS <inout-variable>)
#
#	Input:
#		BOARD					: board
#
#	Input/Output: (appends to existing variable)
#		C_FLAGS					: c compile flags variable
#		CXX_FLAGS				: c++ compile flags variable
#		OPTIMIZATION_FLAGS			: optimization compile flags variable
#		EXE_LINKER_FLAGS			: executable linker flags variable
#		INCLUDE_DIRS				: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Example:
#		px4_add_common_flags(
#			BOARD px4fmu-v2
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			OPTIMIZATION_FLAGS optimization_flags
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_add_common_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS OPTIMIZATION_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_add_common_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

	set(warnings
		-Wall
		-Warray-bounds
		-Wdisabled-optimization
		-Werror
		-Wextra
		-Wfatal-errors
		-Wfloat-equal
		-Wformat-security
		-Winit-self
		-Wlogical-op
		-Wmissing-declarations
		-Wmissing-field-initializers
		#-Wmissing-include-dirs # TODO: fix and enable
		-Wpointer-arith
		-Wshadow
		-Wuninitialized
		-Wunknown-pragmas
		-Wunused-variable

		-Wno-implicit-fallthrough # set appropriate level and update

		-Wno-unused-parameter
		)

	if (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		# QuRT 6.4.X compiler identifies as Clang but does not support this option
		if (NOT ${OS} STREQUAL "qurt")
			list(APPEND warnings
				-Qunused-arguments
				-Wno-unused-const-variable
				-Wno-varargs
				-Wno-address-of-packed-member
				-Wno-unknown-warning-option
				-Wunused-but-set-variable
				#-Wdouble-promotion # needs work first
			)
		endif()
	else()
		list(APPEND warnings
			-Wunused-but-set-variable
			-Wformat=1
			-Wdouble-promotion
		)
	endif()

	set(_optimization_flags
		-fno-strict-aliasing
		-fomit-frame-pointer
		-funsafe-math-optimizations
		-ffunction-sections
		-fdata-sections
		)

	set(c_warnings
		-Wbad-function-cast
		-Wstrict-prototypes
		-Wmissing-prototypes
		-Wnested-externs
		)

	set(c_compile_flags
		-g
		-std=gnu99
		-fno-common
		)

	set(cxx_warnings
		-Wno-missing-field-initializers
		-Wno-overloaded-virtual # TODO: fix and remove
		-Wreorder
		)

	set(cxx_compile_flags
		-g
		-fno-exceptions
		-fno-rtti
		-std=gnu++11
		-fno-threadsafe-statics
		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		)

	# regular Clang or AppleClang
	if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		# force color for clang (needed for clang + ccache)
		list(APPEND _optimization_flags
			-fcolor-diagnostics
		)
	else()
		list(APPEND _optimization_flags
			-fno-strength-reduce
			-fno-builtin-printf
		)

		# -fcheck-new is a no-op for Clang in general
		# and has no effect, but can generate a compile
		# error for some OS
		list(APPEND cxx_compile_flags
			-fcheck-new
		)
	endif()

	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
			# force color for gcc > 4.9
			list(APPEND _optimization_flags
				-fdiagnostics-color=always
			)
		endif()
	endif()

	set(visibility_flags
		-fvisibility=hidden
		-include visibility.h
		)

	set(added_c_flags
		${c_compile_flags}
		${warnings}
		${c_warnings}
		${visibility_flags}
		)

	set(added_cxx_flags
		${cxx_compile_flags}
		${warnings}
		${cxx_warnings}
		${visibility_flags}
		)

	set(added_optimization_flags
		${_optimization_flags}
		)

	set(added_include_dirs
		${PX4_BINARY_DIR}
		${PX4_BINARY_DIR}/src
		${PX4_BINARY_DIR}/src/modules
		${PX4_SOURCE_DIR}/src
		${PX4_SOURCE_DIR}/src/drivers/boards/${BOARD}
		${PX4_SOURCE_DIR}/src/include
		${PX4_SOURCE_DIR}/src/lib
		${PX4_SOURCE_DIR}/src/lib/DriverFramework/framework/include
		${PX4_SOURCE_DIR}/src/lib/matrix
		${PX4_SOURCE_DIR}/src/modules
		${PX4_SOURCE_DIR}/src/platforms
		)

	set(added_link_dirs) # none used currently
	set(added_exe_linker_flags)

	string(TOUPPER ${BOARD} board_upper)
	string(REPLACE "-" "_" board_config ${board_upper})

	set(added_definitions
		-DCONFIG_ARCH_BOARD_${board_config}
		-D__STDC_FORMAT_MACROS
		)

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
		#message(STATUS "set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)")
	endforeach()

endfunction()

#=============================================================================
#
#	px4_strip_optimization
#
function(px4_strip_optimization name)
	set(_compile_flags)
	separate_arguments(_args UNIX_COMMAND ${ARGN})
	foreach(_flag ${_args})
		if(NOT "${_flag}" MATCHES "^-O")
			set(_compile_flags "${_compile_flags} ${_flag}")
		endif()
	endforeach()
	string(STRIP "${_compile_flags}" _compile_flags)
	set(${name} "${_compile_flags}" PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_add_optimization_flags_for_target
#
set(all_posix_cmake_targets "" CACHE INTERNAL "All cmake targets for which optimization can be suppressed")
function(px4_add_optimization_flags_for_target target)
	set(_no_optimization_for_target FALSE)
	# If the current CONFIG is posix_sitl_* then suppress optimization for certain targets.
	if(CONFIG MATCHES "^posix_sitl_")
		foreach(_regexp $ENV{PX4_NO_OPTIMIZATION})
			if("${target}" MATCHES "${_regexp}")
				set(_no_optimization_for_target TRUE)
				set(_matched_regexp "${_regexp}")
			endif()
		endforeach()
		# Create a full list of targets that optimization can be suppressed for.
		list(APPEND all_posix_cmake_targets ${target})
		set(all_posix_cmake_targets ${all_posix_cmake_targets} CACHE INTERNAL "All cmake targets for which optimization can be suppressed")
	endif()
	if(NOT ${_no_optimization_for_target})
		target_compile_options(${target} PRIVATE ${optimization_flags})
	else()
		message(STATUS "Disabling optimization for target '${target}' because it matches the regexp '${_matched_regexp}' in env var PX4_NO_OPTIMIZATION")
		target_compile_options(${target} PRIVATE -O0)
	endif()
	# Pass variable to the parent px4_add_library.
	set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_add_executable
#
#	Like add_executable but with optimization flag fixup.
#
function(px4_add_executable target)
	add_executable(${target} ${ARGN})
	px4_add_optimization_flags_for_target(${target})
endfunction()

#=============================================================================
#
#	px4_add_library
#
#	Like add_library but with optimization flag fixup.
#
function(px4_add_library target)
	add_library(${target} ${ARGN})
	add_dependencies(${target} prebuild_targets)
	px4_add_optimization_flags_for_target(${target})

	# Pass variable to the parent px4_add_module.
	set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)

	set_property(GLOBAL APPEND PROPERTY PX4_LIBRARIES ${target})
endfunction()

#=============================================================================
#
#	px4_find_python_module
#
#	Find a required python module
#
#   Usage
#		px4_find_python_module(module_name [REQUIRED])
#
function(px4_find_python_module module)
	string(TOUPPER ${module} module_upper)
	if(NOT PY_${module_upper})
		if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
			set(PY_${module}_FIND_REQUIRED TRUE)
		endif()
		# A module's location is usually a directory, but for binary modules
		# it's a .so file.
		execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
			"import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
			RESULT_VARIABLE _${module}_status
			OUTPUT_VARIABLE _${module}_location
			ERROR_QUIET 
			OUTPUT_STRIP_TRAILING_WHITESPACE)
		if(NOT _${module}_status)
			set(PY_${module_upper} ${_${module}_location} CACHE STRING
				"Location of Python module ${module}")
		endif()
	endif()
	find_package_handle_standard_args(PY_${module}
		"couldn't find python module ${module}:
		\nfor debian systems try: \
		\n\tsudo apt-get install python-${module} \
		\nor for all other OSs/debian: \
		\n\tsudo -H pip install ${module}\n" PY_${module_upper})
	#if (NOT PY_${module}_FOUND)
		#message(FATAL_ERROR "python module not found, exiting")
	#endif()
endfunction(px4_find_python_module)

# vim: set noet fenc=utf-8 ff=unix nowrap:
