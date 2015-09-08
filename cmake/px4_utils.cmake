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

include(CMakeParseArguments)

#----------------------------------------------------------------------------
#	px4_parse_function_args
#
#	This function simpliies usage of the cmake_parse_arguments module.
#	It is inteded to be called by other functions.
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
#----------------------------------------------------------------------------
function(px4_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			message(FATAL_ERROR "${IN_NAME} requires argument ${arg}, ARGN: ${IN_ARGN}")
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

#----------------------------------------------------------------------------
#	add_git_submodule
#
#	This function add a git submodule target.
#
#	Usage:
#		add_git_submodule(TARGET <target> PATH <path>)
#
#	Input:
#		PATH		: git submodule path
#
#	Output:
#		TARGET		: git target
#
#	Example:
#		add_git_submodule(TARGET git_nuttx PATH "NuttX")
#
#----------------------------------------------------------------------------
function(px4_add_git_submodule)
	px4_parse_function_args(
		NAME px4_add_git_submodule
		ONE_VALUE TARGET PATH
		REQUIRED TARGET PATH
		ARGN ${ARGN})
	string(REPLACE "/" "_" NAME ${PATH})
	add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/git_${NAME}.stamp
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND git submodule init ${PATH}
		COMMAND git submodule update ${PATH}
		COMMAND touch ${CMAKE_BINARY_DIR}/git_${NAME}.stamp
		)
	add_custom_target(${TARGET}
		DEPENDS git_${NAME}.stamp
	)
endfunction()

#----------------------------------------------------------------------------
#	px4_prepend_string
#
#	This function prepends a string to a list
#
#	Usage:
#		px4_prepend_string(OUT <output-list> STR <string> LIST <list>)
#
#	Input:
#		STR			: string to prepend
#		LIST		: list to prepend to
#
#	Output:
#		${OUT}		: prepended list
#
#	Example:
#		px4_prepend_string(OUT test_str STR "path/to/" LIST src/file1.cpp src/file2.cpp)
#		test_str would then be:
#			path/to/src/file1.cpp
#			path/to/src/file2.cpp
#
#----------------------------------------------------------------------------
function(px4_prepend_string)
	px4_parse_function_args(
		NAME px4_prepend_string
		ONE_VALUE OUT STR
		MULTI_VALUE LIST
		REQUIRED OUT STR LIST
		ARGN ${ARGN})
	set(${OUT})
	foreach(file ${LIST})
		list(APPEND ${OUT} ${STR}${file})
	endforeach()
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()

#----------------------------------------------------------------------------
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
#----------------------------------------------------------------------------
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

#----------------------------------------------------------------------------
#	px4_add_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			[ MAIN <string> ]
#			[ STACK <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point, if not given, assumed to be library
#		STACK			: size of stack
#		COMPILE_FLAGS	: compile flags
#		LINK_FLAGS		: link flags
#		SRCS			: source files
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#
#	Output:
#		Static library with name matching MODULE.
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			STACK 1024
#			DEPENDS
#				git_nuttx
#			)
#
#----------------------------------------------------------------------------
function(px4_add_module)
	px4_parse_function_args(
		NAME px4_add_module
		ONE_VALUE MODULE MAIN STACK PRIORITY
		MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS
		REQUIRED MODULE
		ARGN ${ARGN})
	add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})
	if(INCLUDES)
		target_include_directories(${MODULE} ${INCLUDES})
	endif()
	if(DEPENDS)
		add_dependencies(${MODULE} ${DEPENDS})
	endif()
	foreach(prop LINK_FLAGS COMPILE_FLAGS)
		if(${prop})
			px4_join(OUT ${prop} LIST ${${prop}} GLUE " ")
		endif()
	endforeach()
	foreach (prop STACK MAIN COMPILE_FLAGS LINK_FLAGS PRIORITY)
		if (${prop})
			set_target_properties(${MODULE} PROPERTIES "${prop}" "${${prop}}")
		endif()
	endforeach()
	set_target_properties(${MODULE} PROPERTIES LINK_INTERFACE_MULTIPLICITY 4)
endfunction()

#----------------------------------------------------------------------------
#	px4_generate_messages
#
#	This function generates source code from ROS msg definitions.

#	Usage:
#		px4_generate_messages(TARGET <target> MSGS <msg-files>)
#
#	Input:
#		MSG_FILES	: the ROS msgs to generate files from
#		OS			: the operating system selected
#		DEPENDS		: dependencies
#
#	Output:
#		TARGET		: the message generation target
#
#	Example:
#		px4_generate_messages(TARGET <target>
#			MSG_FILES <files> OS <operating-system>
#			[ DEPENDS <dependencies> ]
#			)
#
#----------------------------------------------------------------------------
function(px4_generate_messages)
	px4_parse_function_args(
		NAME px4_generate_messages
		OPTIONS VERBOSE
		ONE_VALUE OS TARGET
		MULTI_VALUE MSG_FILES DEPENDS
		REQUIRED MSG_FILES OS TARGET
		ARGN ${ARGN})
	set(QUIET)
	if(NOT VERBOSE)
		set(QUIET "-q")
	endif()
	set(PYTHONPATH "${CMAKE_SOURCE_DIR}/Tools/genmsg/src:${CMAKE_SOURCE_DIR}/Tools/gencpp/src:$ENV{PYTHONPATH}")
	set(msg_out_path ${CMAKE_BINARY_DIR}/src/modules/uORB/topics)
	set(msg_list)
	foreach(msg_file ${MSG_FILES})
		get_filename_component(msg ${msg_file} NAME_WE)
		list(APPEND msg_list ${msg})
	endforeach()
	set(msg_files_out)
	foreach(msg ${msg_list})
		list(APPEND msg_files_out ${msg_out_path}/${msg}.h)
	endforeach()
	add_custom_command(OUTPUT ${msg_files_out}
		COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
			Tools/px_generate_uorb_topic_headers.py
			${QUIET}
			-d msg
			-o ${msg_out_path} 
			-e msg/templates/uorb
			-t ${CMAKE_BINARY_DIR}/topics_temporary
		DEPENDS ${DEPENDS} ${MSG_FILES}
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMENT "Generating uORB topic headers"
		VERBATIM
		)

	# multi messages for target OS
	set(msg_multi_out_path
		${CMAKE_BINARY_DIR}/src/platforms/${OS}/px4_messages)
	set(msg_multi_files_out)
	foreach(msg ${msg_list})
		list(APPEND msg_multi_files_out ${msg_multi_out_path}/px4_${msg}.h)
	endforeach()
	add_custom_command(OUTPUT ${msg_multi_files_out}
		COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
			Tools/px_generate_uorb_topic_headers.py
			${QUIET}
			-d msg
			-o ${msg_multi_out_path} 
			-e msg/templates/px4/uorb
			-t ${CMAKE_BINARY_DIR}/multi_topics_temporary/${OS}
			-p "px4_"
		DEPENDS ${DEPENDS} ${MSG_FILES}
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMENT "Generating uORB topic multi headers for ${OS}"
		VERBATIM
		)
	add_custom_target(${TARGET}
		DEPENDS ${msg_multi_files_out} ${msg_files_out})
endfunction()

#----------------------------------------------------------------------------
#	px4_add_upload
#
#	This function generates source code from ROS msg definitions.

#	Usage:
#		px4_add_upload(OUT <target> BUNDLE <file.px4>)
#
#	Input:
#		BUNDLE		: the firmware.px4 file
#		OS			: the operating system
#		BOARD		: the board
#
#	Output:
#		OUT			: the firmware target
#
#	Example:
#		px4_add_upload(OUT upload
#			BUNDLE main.px4
#			)
#
#----------------------------------------------------------------------------
function(px4_add_upload)
	px4_parse_function_args(
		NAME px4_generate_messages
		ONE_VALUE OS BOARD OUT BUNDLE
		REQUIRED OS BOARD OUT BUNDLE
		ARGN ${ARGN})
	set(serial_ports)
	if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
		list(APPEND serial_ports
			/dev/serial/by-id/usb-3D_Robotics*
			/dev/serial/by-id/pci-3D_Robotics*
			)
	elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
		list(APPEND serial_ports
			/dev/tty.usbmodemPX*,/dev/tty.usbmodem*
			)
	elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
		foreach(port RANGE 32 0)
			list(APPEND "COM${port}")
		endforeach()
	endif()
	px4_join(OUT serial_ports LIST "${serial_ports}" GLUE ",")
	add_custom_target(${OUT}
		COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE}
			${CMAKE_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${BUNDLE}
		DEPENDS ${BUNDLE}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		)
endfunction()

#----------------------------------------------------------------------------
#	px4_add_common_flags
#
#	Set ths default build flags.
#
#	Usage:
#		px4_add_common_flags(
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
#			EXE_LINKER_FLAGS <inout-variable>
#			INCLUDE_DIRS <inout-variable>
#			LINK_DIRS <inout-variable>
#			DEFINITIONS <inout-variable>)
#
#	Input/Output: (appends to existing variable)
#		C_FLAGS					: c compile flags variable
#		CXX_FLAGS				: c++ compile flags variable
#		EXE_LINKER_FLAGS		: executable linker flags variable
#		INCLUDE_DIRS			: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Example:
#		px4_add_common_flags(
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
#----------------------------------------------------------------------------
function(px4_add_common_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_add_common_flags
		ONE_VALUE ${inout_vars}
		REQUIRED ${inout_vars}
		ARGN ${ARGN})

	set(warnings
		-Wall
		-Wno-sign-compare
		-Wextra
		#-Wshadow # very verbose due to eigen
		-Wfloat-equal
		-Wframe-larger-than=1024
		-Wpointer-arith
		-Wmissing-declarations
		-Wpacked
		-Wno-unused-parameter
		-Werror=format-security
		-Werror=array-bounds
		-Wfatal-errors
		-Werror=unused-variable
		-Werror=reorder
		-Werror=uninitialized
		-Werror=init-self 
		#-Wcast-qual  - generates spurious noreturn attribute warnings,
		#               try again later
		#-Wconversion - would be nice, but too many "risky-but-safe"
		#               conversions in the code
		#-Wcast-align - would help catch bad casts in some cases,
		#               but generates too many false positives
		)

	if (NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
		list(APPEND warnings
			-Werror=unused-but-set-variable
			-Wformat=1
			#-Wlogical-op # very verbose due to eigen
			-Wdouble-promotion
			-Werror=double-promotion
		)
	endif()

	set(max_optimization -Os)

	set(optimization_flags
		-fno-strict-aliasing
		-fomit-frame-pointer
		-funsafe-math-optimizations
		-ffunction-sections
		-fdata-sections
		)
	if (NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
		list(APPEND optimization_flags
			-fno-strength-reduce
			-fno-builtin-printf
		)
	endif()

	set(c_warnings
		-Wbad-function-cast
		-Wstrict-prototypes
		-Wmissing-prototypes
		-Wnested-externs
		)

	if (NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
		list(APPEND c_warnings
			-Wold-style-declaration
			-Wmissing-parameter-type
		)
	endif()

	set(c_compile_flags
		-std=gnu99
		-fno-common
		)

	set(cxx_warnings
		-Wno-missing-field-initializers
		)
	set(cxx_compile_flags
		-fno-exceptions
		-fno-rtti
		-std=gnu++0x
		-fno-threadsafe-statics
		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		)

	set(visibility_flags
		-fvisibility=hidden
		"-include ${CMAKE_SOURCE_DIR}/src/include/visibility.h"
		)

	set(added_c_flags
		${c_compile_flags}
		${warnings}
		${c_warnings}
		${max_optimization}
		${optimization_flags}
		${visibility_flags}
		)

	set(added_cxx_flags
		${cxx_compile_flags}
		${warnings}
		${cxx_warnings}
		${max_optimization}
		${optimization_flags}
		${visibility_flags}
		)

	set(added_include_dirs
		src
		${CMAKE_BINARY_DIR}/src
		src/modules
		src/include
		src/lib
		src/platforms
		# TODO Build/versioning was in Makefile,
		# do we need this, how does it work with cmake
		src/drivers/boards/${BOARD}
		src/lib/eigen
		${CMAKE_BINARY_DIR}
		${CMAKE_BINARY_DIR}/src/modules/px4_messages
		${CMAKE_BINARY_DIR}/src/modules
		mavlink/include/mavlink
		)

	set(added_link_dirs) # none used currently

	set(added_definitions
		-DCONFIG_ARCH_BOARD_${BOARD_CONFIG}
		)

	set(added_exe_link_flags
		-Wl,--warn-common
		-Wl,--gc-sections
		)

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
	endforeach()

endfunction()






# vim: set noet fenc=utf-8 ff=unix nowrap:
