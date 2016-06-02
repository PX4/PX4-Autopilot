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
#		* px4_add_git_submodule
#		* px4_prepend_string
#		* px4_join
#		* px4_add_module
#		* px4_generate_messages
#		* px4_add_upload
#		* px4_add_common_flags
#

include(CMakeParseArguments)

#=============================================================================
#
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
function(px4_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

#=============================================================================
#
#	px4_add_git_submodule
#
#	This function add a git submodule target.
#
#	Usage:
#		px4_add_git_submodule(TARGET <target> PATH <path>)
#
#	Input:
#		PATH		: git submodule path
#
#	Output:
#		TARGET		: git target
#
#	Example:
#		px4_add_git_submodule(TARGET git_nuttx PATH "NuttX")
#
function(px4_add_git_submodule)
	px4_parse_function_args(
		NAME px4_add_git_submodule
		ONE_VALUE TARGET PATH
		REQUIRED TARGET PATH
		ARGN ${ARGN})
	string(REPLACE "/" "_" NAME ${PATH})
	add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/git_init_${NAME}.stamp
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND touch ${CMAKE_BINARY_DIR}/git_init_${NAME}.stamp
		DEPENDS ${CMAKE_SOURCE_DIR}/.gitmodules
		)
	add_custom_target(${TARGET}
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		DEPENDS ${CMAKE_BINARY_DIR}/git_init_${NAME}.stamp
		)
endfunction()

#=============================================================================
#
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
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point, if not given, assumed to be library
#		STACK			: deprecated use stack main instead
#		STACK_MAIN		: size of stack for main function
#		STACK_MAX		: maximum stack size of any frame
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
		REQUIRED MODULE
		ARGN ${ARGN})

	add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})

	# set defaults if not set
	set(MAIN_DEFAULT MAIN-NOTFOUND)
	set(STACK_MAIN_DEFAULT 1024)
	set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)

	# default stack max to stack main
	if(NOT STACK_MAIN AND STACK)
		set(STACK_MAIN ${STACK})
		message(AUTHOR_WARNING "STACK deprecated, USE STACK_MAIN instead!!!!!!!!!!!!")
	endif()

	foreach(property MAIN STACK_MAIN PRIORITY)
		if(NOT ${property})
			set(${property} ${${property}_DEFAULT})
		endif()
		set_target_properties(${MODULE} PROPERTIES ${property}
			${${property}})
	endforeach()

	# default stack max to stack main
	if(NOT STACK_MAX)
		set(STACK_MAX ${STACK_MAIN})
	endif()
	set_target_properties(${MODULE} PROPERTIES STACK_MAX
		${STACK_MAX})

	if(${OS} STREQUAL "qurt" )
		set_property(TARGET ${MODULE} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	elseif(${OS} STREQUAL "nuttx" )
		list(APPEND COMPILE_FLAGS -Wframe-larger-than=${STACK_MAX})
	endif()

	if(MAIN)
		set_target_properties(${MODULE} PROPERTIES
			COMPILE_DEFINITIONS PX4_MAIN=${MAIN}_app_main)
		add_definitions(-DMODULE_NAME="${MAIN}")
	else()
		add_definitions(-DMODULE_NAME="${MODULE}")
	endif()

	if(INCLUDES)
		target_include_directories(${MODULE} ${INCLUDES})
	endif()

	if(DEPENDS)
		add_dependencies(${MODULE} ${DEPENDS})
	endif()

	# join list variables to get ready to send to compiler
	foreach(prop LINK_FLAGS COMPILE_FLAGS)
		if(${prop})
			px4_join(OUT ${prop} LIST ${${prop}} GLUE " ")
		endif()
	endforeach()

	# store module properties in target
	# COMPILE_FLAGS and LINK_FLAGS are passed to compiler/linker by cmake
	# STACK_MAIN, MAIN, PRIORITY are PX4 specific
	foreach (prop COMPILE_FLAGS LINK_FLAGS STACK_MAIN MAIN PRIORITY)
		if (${prop})
			set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
		endif()
	endforeach()

endfunction()

#=============================================================================
#
#	px4_generate_messages
#
#	This function generates source code from ROS msg definitions.
#
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
function(px4_generate_messages)
	px4_parse_function_args(
		NAME px4_generate_messages
		OPTIONS VERBOSE
		ONE_VALUE OS TARGET
		MULTI_VALUE MSG_FILES DEPENDS INCLUDES
		REQUIRED MSG_FILES OS TARGET
		ARGN ${ARGN})
	set(QUIET)
	if(NOT VERBOSE)
		set(QUIET "-q")
	endif()

	# headers
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
		COMMAND ${PYTHON_EXECUTABLE}
			Tools/px_generate_uorb_topic_files.py
			--headers
			${QUIET}
			-d msg
			-o ${msg_out_path}
			-e msg/templates/uorb
			-t ${CMAKE_BINARY_DIR}/topics_temporary_header
		DEPENDS ${DEPENDS} ${MSG_FILES}
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMENT "Generating uORB topic headers"
		VERBATIM
		)

	# !sources
	set(msg_source_out_path	${CMAKE_BINARY_DIR}/topics_sources)
	set(msg_source_files_out ${msg_source_out_path}/uORBTopics.cpp)
	foreach(msg ${msg_list})
		list(APPEND msg_source_files_out ${msg_source_out_path}/${msg}.cpp)
	endforeach()
	add_custom_command(OUTPUT ${msg_source_files_out}
		COMMAND ${PYTHON_EXECUTABLE}
			Tools/px_generate_uorb_topic_files.py
			--sources
			${QUIET}
			-d msg
			-o ${msg_source_out_path}
			-e msg/templates/uorb
			-t ${CMAKE_BINARY_DIR}/topics_temporary_sources
		DEPENDS ${DEPENDS} ${MSG_FILES}
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMENT "Generating uORB topic sources"
		VERBATIM
		)
	set_source_files_properties(${msg_source_files_out} PROPERTIES GENERATED TRUE)

	# We remove uORBTopics.cpp to make sure the generator is re-run, which is
	# necessary when a .msg file is removed and because uORBTopics.cpp depends
	# on all topics.
	execute_process(COMMAND rm uORBTopics.cpp
		WORKING_DIRECTORY ${msg_source_out_path}
		ERROR_QUIET)

	# multi messages for target OS
	set(msg_multi_out_path
		${CMAKE_BINARY_DIR}/src/platforms/${OS}/px4_messages)
	set(msg_multi_files_out)
	foreach(msg ${msg_list})
		list(APPEND msg_multi_files_out ${msg_multi_out_path}/px4_${msg}.h)
	endforeach()
	add_custom_command(OUTPUT ${msg_multi_files_out}
		COMMAND ${PYTHON_EXECUTABLE}
			Tools/px_generate_uorb_topic_files.py
			--headers
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

	add_library(${TARGET}
		${msg_source_files_out}
		${msg_multi_files_out}
		${msg_files_out}
		)

endfunction()

#=============================================================================
#
#	px4_add_upload
#
#	This function generates source code from ROS msg definitions.
#
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
function(px4_add_upload)
	px4_parse_function_args(
		NAME px4_add_upload
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
			list(APPEND serial_ports
				"COM${port}")
		endforeach()
	endif()
	px4_join(OUT serial_ports LIST "${serial_ports}" GLUE ",")
	add_custom_target(${OUT}
		COMMAND ${PYTHON_EXECUTABLE}
			${CMAKE_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${BUNDLE}
		DEPENDS ${BUNDLE}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


function(px4_add_adb_push)
	px4_parse_function_args(
		NAME px4_add_upload
		ONE_VALUE OS BOARD OUT DEST
		MULTI_VALUE FILES DEPENDS
		REQUIRED OS BOARD OUT FILES DEPENDS DEST
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${CMAKE_SOURCE_DIR}/Tools/adb_upload.sh ${FILES} ${DEST}
		DEPENDS ${DEPENDS}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()

function(px4_add_adb_push_to_bebop)
	px4_parse_function_args(
		NAME px4_add_upload_to_bebop
		ONE_VALUE OS BOARD OUT DEST
		MULTI_VALUE FILES DEPENDS
		REQUIRED OS BOARD OUT FILES DEPENDS DEST
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${CMAKE_SOURCE_DIR}/Tools/adb_upload_to_bebop.sh ${FILES} ${DEST}
		DEPENDS ${DEPENDS}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()

function(px4_add_scp_push)
	px4_parse_function_args(
		NAME px4_add_upload
		ONE_VALUE OS BOARD OUT DEST
		MULTI_VALUE FILES DEPENDS
		REQUIRED OS BOARD OUT FILES DEPENDS DEST
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${CMAKE_SOURCE_DIR}/Tools/scp_upload.sh ${FILES} ${DEST}
		DEPENDS ${DEPENDS}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


#=============================================================================
#
#	px4_add_common_flags
#
#	Set ths default build flags.
#
#	Usage:
#		px4_add_common_flags(
#			BOARD <in-string>
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
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
#		EXE_LINKER_FLAGS		: executable linker flags variable
#		INCLUDE_DIRS			: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Example:
#		px4_add_common_flags(
#			BOARD px4fmu-v2
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_add_common_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_add_common_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

	set(warnings
		-Wall
		-Werror
		-Wextra
		-Wno-sign-compare
		-Wshadow
		-Wfloat-equal
		-Wpointer-arith
		-Wmissing-declarations
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

	if (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		# QuRT 6.4.X compiler identifies as Clang but does not support this option
		if (NOT ${OS} STREQUAL "qurt")
			list(APPEND warnings
				-Qunused-arguments
				-Wno-unused-const-variable
				-Wno-varargs
			)
		endif()
	else()
		list(APPEND warnings
			-Werror=unused-but-set-variable
			-Wformat=1
			#-Wlogical-op # very verbose due to eigen
			-Wdouble-promotion
			-Werror=double-promotion
		)
	endif()

	if ($ENV{MEMORY_DEBUG} MATCHES "1")
		set(max_optimization -Os)

		set(optimization_flags
			-fno-strict-aliasing
			-fno-omit-frame-pointer
			-funsafe-math-optimizations
			-ffunction-sections
			-fdata-sections
			-g3 -fsanitize=address
			)
	else()
		set(max_optimization -Os)

		if ("${OS}" STREQUAL "qurt")
			set(PIC_FLAG -fPIC)
		endif()
		set(optimization_flags
			-fno-strict-aliasing
			-fomit-frame-pointer
			-funsafe-math-optimizations
			-ffunction-sections
			-fdata-sections
			${PIC_FLAG}
			)
	endif()

	if (NOT ${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
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

	if (NOT ${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		list(APPEND c_warnings
			-Wold-style-declaration
			-Wmissing-parameter-type
		)
	endif()

	set(c_compile_flags
		-g
		-std=gnu99
		-fno-common
		)

	set(cxx_warnings
		-Wno-missing-field-initializers
		)

	set(cxx_compile_flags
		-g
		-fno-exceptions
		-fno-rtti
		-std=gnu++0x
		-fno-threadsafe-statics
		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		)

	if (NOT (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*"))
		# -fcheck-new is a no-op for Clang in general
		# and has no effect, but can generate a compile
		# error for some OS
		list(APPEND cxx_compile_flags
			-fcheck-new
		)
	endif()

	set(visibility_flags
		-fvisibility=hidden
		-include visibility.h
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
		${CMAKE_SOURCE_DIR}/src
		${CMAKE_BINARY_DIR}
		${CMAKE_BINARY_DIR}/src
		${CMAKE_SOURCE_DIR}/src/modules
		${CMAKE_SOURCE_DIR}/src/include
		${CMAKE_SOURCE_DIR}/src/lib
		${CMAKE_SOURCE_DIR}/src/platforms
		# TODO Build/versioning was in Makefile,
		# do we need this, how does it work with cmake
		${CMAKE_SOURCE_DIR}/src/drivers/boards/${BOARD}
		${CMAKE_BINARY_DIR}
		${CMAKE_BINARY_DIR}/src/modules/px4_messages
		${CMAKE_BINARY_DIR}/src/modules
		${CMAKE_SOURCE_DIR}/mavlink/include/mavlink
		${CMAKE_SOURCE_DIR}/src/lib/DriverFramework/framework/include
		)

	list(APPEND added_include_dirs
		src/lib/matrix
		)

	set(added_link_dirs) # none used currently

	string(TOUPPER ${BOARD} board_upper)
	string(REPLACE "-" "_" board_config ${board_upper})
	set(added_definitions
		-DCONFIG_ARCH_BOARD_${board_config}
		)

	if (NOT (APPLE AND (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")))
		set(added_exe_linker_flags
			-Wl,--warn-common
			-Wl,--gc-sections
			#,--print-gc-sections
			)
	endif()

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
		#message(STATUS "set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)")
	endforeach()

endfunction()

#=============================================================================
#
#	px4_mangle_name
#
#	Convert a path name to a module name
#
#	Usage:
#		px4_mangle_name(dirname newname)
#
#	Input:
#		dirname					: path to module dir
#
#	Output:
#		newname					: module name
#
#	Example:
#		px4_mangle_name(${dirpath} mangled_name)
#		message(STATUS "module name is ${mangled_name}")
#
function(px4_mangle_name dirname newname)
	set(tmp)
	string(REPLACE "/" "__" tmp ${dirname})
	set(${newname} ${tmp} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_create_git_hash_header
#
#	Create a header file containing the git hash of the current tree
#
#	Usage:
#		px4_create_git_hash_header(HEADER ${CMAKE_BUILD_DIR}/git_hash.h)
#
#	Input:
#		HEADER 		: path of the header file to generate
#
#	Example:
#		px4_create_git_hash_header(HEADER ${CMAKE_BUILD_DIR}/git_hash.h)
#
function(px4_create_git_hash_header)
	px4_parse_function_args(
		NAME px4_create_git_hash_header
		ONE_VALUE HEADER
		REQUIRED HEADER
		ARGN ${ARGN})
	execute_process(
		COMMAND git describe --always --tags
		OUTPUT_VARIABLE git_tag
		OUTPUT_STRIP_TRAILING_WHITESPACE
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	message(STATUS "GIT_TAG = ${git_tag}")
	execute_process(
		COMMAND git rev-parse --verify HEAD
		OUTPUT_VARIABLE git_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	#message(STATUS "GIT_VERSION = ${git_version}")
	set(git_version_short)
	string(SUBSTRING ${git_version} 1 16 git_version_short)
	configure_file(${CMAKE_SOURCE_DIR}/cmake/templates/build_git_version.h.in ${HEADER} @ONLY)
endfunction()

#=============================================================================
#
#	px4_generate_parameters_xml
#
#	Generates a parameters.xml file.
#
#	Usage:
#		px4_generate_parameters_xml(OUT <param-xml_file>)
#
#	Input:
#		BOARD : the board
#
#	Output:
#		OUT	: the generated xml file
#
#	Example:
#		px4_generate_parameters_xml(OUT parameters.xml)
#
function(px4_generate_parameters_xml)
	px4_parse_function_args(
		NAME px4_generate_parameters_xml
		ONE_VALUE OUT BOARD
		REQUIRED OUT BOARD
		ARGN ${ARGN})
	set(path ${CMAKE_SOURCE_DIR}/src)
	file(GLOB_RECURSE param_src_files
		${CMAKE_SOURCE_DIR}/src/*params.c
		)
	add_custom_command(OUTPUT ${OUT}
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tools/px_process_params.py
			-s ${path} --board CONFIG_ARCH_${BOARD} --xml --inject-xml
		DEPENDS ${param_src_files}
		)
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_generate_parameters_source
#
#	Generates a source file with all parameters.
#
#	Usage:
#		px4_generate_parameters_source(OUT <list-source-files> XML <param-xml-file> [SCOPE <cmake file for scoping>])
#
#	Input:
#		XML   : the parameters.xml file
#		SCOPE : the cmake file used to limit scope of the paramaters
#		DEPS  : target dependencies
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		px4_generate_parameters_source(OUT param_files XML parameters.xml SCOPE ${OS}_${BOARD}_${LABEL}.cmake )
#
function(px4_generate_parameters_source)
	px4_parse_function_args(
		NAME px4_generate_parameters_source
		ONE_VALUE OUT XML SCOPE DEPS
		REQUIRED OUT XML
		ARGN ${ARGN})
	set(generated_files
		${CMAKE_CURRENT_BINARY_DIR}/px4_parameters.h
		${CMAKE_CURRENT_BINARY_DIR}/px4_parameters.c)
	set_source_files_properties(${generated_files}
		PROPERTIES GENERATED TRUE)
	if ("${config_generate_parameters_scope}" STREQUAL "ALL")
		set(SCOPE "")
	endif()
	add_custom_command(OUTPUT ${generated_files}
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tools/px_generate_params.py ${XML} ${SCOPE}
		DEPENDS ${XML} ${DEPS} ${SCOPE}
		)
	set(${OUT} ${generated_files} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_generate_airframes_xml
#
#	Generates airframes.xml
#
#	Usage:
#		px4_generate_airframes_xml(OUT <airframe-xml-file>)
#
#	Input:
#		XML : the airframes.xml file
#		BOARD : the board
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		px4_generate_airframes_xml(OUT airframes.xml)
#
function(px4_generate_airframes_xml)
	px4_parse_function_args(
		NAME px4_generate_airframes_xml
		ONE_VALUE OUT BOARD
		REQUIRED OUT BOARD
		ARGN ${ARGN})
	set(process_airframes ${CMAKE_SOURCE_DIR}/Tools/px_process_airframes.py)
	add_custom_command(OUTPUT ${OUT}
		COMMAND ${PYTHON_EXECUTABLE} ${process_airframes}
			-a ${CMAKE_SOURCE_DIR}/ROMFS/px4fmu_common/init.d
			--board CONFIG_ARCH_BOARD_${BOARD} --xml
		)
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_copy_tracked
#
#	Copy files to a directory and keep track of dependencies.
#
#	Usage:
#		px4_copy_tracked(OUT <dest-files> FILES <in-files> DIR <dir-name>)
#
#	Input:
#		FILES	:  the source files
#		DEST		:  the directory to copy files to
#		RELATIVE :  relative directory for source files
#
#	Output:
#		OUT	: the copied files
#
#	Example:
#		px4_copy_tracked(OUT copied_files FILES src_files DEST path RELATIVE path_rel)
#
function(px4_copy_tracked)
	px4_parse_function_args(
		NAME px4_copy_tracked
		ONE_VALUE DEST OUT RELATIVE
		MULTI_VALUE FILES
		REQUIRED DEST OUT FILES
		ARGN ${ARGN})
	set(files)
	# before build, make sure dest directory exists
	execute_process(
		COMMAND cmake -E make_directory ${DEST})
	# create rule to copy each file and set dependency as source file
	set(_files_out)
	foreach(_file ${FILES})
		if (RELATIVE)
			file(RELATIVE_PATH _file_path ${RELATIVE} ${_file})
		else()
			set(_file_path ${_file})
		endif()
		set(_dest_file ${DEST}/${_file_path})
		#message(STATUS "copy ${_file} -> ${_dest_file}")
		add_custom_command(OUTPUT ${_dest_file}
			COMMAND cmake -E copy ${_file} ${_dest_file}
			DEPENDS ${_file})
		list(APPEND _files_out ${_dest_file})
	endforeach()
	set(${OUT} ${_files_out} PARENT_SCOPE)
endfunction()


# vim: set noet fenc=utf-8 ff=unix nowrap:
