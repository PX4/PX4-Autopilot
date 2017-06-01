############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
#		* px4_generate_messages
#		* px4_generate_parameters_xml
#		* px4_generate_parameters_source
#		* px4_generate_airframes_xml
#

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

	if("${config_nuttx_config}" STREQUAL "bootloader")
		# do nothing for bootloaders
	else()

		set(QUIET)
		if (NOT VERBOSE)
			set(QUIET "-q")
		endif()

		# headers
		set(msg_out_path ${PX4_BINARY_DIR}/src/modules/uORB/topics)
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
				-f ${MSG_FILES}
				-i ${INCLUDES}
				-o ${msg_out_path}
				-e msg/templates/uorb
				-t ${PX4_BINARY_DIR}/topics_temporary_header
			DEPENDS ${DEPENDS} ${MSG_FILES}
			WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			COMMENT "Generating uORB topic headers"
			VERBATIM
			)

		# !sources
		set(msg_source_out_path	${PX4_BINARY_DIR}/topics_sources)
		set(msg_source_files_out ${msg_source_out_path}/uORBTopics.cpp)
		foreach(msg ${msg_list})
			list(APPEND msg_source_files_out ${msg_source_out_path}/${msg}.cpp)
		endforeach()
		add_custom_command(OUTPUT ${msg_source_files_out}
			COMMAND ${PYTHON_EXECUTABLE}
				Tools/px_generate_uorb_topic_files.py
				--sources
				${QUIET}
				-f ${MSG_FILES}
				-i ${INCLUDES}
				-o ${msg_source_out_path}
				-e msg/templates/uorb
				-t ${PX4_BINARY_DIR}/topics_temporary_sources
			DEPENDS ${DEPENDS} ${MSG_FILES}
			WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			COMMENT "Generating uORB topic sources"
			VERBATIM
			)
		set_source_files_properties(${msg_source_files_out} PROPERTIES GENERATED TRUE)

		# multi messages for target OS
		set(msg_multi_out_path ${PX4_BINARY_DIR}/src/platforms/${OS}/px4_messages)
		set(msg_multi_files_out)
		foreach(msg ${msg_list})
			list(APPEND msg_multi_files_out ${msg_multi_out_path}/px4_${msg}.h)
		endforeach()
		add_custom_command(OUTPUT ${msg_multi_files_out}
			COMMAND ${PYTHON_EXECUTABLE}
				Tools/px_generate_uorb_topic_files.py
				--headers
				${QUIET}
				-f ${MSG_FILES}
				-i ${INCLUDES}
				-o ${msg_multi_out_path}
				-e msg/templates/px4/uorb
				-t ${PX4_BINARY_DIR}/multi_topics_temporary/${OS}
				-p "px4_"
			DEPENDS ${DEPENDS} ${MSG_FILES}
			WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			COMMENT "Generating uORB topic multi headers for ${OS}"
			VERBATIM
			)

		px4_add_library(${TARGET}
			${msg_source_files_out}
			${msg_multi_files_out}
			${msg_files_out}
			)
    endif()
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
#		MODULES : a list of px4 modules used to limit scope of the paramaters
#		OVERRIDES : A json dict with param names as keys and param default
# 			overrides as values
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
		ONE_VALUE OUT BOARD OVERRIDES
		MULTI_VALUE MODULES
		REQUIRED MODULES OUT BOARD
		ARGN ${ARGN})
	set(path ${PX4_SOURCE_DIR}/src)
	file(GLOB_RECURSE param_src_files
		${PX4_SOURCE_DIR}/src/*params.c
		)
	if (NOT OVERRIDES)
		set(OVERRIDES "{}")
	endif()

	# get full path for each module
	set(module_list)
	if(DISABLE_PARAMS_MODULE_SCOPING)
		set(module_list ${path})
	else()
		foreach(module ${MODULES})
			list(APPEND module_list ${PX4_SOURCE_DIR}/src/${module})
		endforeach()
	endif()

	add_custom_command(OUTPUT ${OUT}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_params.py
			-s ${module_list} ${EXTERNAL_MODULES_LOCATION}
			--board CONFIG_ARCH_${BOARD} --xml --inject-xml
			--overrides ${OVERRIDES}
		DEPENDS ${param_src_files} ${PX4_SOURCE_DIR}/Tools/px_process_params.py
			${PX4_SOURCE_DIR}/Tools/px_generate_params.py
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
#		px4_generate_parameters_source(OUT <list-source-files> XML <param-xml-file> MODULES px4 module list)
#
#	Input:
#		XML   : the parameters.xml file
#		MODULES : a list of px4 modules used to limit scope of the paramaters
#		DEPS  : target dependencies
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		px4_generate_parameters_source(OUT param_files XML parameters.xml MODULES lib/controllib modules/ekf2)
#
function(px4_generate_parameters_source)
	px4_parse_function_args(
		NAME px4_generate_parameters_source
		ONE_VALUE OUT XML DEPS
		MULTI_VALUE MODULES
		REQUIRED MODULES OUT XML
		ARGN ${ARGN})

	set(generated_files
		${CMAKE_CURRENT_BINARY_DIR}/px4_parameters.h
		${CMAKE_CURRENT_BINARY_DIR}/px4_parameters.c)

	set_source_files_properties(${generated_files} PROPERTIES GENERATED TRUE)

	if(DISABLE_PARAMS_MODULE_SCOPING)
		add_custom_command(OUTPUT ${generated_files}
			COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_generate_params.py
				--xml ${XML} --dest ${CMAKE_CURRENT_BINARY_DIR}
			DEPENDS ${XML} ${DEPS}
		)
	else()
		px4_join(OUT module_list LIST ${MODULES} GLUE ",")
		add_custom_command(OUTPUT ${generated_files}
			COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_generate_params.py
				--xml ${XML} --modules ${module_list} --dest ${CMAKE_CURRENT_BINARY_DIR}
			DEPENDS ${XML} ${DEPS}
		)
	endif()

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
	set(process_airframes ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py)
	add_custom_command(OUTPUT ${OUT}
		COMMAND ${PYTHON_EXECUTABLE} ${process_airframes}
			-a ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d
			--board CONFIG_ARCH_BOARD_${BOARD} --xml
		)
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()
