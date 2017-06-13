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

include(CMakeParseArguments)
include(common/px4_base)

#=============================================================================
#
#	Defined functions in this file
#
# 	utility functions
#
#		* px4_add_git_submodule
#		* px4_create_git_hash_header
#

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
	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/git_init_${NAME}.stamp
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
		COMMAND touch ${PX4_BINARY_DIR}/git_init_${NAME}.stamp
		DEPENDS ${PX4_SOURCE_DIR}/.gitmodules
		)
	add_custom_target(${TARGET}
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
# todo:Not have 2 list of submodules one (see the end of Tools/check_submodules.sh and Firmware/CMakeLists.txt)
# using the list of submodules from the CMake file to drive the test
#		COMMAND Tools/check_submodules.sh ${PATH}
		DEPENDS ${PX4_BINARY_DIR}/git_init_${NAME}.stamp
		)
endfunction()

#=============================================================================
#
#	px4_create_git_hash_header
#
#	Create a header file containing the git hash of the current tree
#
#	Usage:
#		px4_create_git_hash_header()
#
#	Example:
#		px4_create_git_hash_header()
#
function(px4_create_git_hash_header)
	px4_parse_function_args(
		NAME px4_create_git_hash_header
		ARGN ${ARGN})

	set(px4_git_ver_header ${PX4_BINARY_DIR}/build_git_version.h)

	# check if px4 source is a git repo
	if(EXISTS ${PX4_SOURCE_DIR}/.git)
		if (IS_DIRECTORY ${PX4_SOURCE_DIR}/.git)
			# standard git repo
			set(git_dir_path ${PX4_SOURCE_DIR}/.git)
		else()
			# git submodule
			file(READ ${PX4_SOURCE_DIR}/.git git_dir_path)
			string(STRIP ${git_dir_path} git_dir_path)
			string(REPLACE "gitdir: " "" git_dir_path ${git_dir_path})
			get_filename_component(git_dir_path ${git_dir_path} ABSOLUTE)
		endif()
	else()
		message(FATAL_ERROR "is not a git repository")
	endif()
	if(NOT IS_DIRECTORY "${git_dir_path}")
		message(FATAL_ERROR "${git_dir_path} is not a directory")
	endif()

	set(deps
		${PX4_SOURCE_DIR}/Tools/px_update_git_header.py
		${git_dir_path}/index
		${git_dir_path}/HEAD)

	add_custom_command(
		OUTPUT ${px4_git_ver_header}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_update_git_header.py ${px4_git_ver_header} > ${PX4_BINARY_DIR}/git_header.log
		DEPENDS ${deps}
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
		COMMENT "Generating git hash header"
		)
	set_source_files_properties(${px4_git_ver_header} PROPERTIES GENERATED TRUE)
	add_custom_target(ver_gen ALL DEPENDS ${px4_git_ver_header})
endfunction()
