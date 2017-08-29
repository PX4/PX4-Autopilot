############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

# check if px4 source is a git repo
if (EXISTS ${PX4_SOURCE_DIR}/.git)
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

set(px4_git_ver_header ${PX4_BINARY_DIR}/build_git_version.h)
add_custom_command(OUTPUT ${px4_git_ver_header}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/px_update_git_header.py ${px4_git_ver_header} > ${PX4_BINARY_DIR}/git_header.log
	DEPENDS
		${CMAKE_CURRENT_SOURCE_DIR}/px_update_git_header.py
		${git_dir_path}/HEAD
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	COMMENT "Generating git hash header"
	)
set_source_files_properties(${px4_git_ver_header} PROPERTIES GENERATED TRUE)
add_custom_target(ver_gen ALL DEPENDS ${px4_git_ver_header})

px4_add_module(
	MODULE lib__version
	COMPILE_FLAGS
	SRCS
		version.c
	DEPENDS
		ver_gen
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
