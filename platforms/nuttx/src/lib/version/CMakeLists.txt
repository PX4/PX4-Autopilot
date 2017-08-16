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

add_custom_command(
	OUTPUT nuttx_version.h
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/platforms/${OS}/Tools/px_update_git_nuttx_header.py nuttx_version.h ${PX4_SOURCE_DIR} > ${PX4_BINARY_DIR}/git_nuttx_header.log
	DEPENDS prebuild_targets
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Generating OS specific git hash header"
	)
set_source_files_properties(nuttx_version.h PROPERTIES GENERATED TRUE)
add_custom_target(ver_gen_nuttx ALL DEPENDS nuttx_version.h)

include_directories(
	${PX4_SOURCE_DIR}/src/lib/version
	${CMAKE_CURRENT_BINARY_DIR}
	)

px4_add_module(
	MODULE lib__version
	COMPILE_FLAGS
	SRCS
		${PX4_SOURCE_DIR}/src/lib/version/version.c
		version_nuttx.c
	DEPENDS
		ver_gen
		ver_gen_nuttx
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
