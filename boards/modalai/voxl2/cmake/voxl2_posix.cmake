############################################################################
#
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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
#
# Overview:
# Voxl2 PX4 is built in 2 parts, the part that runs on the
# application (apps) processor, and the library that is loaded on the DSP.
#
############################################################################

include(px4_git)

list(APPEND CMAKE_MODULE_PATH
	"${PX4_SOURCE_DIR}/platforms/posix/cmake"
)

set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

add_definitions(-DORB_COMMUNICATOR)
add_definitions(-DRELEASE_BUILD)

set(CONFIG_PARAM_SERVER "1")

add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=gnu99>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=gnu++14>)

add_compile_options( -Wno-array-bounds )

add_definitions( -D__PX4_LINUX )

link_directories(/home ${PX4_SOURCE_DIR}/boards/modalai/voxl2/lib)

include(CMakeParseArguments)

# Process Apps proc app source and libs
function (LINUX_APP)
	set(oneValueArgs APP_NAME APP_DEST)
	set(multiValueArgs SOURCES LINK_LIBS INCS)
	cmake_parse_arguments(LINUX_APP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${LINUX_APP_SOURCES}" STREQUAL "")
		message(FATAL_ERROR "LINUX_APP called without SOURCES")
	endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		)

	add_executable(${LINUX_APP_APP_NAME}
		${LINUX_APP_SOURCES}
		)

	if (NOT "${LINUX_APP_INCS}" STREQUAL "")
		target_include_directories(${LINUX_APP_APP_NAME} PUBLIC ${LINUX_APP_INCS})
	endif()

	target_link_libraries(${LINUX_APP_APP_NAME}
		${LINUX_APP_LINK_LIBS}
		)
endfunction()
