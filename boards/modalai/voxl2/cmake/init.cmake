############################################################################
#
#   Copyright (c) 2024 ModalAI, Inc. All rights reserved.
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

# Initialize libfc-sensor-api submodule (fetches from GitLab if not present)
execute_process(
	COMMAND Tools/check_submodules.sh boards/modalai/voxl2/libfc-sensor-api
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
)

include_directories(${PX4_BOARD_DIR}/libfc-sensor-api/inc)

# Build libfc_sensor.so stub library automatically if not already built
set(FC_SENSOR_LIB ${PX4_BOARD_DIR}/libfc-sensor-api/build/libfc_sensor.so)
if(NOT EXISTS ${FC_SENSOR_LIB})
	execute_process(
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BOARD_DIR}/libfc-sensor-api/build
	)
	execute_process(
		COMMAND ${CMAKE_COMMAND} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} ..
		WORKING_DIRECTORY ${PX4_BOARD_DIR}/libfc-sensor-api/build
		RESULT_VARIABLE FC_SENSOR_CMAKE_RESULT
	)
	if(NOT FC_SENSOR_CMAKE_RESULT EQUAL 0)
		message(FATAL_ERROR "Failed to configure libfc_sensor stub library")
	endif()
	execute_process(
		COMMAND ${CMAKE_COMMAND} --build .
		WORKING_DIRECTORY ${PX4_BOARD_DIR}/libfc-sensor-api/build
		RESULT_VARIABLE FC_SENSOR_BUILD_RESULT
	)
	if(NOT FC_SENSOR_BUILD_RESULT EQUAL 0)
		message(FATAL_ERROR "Failed to build libfc_sensor stub library")
	endif()
endif()
