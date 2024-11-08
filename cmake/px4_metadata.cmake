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
#		* px4_generate_airframes_xml
#

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

include(px4_airframes)

set(EXCLUDED_AIRFRAMES NULL)

if(NOT CONFIG_MODULES_SIMULATION_PWM_OUT_SIM)
	list(APPEND EXCLUDED_AIRFRAMES ${SIMULATION_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_MC_RATE_CONTROL)
	list(APPEND EXCLUDED_AIRFRAMES ${MULTICOPTER_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_FW_RATE_CONTROL)
	list(APPEND EXCLUDED_AIRFRAMES ${FIXEDWING_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_AIRSHIP_ATT_CONTROL)
	list(APPEND EXCLUDED_AIRFRAMES ${AIRSHIP_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_VTOL_ATT_CONTROL)
	list(APPEND EXCLUDED_AIRFRAMES ${VTOL_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_ROVER_DIFFERENTIAL)
	list(APPEND EXCLUDED_AIRFRAMES ${DIFFERENTIAL_ROVER_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_ROVER_ACKERMANN)
	list(APPEND EXCLUDED_AIRFRAMES ${ACKERMANN_ROVER_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_ROVER_POS_CONTROL)
	list(APPEND EXCLUDED_AIRFRAMES ${ROVER_AIRFRAMES})
endif()

if(NOT CONFIG_MODULES_UUV_ATT_CONTROL)
	list(APPEND EXCLUDED_AIRFRAMES ${UUV_AIRFRAMES})
endif()

function(px4_generate_airframes_xml)
	px4_parse_function_args(
		NAME px4_generate_airframes_xml
		ONE_VALUE BOARD
		REQUIRED BOARD
		ARGN ${ARGN})

	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/airframes.xml
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
			--airframes-path ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d
			--board CONFIG_ARCH_BOARD_${PX4_BOARD}
			--xml ${PX4_BINARY_DIR}/airframes.xml
			--excluded_airframes ${EXCLUDED_AIRFRAMES}
		DEPENDS ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		COMMENT "Creating airframes.xml"
		)
	add_custom_target(airframes_xml DEPENDS ${PX4_BINARY_DIR}/airframes.xml)
endfunction()
