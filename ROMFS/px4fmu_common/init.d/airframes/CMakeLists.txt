############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

# Airframes live flat in this directory; each carries an `@class` header naming
# its vehicle class. A frame is added to the ROMFS iff the matching
# CONFIG_AIRFRAMES_<class> option is set (see ROMFS/Kconfig) -- inclusion is
# explicit per target and fully decoupled from which controllers are compiled in.
# Product/vendor frames carry `@class` too and ship with their class. A frame
# whose `@class` has no AIRFRAMES_<class> option (e.g. Autogyro) ships nowhere
# (intentionally unwired). HITL airframes (*.hil) ship under AIRFRAMES_SIMULATION
# regardless of the vehicle `@class` in their header.

file(GLOB airframe_files RELATIVE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/*)
list(REMOVE_ITEM airframe_files CMakeLists.txt)

set(selected_airframes)
foreach(frame ${airframe_files})
	if(frame MATCHES "\\.hil$")
		set(option AIRFRAMES_SIMULATION)
	else()
		file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/${frame} class_line LIMIT_COUNT 1 REGEX "@class")
		if(NOT class_line)
			continue() # no @class header -> not shipped
		endif()
		string(REGEX REPLACE "^.*@class" "" class_name "${class_line}")
		string(STRIP "${class_name}" class_name)

		if(class_name STREQUAL "Copter")
			set(option AIRFRAMES_COPTER)
		elseif(class_name STREQUAL "Plane")
			set(option AIRFRAMES_PLANE)
		elseif(class_name STREQUAL "VTOL")
			set(option AIRFRAMES_VTOL)
		elseif(class_name STREQUAL "Rover")
			set(option AIRFRAMES_ROVER)
		elseif(class_name STREQUAL "Underwater Robot")
			set(option AIRFRAMES_UUV)
		elseif(class_name STREQUAL "Spacecraft")
			set(option AIRFRAMES_SPACECRAFT)
		elseif(class_name STREQUAL "Airship")
			set(option AIRFRAMES_AIRSHIP)
		elseif(class_name STREQUAL "Balloon")
			set(option AIRFRAMES_BALLOON)
		else()
			continue() # @class with no AIRFRAMES_<class> option (e.g. Autogyro)
		endif()
	endif()

	if(CONFIG_${option})
		list(APPEND selected_airframes ${frame})
	endif()
endforeach()

if(selected_airframes)
	px4_add_romfs_files(${selected_airframes})
endif()
