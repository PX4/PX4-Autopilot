############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
#	px4_add_library
#
#	Like add_library but with PX4 platform dependencies
#
function(px4_add_library)

	px4_parse_function_args(
			NAME px4_add_module
			ONE_VALUE LIBRARY
			MULTI_VALUE SRCS LIBRARY_CONFIG
			REQUIRED LIBRARY
			ARGN ${ARGN})

	add_library(${LIBRARY} EXCLUDE_FROM_ALL ${SRCS})

	target_compile_definitions(${LIBRARY} PRIVATE MODULE_NAME="${LIBRARY}")

	# all PX4 libraries have access to parameters and uORB
	add_dependencies(${LIBRARY} uorb_headers)
	target_link_libraries(${LIBRARY} PRIVATE prebuild_targets parameters_interface px4_platform uorb_msgs)

	# TODO: move to platform layer
	if ("${PX4_PLATFORM}" MATCHES "nuttx")
		target_link_libraries(${LIBRARY} PRIVATE m nuttx_c)
	endif()

	set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})

	if(LIBRARY_CONFIG)
		foreach(module_config ${LIBRARY_CONFIG})
			set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${module_config})
		endforeach()
	endif()
endfunction()
