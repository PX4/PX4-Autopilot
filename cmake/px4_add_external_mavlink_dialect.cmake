############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#	px4_add_external_mavlink_dialect
#
#	Registers an external MAVLink dialect XML for mavgen code generation.
#	The dialect XML should <include>common.xml</include> (or another base
#	dialect) so that all standard MAVLink messages remain available.
#
#	Multiple external dialects are supported. The first registered dialect
#	becomes the primary dialect (overrides CONFIG_MAVLINK_DIALECT from
#	"common" if applicable).
#
#	Usage:
#		px4_add_external_mavlink_dialect(
#			XML ${CMAKE_CURRENT_SOURCE_DIR}/../../mavlink/se05x.xml
#		)
#
#	Effects:
#		1. Copies the XML into mavgen's message_definitions/v1.0/ search path
#		2. Appends the dialect name to PX4_EXTERNAL_MAVLINK_DIALECTS global property
#		3. Dialect override happens in root CMakeLists.txt after all add_subdirectory()
#		   calls have completed
#
function(px4_add_external_mavlink_dialect)
	px4_parse_function_args(
		NAME px4_add_external_mavlink_dialect
		ONE_VALUE XML
		REQUIRED XML
		ARGN ${ARGN}
	)

	if(NOT EXISTS "${XML}")
		message(FATAL_ERROR "px4_add_external_mavlink_dialect: XML not found: ${XML}")
	endif()

	get_filename_component(_dialect_name "${XML}" NAME_WE)
	set(_mavlink_defs "${PX4_SOURCE_DIR}/src/modules/mavlink/mavlink/message_definitions/v1.0")

	configure_file("${XML}" "${_mavlink_defs}/${_dialect_name}.xml" COPYONLY)

	set_property(GLOBAL APPEND PROPERTY PX4_EXTERNAL_MAVLINK_DIALECTS "${_dialect_name}")

	message(STATUS "External MAVLink dialect registered: ${_dialect_name} (from ${XML})")
endfunction()
