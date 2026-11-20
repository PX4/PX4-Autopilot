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
#	Makes an out-of-tree dialect XML the MAVLink dialect of this build.
#	The XML must <include>common.xml</include> (or another upstream dialect).
#	Only one external dialect can be registered; a second module's messages
#	belong in an XML that the registered one includes.
#
#	mavgen resolves <include> relative to the dialect file, so the upstream
#	definitions are staged next to the copy in the build tree (by the mavlink
#	module at build time, once the submodule is checked out); nothing is
#	written into the source tree.
#
#	Usage:
#		px4_add_external_mavlink_dialect(XML ${CMAKE_CURRENT_SOURCE_DIR}/../../mavlink/my_dialect.xml)
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

	get_property(_registered GLOBAL PROPERTY PX4_EXTERNAL_MAVLINK_DIALECT)
	if(_registered)
		message(FATAL_ERROR "px4_add_external_mavlink_dialect: '${_registered}' is already registered; <include> ${XML} from ${_registered}.xml instead")
	endif()

	get_filename_component(_dialect_name "${XML}" NAME_WE)
	set(_staging_dir "${PX4_BINARY_DIR}/mavlink/message_definitions/v1.0")

	configure_file("${XML}" "${_staging_dir}/${_dialect_name}.xml" COPYONLY)

	set_property(GLOBAL PROPERTY PX4_EXTERNAL_MAVLINK_DIALECT "${_dialect_name}")
	set_property(GLOBAL PROPERTY PX4_EXTERNAL_MAVLINK_DIALECT_DIR "${_staging_dir}")

	message(STATUS "External MAVLink dialect: ${_dialect_name} (${XML})")
endfunction()

#=============================================================================
#
#	px4_target_use_external_mavlink_dialect
#
#	Gives an out-of-tree module target the generated headers of the dialect
#	registered with px4_add_external_mavlink_dialect(). External modules are
#	configured before src/modules/mavlink, so the generator target is named
#	rather than linked.
#
#	Usage:
#		px4_target_use_external_mavlink_dialect(modules__my_module)
#
function(px4_target_use_external_mavlink_dialect target)
	get_property(_dialect GLOBAL PROPERTY PX4_EXTERNAL_MAVLINK_DIALECT)
	if(NOT _dialect)
		message(FATAL_ERROR "px4_target_use_external_mavlink_dialect: call px4_add_external_mavlink_dialect() first")
	endif()

	target_include_directories(${target} PRIVATE
		${PX4_BINARY_DIR}/mavlink
		${PX4_BINARY_DIR}/mavlink/${_dialect}
		${PX4_BINARY_DIR}/mavlink/uAvionix
	)
	target_compile_options(${target} PRIVATE -Wno-address-of-packed-member -Wno-cast-align)
	add_dependencies(${target} mavlink_c_generate)
endfunction()
