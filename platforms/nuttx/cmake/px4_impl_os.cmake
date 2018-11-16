############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
#	Required OS Interface Functions
#
#		* px4_os_add_flags
#		* px4_os_prebuild_targets
#

include(px4_base)

#=============================================================================
#
#	px4_os_add_flags
#
#	Set the nuttx build flags.
#
#	Usage:
#		px4_os_add_flags(
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
#			OPTIMIZATION_FLAGS <inout-variable>
#			EXE_LINKER_FLAGS <inout-variable>
#			INCLUDE_DIRS <inout-variable>
#			LINK_DIRS <inout-variable>
#			DEFINITIONS <inout-variable>)
#
#	Input:
#		BOARD					: flags depend on board/nuttx config
#
#	Input/Output: (appends to existing variable)
#		C_FLAGS					: c compile flags variable
#		CXX_FLAGS				: c++ compile flags variable
#		OPTIMIZATION_FLAGS			: optimization compile flags variable
#		EXE_LINKER_FLAGS			: executable linker flags variable
#		INCLUDE_DIRS				: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Note that EXE_LINKER_FLAGS is not suitable for adding libraries because
#	these flags are added before any of the object files and static libraries.
#	Add libraries in src/firmware/nuttx/CMakeLists.txt.
#
#	Example:
#		px4_os_add_flags(
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			OPTIMIZATION_FLAGS optimization_flags
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_os_add_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS OPTIMIZATION_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_os_add_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars}
		ARGN ${ARGN})

	px4_add_common_flags(
		BOARD ${PX4_BOARD}
		C_FLAGS ${C_FLAGS}
		CXX_FLAGS ${CXX_FLAGS}
		OPTIMIZATION_FLAGS ${OPTIMIZATION_FLAGS}
		EXE_LINKER_FLAGS ${EXE_LINKER_FLAGS}
		INCLUDE_DIRS ${INCLUDE_DIRS}
		LINK_DIRS ${LINK_DIRS}
		DEFINITIONS ${DEFINITIONS})

	include_directories(BEFORE SYSTEM
		${PX4_BINARY_DIR}/NuttX/nuttx/include
		${PX4_BINARY_DIR}/NuttX/nuttx/include/cxx
	)

	include_directories(
		${PX4_BINARY_DIR}/NuttX/nuttx/arch/${CONFIG_ARCH}/src/${CONFIG_ARCH_FAMILY}
		${PX4_BINARY_DIR}/NuttX/nuttx/arch/${CONFIG_ARCH}/src/chip
		${PX4_BINARY_DIR}/NuttX/nuttx/arch/${CONFIG_ARCH}/src/common

		${PX4_BINARY_DIR}/NuttX/apps/include
		)

	add_definitions(
		-D__PX4_NUTTX
		-D__DF_NUTTX
		)

	if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
		message(STATUS "NuttX Stack Checking (CONFIG_ARMV7M_STACKCHECK) enabled")
		set(instrument_flags
			-finstrument-functions
			-ffixed-r10
			)
		list(APPEND c_flags ${instrument_flags})
		list(APPEND cxx_flags ${instrument_flags})
	endif()

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
		#message(STATUS "nuttx: set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)")
	endforeach()
endfunction()

#=============================================================================
#
#	px4_os_prebuild_targets
#
#	This function generates os dependent targets
#
#	Usage:
#		px4_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		px4_os_prebuild_targets(OUT target_list BOARD px4fmu-v2)
#
function(px4_os_prebuild_targets)
	px4_parse_function_args(
			NAME px4_os_prebuild_targets
			ONE_VALUE OUT BOARD
			REQUIRED OUT
			ARGN ${ARGN})

	add_library(prebuild_targets INTERFACE)
	target_link_libraries(prebuild_targets INTERFACE nuttx_cxx nuttx_c nuttx_fs nuttx_mm nuttx_sched m gcc)
	add_dependencies(prebuild_targets DEPENDS nuttx_context uorb_headers)

	# parse nuttx config options for cmake
	file(STRINGS ${PX4_BOARD_DIR}/nuttx-config/${NUTTX_CONFIG}/defconfig ConfigContents)
	foreach(NameAndValue ${ConfigContents})
		# Strip leading spaces
		string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

		# Find variable name
		string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

		if (Name)
			# Find the value
			string(REPLACE "${Name}=" "" Value ${NameAndValue})

			# remove extra quotes
			string(REPLACE "\"" "" Value ${Value})

			# Set the variable
			#message(STATUS "${Name} ${Value}")
			set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
		endif()
	endforeach()
endfunction()
