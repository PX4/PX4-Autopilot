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

include(px4_base)

#=============================================================================
#
#	px4_add_common_flags
#
#	Set the default build flags.
#
#	Usage:
#		px4_add_common_flags(
#			BOARD <in-string>
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
#			OPTIMIZATION_FLAGS <inout-variable>
#			EXE_LINKER_FLAGS <inout-variable>
#			INCLUDE_DIRS <inout-variable>
#			LINK_DIRS <inout-variable>
#			DEFINITIONS <inout-variable>)
#
#	Input:
#		BOARD					: board
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
#	Example:
#		px4_add_common_flags(
#			BOARD px4_fmu-v2
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			OPTIMIZATION_FLAGS optimization_flags
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_add_common_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS OPTIMIZATION_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_add_common_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars}
		ARGN ${ARGN})

	set(warnings
		-Wall
		-Wextra
		-Werror

		-Warray-bounds
		-Wdisabled-optimization
		-Wdouble-promotion
		-Wfatal-errors
		-Wfloat-equal
		-Wformat-security
		-Winit-self
		-Wlogical-op
		-Wmissing-declarations
		-Wpointer-arith
		-Wshadow
		-Wuninitialized
		-Wunknown-pragmas
		-Wunused-variable

		-Wno-implicit-fallthrough # set appropriate level and update
		-Wno-missing-field-initializers
		-Wno-missing-include-dirs # TODO: fix and enable
		-Wno-unused-parameter
		)

	if (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		# QuRT 6.4.X compiler identifies as Clang but does not support this option
		if (NOT ${PX4_PLATFORM} STREQUAL "qurt")
			list(APPEND warnings
				-Qunused-arguments
				-Wno-unused-const-variable
				-Wno-varargs
				-Wno-address-of-packed-member
				-Wno-unknown-warning-option
				-Wunused-but-set-variable
			)
		endif()
	else()
		list(APPEND warnings
			-Wunused-but-set-variable
			-Wformat=1
		)
	endif()

	set(_optimization_flags
		-fno-strict-aliasing
		-fomit-frame-pointer

		-fno-math-errno
		-funsafe-math-optimizations

		-ffunction-sections
		-fdata-sections
		)

	set(c_warnings
		-Wbad-function-cast
		-Wstrict-prototypes
		-Wmissing-prototypes
		-Wnested-externs
		)

	set(c_compile_flags
		-g
		-std=gnu99
		-fno-common
		)

	set(cxx_warnings
		-Wno-overloaded-virtual # TODO: fix and remove
		-Wreorder
		)

	set(cxx_compile_flags
		-g
		-fno-exceptions
		-fno-rtti
		-std=gnu++11
		-fno-threadsafe-statics
		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		)

	# regular Clang or AppleClang
	if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		# force color for clang (needed for clang + ccache)
		list(APPEND _optimization_flags
			-fcolor-diagnostics
		)
	else()
		list(APPEND _optimization_flags
			-fno-strength-reduce
			-fno-builtin-printf
		)

		# -fcheck-new is a no-op for Clang in general
		# and has no effect, but can generate a compile
		# error for some OS
		list(APPEND cxx_compile_flags
			-fcheck-new
		)
	endif()

	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
			# force color for gcc > 4.9
			list(APPEND _optimization_flags
				-fdiagnostics-color=always
			)
		endif()

		list(APPEND cxx_warnings
			-Wno-format-truncation # TODO: fix
		)
	endif()

	set(visibility_flags
		-fvisibility=hidden
		-include visibility.h
		)

	set(added_c_flags
		${c_compile_flags}
		${warnings}
		${c_warnings}
		${visibility_flags}
		)

	set(added_cxx_flags
		${cxx_compile_flags}
		${warnings}
		${cxx_warnings}
		${visibility_flags}
		)

	set(added_optimization_flags
		${_optimization_flags}
		)

	include_directories(
		${PX4_BINARY_DIR}
		${PX4_BINARY_DIR}/src
		${PX4_BINARY_DIR}/src/lib
		${PX4_BINARY_DIR}/src/modules

		${PX4_SOURCE_DIR}/src
		${PX4_SOURCE_DIR}/src/include
		${PX4_SOURCE_DIR}/src/lib
		${PX4_SOURCE_DIR}/src/lib/DriverFramework/framework/include
		${PX4_SOURCE_DIR}/src/lib/matrix
		${PX4_SOURCE_DIR}/src/modules
		${PX4_SOURCE_DIR}/src/platforms
		)

	string(TOUPPER ${PX4_BOARD} board_upper)
	string(REPLACE "-" "_" board_config ${board_upper})

	add_definitions(
		-DCONFIG_ARCH_BOARD_${board_config}
		-D__STDC_FORMAT_MACROS
		)

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
		#message(STATUS "set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)")
	endforeach()

endfunction()
