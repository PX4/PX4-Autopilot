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
# CMAKE CODING STANDARD FOR PX4
#
# Structure
# ---------------------------------------------------------------------------
#
# * Common functions should be included in px_utils.cmake.
#
# * OS/ board specific fucntions should be include in
#	px_${OS}_utils.cmake or px4_${OS}_${BOARD}_utils.cmake.
#
# Formatting
# ---------------------------------------------------------------------------
#
# * Use hard indents to match the px4 source code.
#
# * All function and script arguments are upper case.
#
# * All local variables are lower case.
#
# * All cmake functions are lowercase.
#
# Functions/Macros
# ---------------------------------------------------------------------------
#
# * Never use macros. They allow overwriting global variables and this
#	makes variable declarations hard to locate.
#
# * If a target from add_custom_* is set in a function, explicitly pass it
#	as an output argument so that the target name is clear to the user.
#
# * Avoid use of global variables in functions. Functions in a nested 
#	scope may use global variables, but this makes it difficult to
#	resuse functions.
#
# Included CMake Files
# ---------------------------------------------------------------------------
#
# * Never set global variables in an included cmake file,
#	you may only define functions. This excludes Toolchain files. This
#	makes it clear to the user when variables are being set or targets
#	are being created.
#
# * All toolchain files should be included in the cmake
#	directory and named Toolchain-"name".cmake.
#
# Misc
# ---------------------------------------------------------------------------
#
# * If referencing a string variable, don't put it in quotes.
#	Don't do "${OS}" STREQUAL "posix",
#	instead type ${OS} STREQUAL "posix". This will throw an
#	error when ${OS} is not defined instead of silently
#	evaluating to false.
#
#=============================================================================


cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(px4 CXX C ASM)
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0045 NEW) # error on non-existent target in get prop
	cmake_policy(SET CMP0046 NEW) # no non-existent targets as dependencies
endif()

set(version_major 0)
set(version_minor 1)
set(verion_patch 2)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "px4users@googlegroups.com")

#=============================================================================
# cmake modules
#

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(px4_utils)
include(px4_nuttx_utils)

#=============================================================================
# parameters
#

set(OS "posix" CACHE STRING "desired operating system")
set_property(CACHE OS PROPERTY STRINGS nuttx posix qurt)

set(BOARD "sitl" CACHE STRING "target board")
set_property(CACHE BOARD PROPERTY STRINGS px4fmu-v2 sitl)

set(LABEL "simple" CACHE STRING "module set label")
set_property(CACHE LABEL PROPERTY STRINGS simple default)

set(NUTTX_BUILD_THREADS "4" CACHE STRING
	"number of threads to use when building NuttX")

set(required_toolchain_variables
	CMAKE_C_COMPILER_ID
	)

#=============================================================================
# check required toolchain variables
#
foreach(var ${required_toolchain_variables})
	if (NOT ${var})
		message(FATAL_ERROR "Toolchain must define ${var}")
	endif()
endforeach()

#=============================================================================
# git
#
px4_add_git_submodule(TARGET git_nuttx PATH "NuttX")
px4_add_git_submodule(TARGET git_genmsg PATH "Tools/genmsg")
px4_add_git_submodule(TARGET git_gencpp PATH "Tools/gencpp")
px4_add_git_submodule(TARGET git_mavlink PATH "mavlink/include/mavlink/v1.0")
px4_add_git_submodule(TARGET git_gtest PATH "unittets/gtest")
px4_add_git_submodule(TARGET git_eigen PATH "src/lib/eigen")

add_custom_target(submodule_clean
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND git submodule deinit -f .
	COMMAND rm -rf .git/modules/*
	)

#=============================================================================
# external libraries
#
if(${OS} STREQUAL "nuttx")
	px4_nuttx_add_export(OUT nuttx_export
		CONFIG ${BOARD}
		THREADS ${NUTTX_BUILD_THREADS}
		DEPENDS git_nuttx)
endif()

#=============================================================================
# build flags
#

px4_add_common_flags(
	C_FLAGS c_flags
	CXX_FLAGS cxx_flags
	EXE_LINKER_FLAGS exe_linker_flags
	INCLUDE_DIRS include_dirs
	LINK_DIRS link_dirs
	DEFINITIONS definitions)

if(${OS} STREQUAL "nuttx")
	px4_add_nuttx_flags(
		BOARD ${BOARD}
		C_FLAGS c_flags
		CXX_FLAGS cxx_flags
		EXE_LINKER_FLAGS exe_linker_flags
		INCLUDE_DIRS include_dirs
		LINK_DIRS link_dirs
		DEFINITIONS definitions)

endif()

px4_join(OUT CMAKE_EXE_LINKER_FLAGS LIST "${exe_linker_flags}" GLUE " ")
px4_join(OUT CMAKE_C_FLAGS LIST "${c_flags}" GLUE " ")
px4_join(OUT CMAKE_CXX_FLAGS LIST "${cxx_flags}" GLUE " ")
include_directories(${include_dirs})
link_directories(${link_dirs})
add_definitions(${definitions})

#=============================================================================
# source code generation
#
file(GLOB_RECURSE msg_files msg/*.msg)
px4_generate_messages(TARGET msg_gen
	MSG_FILES ${msg_files}
	OS ${OS}
	DEPENDS git_genmsg git_gencpp
	)

#=============================================================================
# subdirectories
#
add_subdirectory(src)

#=============================================================================
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_VERSION ${version})
set(CPACK_PACKAGE_CONTACT ${package_contact})
include(CPack)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
