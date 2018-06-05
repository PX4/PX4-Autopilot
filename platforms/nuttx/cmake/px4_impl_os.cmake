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
#	OS Specific Functions
#
#		* px4_nuttx_make_uavcan_bootloadable
#
#	Required OS Inteface Functions
#
#		* px4_os_add_flags
#		* px4_os_prebuild_targets
#

include(common/px4_base)

#=============================================================================
#
#	px4_nuttx_make_uavcan_bootloadable
#
#	This function adds a uavcan boot loadable target.
#
#	Usage:
#	  px4_nuttx_make_uavcan_bootloadable(
#	   BOARD	<board>
#	   BIN <input bin file>)
#	   HWNAME <uavcan name>
#	   HW_MAJOR <number>
#	   HW_MINOR <number>
#	   SW_MAJOR <number>
#	   SW_MINOR <number>)
#
#	Input:
#	  BOARD	     : the board
#	  BIN	     : the bin file to generate the bootloadable image from
#	  HWNAME     : the uavcan name
#	  HW_MAJOR   : the major hardware revision
#	  HW_MINOR   : the minor hardware revision
#	  SW_MAJOR   : the major software revision
#	  SW_MINOR   : the minor software revision
#
#	Output:
#		OUT			: None
#
#	Example:
#	px4_nuttx_make_uavcan_bootloadable(
#	  BOARD ${BOARD}
#		BIN ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
#		HWNAME ${uavcanblid_name}
#		HW_MAJOR ${uavcanblid_hw_version_major}
#		HW_MINOR ${uavcanblid_hw_version_minor}
#		SW_MAJOR ${uavcanblid_sw_version_major}
#		SW_MINOR ${uavcanblid_sw_version_minor}
#	 )
#
function(px4_nuttx_make_uavcan_bootloadable)
	px4_parse_function_args(
		NAME px4_nuttx_make_uavcan_bootloadable
		ONE_VALUE BOARD BIN HWNAME HW_MAJOR HW_MINOR SW_MAJOR SW_MINOR
		REQUIRED BOARD BIN HWNAME HW_MAJOR HW_MINOR SW_MAJOR SW_MINOR
		ARGN ${ARGN})

	string(REPLACE "\"" "" HWNAME ${HWNAME})

	execute_process(
		COMMAND git rev-list HEAD --max-count=1 --abbrev=8 --abbrev-commit
		OUTPUT_VARIABLE uavcanbl_git_desc
		OUTPUT_STRIP_TRAILING_WHITESPACE
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

	if ("${uavcanbl_git_desc}" STREQUAL "")
		set(uavcanbl_git_desc ffffffff)
	endif()
	set(uavcan_bl_imange_name ${HWNAME}-${HW_MAJOR}.${HW_MINOR}-${SW_MAJOR}.${SW_MINOR}.${uavcanbl_git_desc}.uavcan.bin)
	message(STATUS "Generating UAVCAN Bootable as ${uavcan_bl_imange_name}")
	add_custom_command(OUTPUT ${uavcan_bl_imange_name}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/make_can_boot_descriptor.py
			-v --use-git-hash ${BIN} ${uavcan_bl_imange_name}
		DEPENDS ${BIN}
		)
	add_custom_target(build_uavcan_bl_${BOARD} ALL DEPENDS ${uavcan_bl_imange_name})
endfunction()

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
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

	px4_add_common_flags(
		BOARD ${BOARD}
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


	if("${config_nuttx_hw_stack_check_${BOARD}}" STREQUAL "y")
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

#	Usage:
#		px4_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#		THREADS		: number of threads for building
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
			ONE_VALUE OUT BOARD THREADS
			REQUIRED OUT BOARD
			ARGN ${ARGN})

	add_library(${OUT} INTERFACE)
	target_link_libraries(${OUT} INTERFACE nuttx_cxx nuttx_c nuttx_fs nuttx_mm nuttx_sched m gcc)
	add_dependencies(${OUT} DEPENDS nuttx_context uorb_headers)

	# parse nuttx config options for cmake
	file(STRINGS ${PX4_SOURCE_DIR}/platforms/nuttx/nuttx-configs/${BOARD}/nsh/defconfig ConfigContents)
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
			set(${Name} ${Value} PARENT_SCOPE)
		endif()
	endforeach()
endfunction()

#=============================================================================
#
#	px4_nuttx_configure
#
#	This function sets the nuttx configuration
#
#	Usage:
#		px4_nuttx_configure(
#	    HWCLASS <m3|m4>
#		  [ROMFS <y|n>
#		  ROMFSROOT <root>]
#			)
#
#	Input:
#	  HWCLASS		: the class of hardware
#	  CONFIG		: the nuttx configuration to use
#	  ROMFS			: whether or not to use incllude theROMFS
#	  ROMFSROOT		: If ROMFS used set the root the default is px4fmu_common
#
#	Output:
#		OUT	: None
#
#	Example:
#		px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y)
#
function(px4_nuttx_configure)
	px4_parse_function_args(
			NAME px4_nuttx_configure
			ONE_VALUE HWCLASS CONFIG ROMFS ROMFSROOT IO
			REQUIRED HWCLASS
			ARGN ${ARGN})

	# HWCLASS -> CMAKE_SYSTEM_PROCESSOR
	if(HWCLASS STREQUAL "m7")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-m7" PARENT_SCOPE)
	elseif(HWCLASS STREQUAL "m4")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-m4" PARENT_SCOPE)
	elseif(HWCLASS STREQUAL "m3")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-m3" PARENT_SCOPE)
	endif()
	set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL "system processor" FORCE)
	set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake CACHE INTERNAL "toolchain file" FORCE)

	# ROMFS
	if("${ROMFS}" STREQUAL "y")
		if (NOT DEFINED ROMFSROOT)
			set(config_romfs_root px4fmu_common)
		else()
			set(config_romfs_root ${ROMFSROOT})
		endif()
		set(config_romfs_root ${config_romfs_root} PARENT_SCOPE)
	endif()

	# IO board placed in ROMFS
	if(config_romfs_root)
		set(config_io_board ${IO} PARENT_SCOPE)
	endif()
endfunction()

