############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

# find PX4 config
#  look for in tree board config that matches CONFIG input

if(NOT CONFIG)
	# default to px4_ros2 if building within a ROS2 colcon environment
	if(("$ENV{COLCON}" MATCHES "1") AND ("$ENV{ROS_VERSION}" MATCHES "2"))
		set(CONFIG "px4_ros2" CACHE STRING "desired configuration")
	else()
		set(CONFIG "px4_sitl" CACHE STRING "desired configuration")
	endif()
else()
	# Promote CONFIG from UNINITIALIZED (set by -D on command line) to STRING
	# so that cmake -L lists it. The Makefile's cmake-cache-check relies on
	# finding CONFIG in cmake -L output to avoid unnecessary reconfiguration.
	set(CONFIG "${CONFIG}" CACHE STRING "desired configuration" FORCE)
endif()

# The set of target-class names (the buildable role of a board). A bare board
# name resolves to its sole class; a board with several is ambiguous.
set(PX4_CLASS_NAMES copter fixedwing vtol rover uuv spacecraft airship cannode linux sitl io ros2)

if(NOT PX4_CONFIG_FILE)

	file(GLOB_RECURSE board_configs
		RELATIVE "${PX4_SOURCE_DIR}/boards"
		"boards/*.px4board"
		)

	# Resolve the requested CONFIG to a board config fragment:
	#   <vendor>_<model>_<label>  selects boards/<vendor>/<model>/<label>.px4board
	#   <vendor>_<model>          bare board name -> the board's SOLE target class
	# The 'base' foundation fragment is not buildable, and the 'default' label is
	# retired (replaced by per-class targets).
	set(_resolved FALSE)

	# Pass 1: exact <vendor>_<model>_<label> match (label may contain '.' or '-')
	foreach(filename ${board_configs})
		string(REPLACE ".px4board" "" filename_stripped ${filename})
		string(REPLACE "/" ";" config ${filename_stripped})
		list(LENGTH config config_len)

		if(${config_len} EQUAL 3)
			list(GET config 0 vendor)
			list(GET config 1 model)
			list(GET config 2 label)

			if(CONFIG STREQUAL "${vendor}_${model}_${label}")
				if((label STREQUAL "base") OR (label STREQUAL "default"))
					message(FATAL_ERROR "'${CONFIG}': '${label}' is not a buildable target; "
						"build a target class instead (e.g. ${vendor}_${model}_<class>).")
				endif()
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				set(PX4_BOARD_DIR "${PX4_SOURCE_DIR}/boards/${vendor}/${model}" CACHE STRING "PX4 board directory" FORCE)
				set(MODEL "${model}" CACHE STRING "PX4 board model" FORCE)
				set(VENDOR "${vendor}" CACHE STRING "PX4 board vendor" FORCE)
				set(LABEL "${label}" CACHE STRING "PX4 board label" FORCE)
				set(_resolved TRUE)
				break()
			endif()
		endif()
	endforeach()

	# Pass 2: bare <vendor>_<model> -> the board's sole target class
	if(NOT _resolved)
		foreach(filename ${board_configs})
			string(REPLACE ".px4board" "" filename_stripped ${filename})
			string(REPLACE "/" ";" config ${filename_stripped})
			list(LENGTH config config_len)

			if(${config_len} EQUAL 3)
				list(GET config 0 vendor)
				list(GET config 1 model)

				if(CONFIG STREQUAL "${vendor}_${model}")
					# collect the board's target-class overlays
					file(GLOB _board_files RELATIVE "${PX4_SOURCE_DIR}/boards/${vendor}/${model}"
						"${PX4_SOURCE_DIR}/boards/${vendor}/${model}/*.px4board")
					set(_classes)
					foreach(_bf ${_board_files})
						string(REPLACE ".px4board" "" _bl ${_bf})
						if(_bl IN_LIST PX4_CLASS_NAMES)
							list(APPEND _classes ${_bl})
						endif()
					endforeach()
					list(LENGTH _classes _nclasses)

					if(_nclasses EQUAL 1)
						set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${vendor}/${model}/${_classes}.px4board" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
						set(PX4_BOARD_DIR "${PX4_SOURCE_DIR}/boards/${vendor}/${model}" CACHE STRING "PX4 board directory" FORCE)
						set(MODEL "${model}" CACHE STRING "PX4 board model" FORCE)
						set(VENDOR "${vendor}" CACHE STRING "PX4 board vendor" FORCE)
						set(LABEL "${_classes}" CACHE STRING "PX4 board label" FORCE)
						set(_resolved TRUE)
					elseif(_nclasses GREATER 1)
						list(SORT _classes)
						string(REPLACE ";" ", " _classlist "${_classes}")
						message(FATAL_ERROR "'${CONFIG}' is ambiguous: ${vendor}/${model} has multiple "
							"target classes (${_classlist}); specify one, e.g. ${CONFIG}_vtol.")
					else()
						message(FATAL_ERROR "'${CONFIG}': board ${vendor}/${model} defines no target class.")
					endif()
					break()
				endif()
			endif()
		endforeach()
	endif()

	if(NOT _resolved)
		message(FATAL_ERROR "PX4 config '${CONFIG}' not found.")
	endif()
endif()

message(STATUS "PX4 config file: ${PX4_CONFIG_FILE}")

include_directories(${PX4_BOARD_DIR}/src)

set(PX4_BOARD ${VENDOR}_${MODEL} CACHE STRING "PX4 board" FORCE)

# board name is uppercase with no underscores when used as a define
string(TOUPPER ${PX4_BOARD} PX4_BOARD_NAME)
string(REPLACE "-" "_" PX4_BOARD_NAME ${PX4_BOARD_NAME})
set(PX4_BOARD_NAME ${PX4_BOARD_NAME} CACHE STRING "PX4 board define" FORCE)

set(PX4_BOARD_VENDOR ${VENDOR} CACHE STRING "PX4 board vendor" FORCE)
set(PX4_BOARD_MODEL ${MODEL} CACHE STRING "PX4 board model" FORCE)

set(PX4_BOARD_LABEL ${LABEL} CACHE STRING "PX4 board label" FORCE)

# Determine the target class that drives the config merge in kconfig.cmake:
#   <class> or <class>.<variant>                 -> the leading dot-component
#   bootloader / canbootloader / performance-test -> none (standalone savedefconfig)
#   a bare variant (e.g. 'test')                  -> the board's sole target class
set(PX4_TARGET_CLASS "")
if(LABEL MATCHES "^(bootloader|canbootloader|performance-test)")
	# standalone savedefconfig labels carry a complete config and no class
elseif(LABEL MATCHES "\\.")
	string(REGEX REPLACE "\\..*" "" PX4_TARGET_CLASS "${LABEL}")
elseif(LABEL IN_LIST PX4_CLASS_NAMES)
	set(PX4_TARGET_CLASS "${LABEL}")
else()
	# bare variant label: attach it to the board's sole target class
	file(GLOB _board_files RELATIVE "${PX4_BOARD_DIR}" "${PX4_BOARD_DIR}/*.px4board")
	set(_classes)
	foreach(_bf ${_board_files})
		string(REPLACE ".px4board" "" _bl ${_bf})
		if(_bl IN_LIST PX4_CLASS_NAMES)
			list(APPEND _classes ${_bl})
		endif()
	endforeach()
	list(LENGTH _classes _nclasses)
	if(_nclasses EQUAL 1)
		set(PX4_TARGET_CLASS "${_classes}")
	else()
		message(FATAL_ERROR "'${CONFIG}': variant label '${LABEL}' on ${VENDOR}/${MODEL} cannot be "
			"mapped to a single target class.")
	endif()
endif()
set(PX4_TARGET_CLASS "${PX4_TARGET_CLASS}" CACHE STRING "PX4 target class" FORCE)

set(PX4_CONFIG "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}" CACHE STRING "PX4 config" FORCE)

if(EXISTS "${PX4_BOARD_DIR}/uavcan_board_identity")
	include ("${PX4_BOARD_DIR}/uavcan_board_identity")
endif()

if(EXISTS "${PX4_BOARD_DIR}/sitl.cmake")
	include ("${PX4_BOARD_DIR}/sitl.cmake")
endif()
