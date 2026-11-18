############################################################################
#
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

include(px4_list_make_absolute)

#=============================================================================
#
#	px4_add_mavlink_extension
#
#	Adds MAVLink message handlers and streams to the mavlink module from
#	another module, without changing the mavlink module's sources.
#
#	Each handler class gets one instance per MAVLink instance, constructed
#	from that instance's Mavlink object, and its handle_message() receives
#	every message the instance receives, after the mavlink module's own
#	handlers. Each stream class joins the list of streams after the
#	mavlink module's own, so the mavlink stream command, the stream sets of
#	the MAVLink modes and MAV_CMD_SET_MESSAGE_INTERVAL reach it by name or
#	message ID like any other stream.
#
#	The mavlink module reads the registrations when it is configured. A
#	module configured after it can't register and stops the configuration;
#	out-of-tree modules (EXTERNAL_MODULES_LOCATION) are configured first.
#
#	SRCS compile as part of the mavlink module, which on NuttX joins its
#	sources into one file in its build directory. Each source's own
#	directory and the headers' directories join the module's include
#	directories, so a source's quoted includes resolve there too; file-static
#	and anonymous-namespace names in SRCS, and header names, must not collide
#	with the module's own.
#
#	Usage:
#		px4_add_mavlink_extension(
#			[HANDLER_HEADER <file>]
#			[HANDLERS <class> ...]
#			[STREAM_HEADER <file>]
#			[STREAMS <class> ...]
#			[SRCS <file> ...]
#			[INCLUDES <directory> ...]
#			)
#
#	Input:
#		HANDLER_HEADER	: header declaring HANDLERS; it is included where
#				  Mavlink is only declared, so the handlers'
#				  methods are defined in SRCS
#		HANDLERS	: classes with a constructor taking Mavlink & and a
#				  void handle_message(const mavlink_message_t *msg)
#		STREAM_HEADER	: header defining STREAMS, included after the
#				  mavlink module's own streams
#		STREAMS		: MavlinkStream classes with get_name_static(),
#				  get_id_static() and new_instance(), like the
#				  streams in src/modules/mavlink/streams
#		SRCS		: sources compiled into the mavlink module
#		INCLUDES	: further include directories for those sources and
#				  headers, beyond their own directories
#
#	Example:
#		px4_add_mavlink_extension(
#			HANDLER_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/MyHandler.hpp
#			HANDLERS MyHandler
#			STREAM_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/MY_MESSAGE.hpp
#			STREAMS MavlinkStreamMyMessage
#			SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MyHandler.cpp
#			)
#
function(px4_add_mavlink_extension)

	px4_parse_function_args(
		NAME px4_add_mavlink_extension
		ONE_VALUE HANDLER_HEADER STREAM_HEADER
		MULTI_VALUE HANDLERS STREAMS SRCS INCLUDES
		ARGN ${ARGN})

	get_property(configured GLOBAL PROPERTY PX4_MAVLINK_EXTENSIONS_CONFIGURED)
	if(configured)
		message(FATAL_ERROR "px4_add_mavlink_extension: the mavlink module is configured already, so it can't see this registration")
	endif()

	if((HANDLERS AND NOT HANDLER_HEADER) OR (STREAMS AND NOT STREAM_HEADER))
		message(FATAL_ERROR "px4_add_mavlink_extension: HANDLERS need a HANDLER_HEADER and STREAMS a STREAM_HEADER")
	endif()

	if(NOT HANDLERS AND NOT STREAMS)
		message(FATAL_ERROR "px4_add_mavlink_extension: name at least one of HANDLERS or STREAMS")
	endif()

	px4_list_make_absolute(SRCS ${CMAKE_CURRENT_SOURCE_DIR} ${SRCS})
	px4_list_make_absolute(INCLUDES ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDES})
	set(headers)

	if(HANDLERS)
		get_filename_component(HANDLER_HEADER ${HANDLER_HEADER} ABSOLUTE)
		set_property(GLOBAL APPEND PROPERTY PX4_MAVLINK_EXTENSION_HANDLER_HEADERS ${HANDLER_HEADER})
		set_property(GLOBAL APPEND PROPERTY PX4_MAVLINK_EXTENSION_HANDLERS ${HANDLERS})
		list(APPEND headers ${HANDLER_HEADER})
	endif()

	if(STREAMS)
		get_filename_component(STREAM_HEADER ${STREAM_HEADER} ABSOLUTE)
		set_property(GLOBAL APPEND PROPERTY PX4_MAVLINK_EXTENSION_STREAM_HEADERS ${STREAM_HEADER})
		set_property(GLOBAL APPEND PROPERTY PX4_MAVLINK_EXTENSION_STREAMS ${STREAMS})
		list(APPEND headers ${STREAM_HEADER})
	endif()

	# A unity build compiles the sources from the module's build directory, where their quoted
	# includes would not find the files next to them
	foreach(file ${SRCS} ${headers})
		get_filename_component(directory ${file} DIRECTORY)
		list(APPEND INCLUDES ${directory})
	endforeach()
	list(REMOVE_DUPLICATES INCLUDES)

	set_property(GLOBAL APPEND PROPERTY PX4_MAVLINK_EXTENSION_SRCS ${SRCS})
	set_property(GLOBAL APPEND PROPERTY PX4_MAVLINK_EXTENSION_INCLUDES ${INCLUDES})

endfunction()
