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

if(NOT CONFIG_PX4)
	return()
endif()

set(px4_source_dir ${ZEPHYR_CURRENT_MODULE_DIR})

if(NOT CONFIG_PX4_BOARD_CONFIG STREQUAL "")
	set(px4_config ${CONFIG_PX4_BOARD_CONFIG})
elseif(EXISTS ${px4_source_dir}/boards/zephyr/${BOARD}/default.px4board)
	set(px4_config zephyr_${BOARD})
else()
	set(px4_config zephyr_generic)
endif()

set(CONFIG ${px4_config} CACHE STRING "PX4 board configuration" FORCE)
unset(PX4_CONFIG_FILE CACHE)

set(PX4_GIT_SUBMODULES
	src/lib/events/libevents
	src/lib/heatshrink/heatshrink
	src/modules/mavlink/mavlink
)

zephyr_include_directories(include)

add_subdirectory(${px4_source_dir} ${CMAKE_CURRENT_BINARY_DIR}/px4)

target_link_libraries(app PRIVATE px4)

# compile settings for application code using the PX4 API
get_directory_property(px4_include_dirs DIRECTORY ${px4_source_dir} INCLUDE_DIRECTORIES)
get_directory_property(px4_compile_definitions DIRECTORY ${px4_source_dir} COMPILE_DEFINITIONS)
add_library(px4_interface INTERFACE)
target_include_directories(px4_interface INTERFACE ${px4_include_dirs})
target_compile_definitions(px4_interface INTERFACE ${px4_compile_definitions})
target_compile_options(px4_interface INTERFACE
	$<$<COMPILE_LANGUAGE:C>:-std=gnu11>
	$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>
	-include visibility.h
)
add_dependencies(px4_interface uorb_headers)

zephyr_library_named(px4_pthread)
# px4_pthread_attr_destroy() keeps the stacks that threads run on, see px4_pthread.c
zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_pthread.c)

if(CONFIG_PX4_SHELL)
	zephyr_library_named(px4_shell)
	zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_shell.c)
endif()
