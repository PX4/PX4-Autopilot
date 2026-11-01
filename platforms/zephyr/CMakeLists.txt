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

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

set(builtin_apps_string)
set(builtin_apps_decl_string)
foreach(module ${module_libraries})
	get_target_property(MAIN ${module} MAIN)
	get_target_property(STACK_MAIN ${module} STACK_MAIN)
	get_target_property(PRIORITY ${module} PRIORITY)

	if(MAIN)
		string(APPEND builtin_apps_string "\t{\"${MAIN}\", ${MAIN}_main, ${STACK_MAIN}, ${PRIORITY}},\n")
		string(APPEND builtin_apps_decl_string "int ${MAIN}_main(int argc, char *argv[]);\n")
	endif()
endforeach()

set(startup_script ${PX4_BOARD_DIR}/init/rc.px4)
if(EXISTS ${PX4_BOARD_DIR}/init/rc.${PX4_BOARD_LABEL}.px4)
	set(startup_script ${PX4_BOARD_DIR}/init/rc.${PX4_BOARD_LABEL}.px4)
endif()
set(startup_script_string)
if(EXISTS ${startup_script})
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${startup_script})
	file(STRINGS ${startup_script} startup_script_lines)
	foreach(line ${startup_script_lines})
		string(STRIP "${line}" line)
		if(line AND NOT line MATCHES "^#")
			string(REPLACE "\\" "\\\\" line "${line}")
			string(REPLACE "\"" "\\\"" line "${line}")
			string(APPEND startup_script_string "\t\"${line}\",\n")
		endif()
	endforeach()
endif()

configure_file(px4_zephyr_apps.c.in ${CMAKE_CURRENT_BINARY_DIR}/px4_zephyr_apps.c)

add_library(px4 STATIC
	${CMAKE_CURRENT_BINARY_DIR}/px4_zephyr_apps.c
	src/px4/common/px4_zephyr.cpp
)
target_compile_definitions(px4 PRIVATE MODULE_NAME="px4")
target_link_libraries(px4 PUBLIC
	${module_libraries}
	parameters
	px4_layer
	px4_platform
	uORB
	work_queue
	px4_work_queue
)
