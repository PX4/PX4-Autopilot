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

# Out-of-tree planner CLI for Tools/navigator_mission_planner_visualizer. Build it with:
#   make px4_sitl_test EXTERNAL_MODULES_LOCATION=$PWD/Tools/navigator_mission_planner_visualizer/planner_cli mission_route_planner_cli

if(CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE AND PX4_PLATFORM STREQUAL "posix" AND BUILD_TESTING)
	set(NAVIGATOR_DIR ${PX4_SOURCE_DIR}/src/modules/navigator)

	# The planner sources are compiled here, not linked, so DEBUG_BUILD turns on their
	# PX4_DEBUG scoring traces for this binary only.
	add_executable(mission_route_planner_cli EXCLUDE_FROM_ALL
		mission_route_planner_cli.cpp
		${NAVIGATOR_DIR}/mission_route_internal_types.cpp
		${NAVIGATOR_DIR}/mission_route_projection.cpp
		${NAVIGATOR_DIR}/mission_route_planner.cpp
	)
	target_include_directories(mission_route_planner_cli PRIVATE ${NAVIGATOR_DIR})
	target_compile_definitions(mission_route_planner_cli PRIVATE MODULE_NAME="mission_route_planner_cli" DEBUG_BUILD)

	# Same platform libraries as px4_add_functional_gtest(), without gtest.
	target_link_libraries(mission_route_planner_cli PRIVATE
		navigator_mission_route
		px4_layer
		px4_platform
		uORB
		systemlib
		cdev
		px4_work_queue
		px4_daemon
		work_queue
		parameters
		events
		perf
		tinybson
		uorb_msgs
		pthread
		# Daemon app-map stubs, defined later in the configure run, hence no TARGET check here.
		-Wl,--whole-archive test_stubs -Wl,--no-whole-archive
	)
endif()
