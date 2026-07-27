############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

add_subdirectory(MissionFeasibility)
add_subdirectory(RTLPlanner)

if(CONFIG_NAVIGATOR_ADSB)
	add_subdirectory(DetectAndAvoid)
endif()

if(BUILD_TESTING)
	add_subdirectory(test)
endif()

set(NAVIGATOR_SOURCES
	navigator_main.cpp
	navigator_mode.cpp
	mission_base.cpp
	mission_block.cpp
	mission.cpp
	course.cpp
	loiter.cpp
	rtl.cpp
	rtl_direct.cpp
	rtl_direct_mission_land.cpp
	rtl_mission_fast.cpp
	rtl_mission_fast_reverse.cpp
	takeoff.cpp
	land.cpp
	precland.cpp
	mission_feasibility_checker.cpp
	geofence.cpp)

set(NAVIGATOR_DEPENDS
	dataman_client
	geo
	motion_planning
	mission_feasibility_checker
	rtl_time_estimator)

if(CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE)
	list(APPEND NAVIGATOR_SOURCES rtl_geofence_avoidance_helper.cpp)
	# geofence_utils (polygon offsetting / visibility graph) is only used by the planner
	list(APPEND NAVIGATOR_DEPENDS geofence_utils geofence_avoidance_planner)
endif()

if(CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF)
	set(NAVIGATOR_SOURCES
		${NAVIGATOR_SOURCES}
		vtol_takeoff.cpp)
endif()

if(CONFIG_NAVIGATOR_ADSB)
	list(APPEND NAVIGATOR_DEPENDS adsb detect_and_avoid)
endif()

px4_add_module(
	MODULE modules__navigator
	MAIN navigator
	SRCS ${NAVIGATOR_SOURCES}
	MODULE_CONFIG
		geofence_params.yaml
		mission_params.yaml
		navigator_params.yaml
		precland_params.yaml
		rtl_params.yaml
		vtol_takeoff_params.yaml
	DEPENDS
		${NAVIGATOR_DEPENDS}
	)
