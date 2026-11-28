/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mission_route_join.cpp
 *
 * Mission's route join on activation. Built only with the mission cache, so its events stay out of the metadata
 * of boards without it.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission.h"
#include "mission_route_geofence.h"
#include "mission_route_planner.h"
#include "navigator.h"

#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>

bool Mission::routeJoinEnabled() const
{
	// Multicopters only. A VTOL reports ROTARY_WING in hover and in both transitions.
	const vehicle_status_s &status = _vehicle_status_sub.get();
	return (_param_mis_route_join.get() != 0)
	       && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol;
}

void Mission::trySetRouteJoinOnActivation()
{
	// A finished mission holds as without the join, so the planned item must still be valid.
	if (!isMissionValid()
	    || !_is_current_planned_mission_item_valid
	    || _land_detected_sub.get().landed
	    || (_mission.count < 2)) {
		return;
	}

	// Mission continues as without MIS_ROUTE_JOIN; tell the operator the join was skipped.
	const auto report_unavailable = [this]() {
		mavlink_log_warning(_navigator->get_mavlink_log_pub(), "Route rejoin unavailable\t");
		events::send(events::ID("mission_route_rejoin_unavailable"), events::Log::Warning, "Route rejoin unavailable");
	};

	const MissionRouteCache &mission_route_cache = _navigator->get_mission_route_cache();

	if (!mission_route_cache.missionItemsReady(_mission)) {
		PX4_WARN("Mission route rejoin unavailable: route cache not ready");
		report_unavailable();
		return;
	}

	const vehicle_global_position_s &global_position = _global_pos_sub.get();

	mission_route::MissionResumeRequest request{};
	request.vehicle_position = {global_position.lat, global_position.lon, global_position.alt};
	request.mission_index = _mission.current_seq;
	request.active_jump_anchor = _active_jump_anchor;
	request.projection_search_distance_m = _param_mis_mc_seg_dist.get();
	request.acceptance_radius_m = _navigator->get_acceptance_radius();
	request.home_altitude_amsl = _navigator->get_home_position()->valid_alt
				     ? _navigator->get_home_position()->alt : NAN;

	const mission_route::GeofencePathChecker path_checker{_navigator->get_geofence()};
	MissionRoutePlanner planner{mission_route_cache, &path_checker};
	mission_route::MissionResumePlan join_plan{};
	const mission_route::FailureReason failure_reason = planner.planMissionResumeJoin(request, join_plan);

	if (failure_reason != mission_route::FailureReason::kNone) {
		// A FailureReason number: the names would cost flash for one console line.
		PX4_WARN("Mission route rejoin unavailable: planner failure %d", static_cast<int>(failure_reason));
		report_unavailable();
		return;
	}

	_active_jump_anchor = join_plan.active_jump_anchor;
	setMissionIndex(join_plan.first_mission_item_index);
	// The join replaces the resume at the paused item, also the survey rewind of MissionBase::on_activation().
	_inactivation_index = -1;

	// The selected landing is within the acceptance radius: land directly instead of flying a join waypoint.
	if (join_plan.use_current_altitude) {
		return;
	}

	// Pending until setActiveMissionItems() flies it as a work item.
	_route_join_position = join_plan.join_position;

	PX4_DEBUG("Mission route join: target=%d", static_cast<int>(join_plan.first_mission_item_index));

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Rejoining mission route\t");
	events::send(events::ID("mission_route_rejoin"), events::Log::Info, "Rejoining mission route");
}
