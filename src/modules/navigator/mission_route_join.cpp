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

bool Mission::trySetRouteJoinOnActivation(const bool keep_mission_index)
{
	resetJoinRouteState();

	// A finished mission holds as without the join, so the planned item must still be valid.
	if (keep_mission_index
	    || (_param_mis_route_join.get() == 0)
	    || !isMissionValid()
	    || !_is_current_planned_mission_item_valid
	    || _land_detected_sub.get().landed
	    || (_mission.count < 2)
	    || (_mission.current_seq < 0)
	    || cameraWasTriggeringAtPause()) {
		return false;
	}

	// Mission continues as without MIS_ROUTE_JOIN; tell the operator the join was skipped.
	const auto fall_back = [this](const char *reason) {
		PX4_WARN("Mission route rejoin unavailable: %s", reason);
		mavlink_log_warning(_navigator->get_mavlink_log_pub(), "Route rejoin unavailable, flying to current item\t");
		events::send(events::ID("mission_route_rejoin_unavailable"), events::Log::Warning,
			     "Route rejoin unavailable, flying to current item");
		return false;
	};

	const MissionRouteCache &mission_route_cache = _navigator->get_mission_route_cache();

	if (!mission_route_cache.missionItemsReady(_mission)) {
		return fall_back("route cache not ready");
	}

	const vehicle_global_position_s &global_position = _global_pos_sub.get();
	const vehicle_status_s &vehicle_status = _vehicle_status_sub.get();
	const vehicle_local_position_s *local_position = _navigator->get_local_position();

	mission_route::MissionResumeRequest request{};
	request.vehicle_position = {global_position.lat, global_position.lon, global_position.alt};
	request.mission_index = _mission.current_seq;
	request.active_jump_anchor = _active_jump_anchor;
	request.projection_search_distance_m =
		(vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
		? _param_mis_mc_seg_dist.get() : _param_mis_fw_seg_dist.get();
	request.acceptance_radius_m = _navigator->get_acceptance_radius();
	request.home_altitude_amsl = _navigator->get_home_position()->valid_alt
				     ? _navigator->get_home_position()->alt : NAN;
	request.is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	request.in_transition_to_fw = vehicle_status.in_transition_to_fw;
	request.is_vtol = vehicle_status.is_vtol;

	if (local_position->v_xy_valid) {
		request.velocity_north_m_s = local_position->vx;
		request.velocity_east_m_s = local_position->vy;
	}

	const mission_route::GeofencePathChecker path_checker{_navigator->get_geofence(), request.vehicle_position};
	MissionRoutePlanner planner{mission_route_cache, &path_checker};
	mission_route::MissionResumePlan join_plan{};
	const mission_route::FailureReason failure_reason = planner.planMissionResumeJoin(request, join_plan);

	if (failure_reason != mission_route::FailureReason::kNone) {
		return fall_back(mission_route::failureReasonString(failure_reason));
	}

	_active_jump_anchor = join_plan.active_jump_anchor;
	setMissionIndex(join_plan.first_mission_item_index);
	// As for a user-selected index, MissionBase::on_activation() must not rewind to the paused leg.
	_inactivation_index = -1;

	// The selected landing is within the acceptance radius: land directly, as route Return does,
	// instead of restoring the approach's VTOL state.
	if (join_plan.use_current_altitude) {
		return false;
	}

	setupJoinRoute(join_plan.join_position, join_plan.use_current_altitude, join_plan.vtol_transition_action);

	PX4_DEBUG("Mission route join: target=%d vtol=%u", static_cast<int>(join_plan.first_mission_item_index),
		  static_cast<unsigned>(join_plan.vtol_transition_action));

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Rejoining mission route\t");
	events::send(events::ID("mission_route_rejoin"), events::Log::Info, "Rejoining mission route");
	return true;
}
