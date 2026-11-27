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
 * @file rtl_route.cpp
 *
 * RTL_TYPE 7, the Return along the mission route. Built only with the mission route cache, so its events stay out
 * of the metadata of boards without it.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "rtl.h"
#include "mission_route_cache.h"
#include "navigator.h"
#include "rtl_mission_safe_point_follow.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

void RTL::selectRoute(ReturnSelection &selection)
{
	const vehicle_status_s &vehicle_status = _vehicle_status_sub.get();

	// Multicopters only. A VTOL reports ROTARY_WING in hover and in both transitions.
	if (vehicle_status.is_vtol || (vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)) {
		selectClosest(_param_rtl_type.get(), selection);
		return;
	}

	// The follower plans its own route. Only if it cannot be created, Return lands here as RTL_TYPE 5 does
	// without a rally point.
	selection.executor = RtlType::RTL_MISSION_SAFE_POINT_FOLLOW;
	selection.destination.lat = _global_pos_sub.get().lat;
	selection.destination.lon = _global_pos_sub.get().lon;
	selection.destination.alt = _global_pos_sub.get().alt;
	selection.destination_type = DestinationType::DESTINATION_TYPE_CURRENT_POSITION;
}

void RTL::updateRoute(bool evaluation_due)
{
	if (_rtl_type != RtlType::RTL_MISSION_SAFE_POINT_FOLLOW) {
		return;
	}

	RtlMissionSafePointFollow &follower = *static_cast<RtlMissionSafePointFollow *>(_rtl_mission_type_handle);
	const mission_s &mission = _mission_sub.get();
	const bool had_route = follower.plan().valid();
	// Idle on the ground, Return stays there, also if the land detector then reports a takeoff. Example: a vehicle
	// lifted by hand.
	const bool on_ground = _navigator->get_land_detected()->landed || follower.isIdle();

	// In the air, an upload replans at once, unless the landing has started, and without a route Return plans at
	// each evaluation. Example: new rally points while data still loads stop the vehicle until a replan finds a route.
	const bool replan = had_route && !on_ground && !follower.isLanding() && !follower.plannedOn(mission);
	const bool retry = !had_route && !on_ground && evaluation_due;

	if (!replan && !retry) {
		return;
	}

	follower.planRoute(mission);

	// Start again on the new route, or hold where the vehicle is. A failed retry keeps the hold.
	if (replan || follower.plan().valid()) {
		follower.run(false);
		follower.run(true);
	}

	if (retry && follower.plan().valid()) {
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Return: mission route found\t");
		events::send(events::ID("rtl_route_found"), events::Log::Info, "Return: mission route found");
	}

	publishRouteStatus();
}

void RTL::publishRouteStatus()
{
	const MissionRouteCache &mission_route_cache = _navigator->get_mission_route_cache();
	const mission_route::RtlRoutePlan &plan = static_cast<RtlMissionSafePointFollow *>(_rtl_mission_type_handle)->plan();

	rtl_status_s rtl_status{};
	rtl_status.safe_points_id = mission_route_cache.safePointsId();
	rtl_status.is_evaluation_pending = !mission_route_cache.safePointsReady()
					   || mission_route_cache.missionLandItemUpdatePending();
	// Without a route there is no Return type: Return holds.
	rtl_status.rtl_type = plan.valid() ? rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT
			      : rtl_status_s::RTL_STATUS_TYPE_NONE;
	rtl_status.safe_point_index = (plan.goal_type == mission_route::GoalType::kSafePoint)
				      ? static_cast<uint8_t>(plan.safe_point_index) : RTL_STATUS_NO_SAFE_POINT;
	rtl_status.timestamp = hrt_absolute_time();
	_rtl_status_pub.publish(rtl_status);
}
