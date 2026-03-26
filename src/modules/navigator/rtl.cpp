/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file rtl.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 */

#include "rtl.h"
#include "navigator.h"
#include "mission_block.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>

using namespace time_literals;
using namespace math;
using matrix::wrap_pi;

static constexpr float MIN_DIST_THRESHOLD = 2.f;

// Named constants for RTL_TYPE parameter values (must match @value tags in rtl_params.c).
static constexpr int RTL_TYPE_MISSION_FAST = 2;
static constexpr int RTL_TYPE_DIRECT_WITH_MISSION_LAND = 3;
static constexpr int RTL_TYPE_MISSION_FAST_OR_REVERSE = 4;
static constexpr int RTL_TYPE_SAFE_POINT_DIRECT = 5;
static constexpr int RTL_TYPE_ROUTE_SAFE_POINT = 6;


RTL::RTL(Navigator *navigator) :
	NavigatorMode(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL),
	ModuleParams(navigator),
	_rtl_direct(navigator)
{
	_rtl_direct.initialize();
}

void RTL::updateDatamanCache()
{
	const mission_s &mission = _mission_sub.get();
	MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();

	if (mission.mission_id != _route_plan_mission_id || mission.safe_points_id != _route_plan_safe_points_id) {
		_route_plan_mission_id = mission.mission_id;
		_route_plan_safe_points_id = mission.safe_points_id;
		resetRouteSafePointCache();
	}

	if (mission_route_cache == nullptr) {
		return;
	}

	mission_route_cache->update(mission);

	if (_param_rtl_type.get() == RTL_TYPE_ROUTE_SAFE_POINT
	    && mission_route_cache->missionExceedsCacheLimit(mission)) {
		static uint32_t last_warned_mission_id = 0;

		if (mission.mission_id != last_warned_mission_id) {
			mavlink_log_warning(_navigator->get_mavlink_log_pub(),
					    "Mission exceeds %d items. Route RTL unavailable, falling back to direct RTL.\t",
					    static_cast<int>(MissionRouteCache::MAX_ROUTE_MISSION_CACHE_SIZE));
			events::send(events::ID("rtl_route_mission_too_large"), events::Log::Warning,
				     "Mission exceeds route RTL cache limit, falling back to direct RTL");
			last_warned_mission_id = mission.mission_id;
		}
	}
}

void RTL::resetRouteSafePointCache()
{
	_route_safe_point_plan = {};
	_last_route_safe_point_loop_segment = {};
	_should_go_straight_to_safe_point = false;
}

void RTL::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_mission_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();

	updateDatamanCache();

	parameters_update();

	if (_rtl_mission_type_handle) {
		_rtl_mission_type_handle->run(false);
	}

	_rtl_direct.run(false);

	// Limit inactive calculation to 0.5Hz
	hrt_abstime now{hrt_absolute_time()};

	if ((now - _destination_check_time) > 2_s) {
		_destination_check_time = now;
		setRtlTypeAndDestination();
		publishRemainingTimeEstimate();
	}

}

/**
 * @brief Preserve the currently branched-off state before the nested mode is deactivated.
 */
void RTL::on_inactivation()
{
	if (_rtl_mission_type_handle) {
		_rtl_mission_type_handle->run(false);
		_should_go_straight_to_safe_point = _rtl_mission_type_handle->shouldGoStraightToGoal();
		_last_route_safe_point_loop_segment = _rtl_mission_type_handle->lastFlownLoopSegment();

	} else {
		_should_go_straight_to_safe_point = false;
		_last_route_safe_point_loop_segment = {};
	}

	_rtl_direct.run(false);
}

void RTL::publishRemainingTimeEstimate()
{
	const bool global_position_recently_updated = _global_pos_sub.get().timestamp > 0
			&& hrt_elapsed_time(&_global_pos_sub.get().timestamp) < 10_s;

	rtl_time_estimate_s estimated_time{};
	estimated_time.valid = false;

	if (_navigator->home_global_position_valid() && global_position_recently_updated) {
		switch (_rtl_type) {
		case RtlType::RTL_DIRECT:
			estimated_time = _rtl_direct.calc_rtl_time_estimate();
			break;

		case RtlType::RTL_DIRECT_MISSION_LAND:
		case RtlType::RTL_MISSION_FAST:
		case RtlType::RTL_MISSION_FAST_REVERSE:
		case RtlType::RTL_MISSION_SAFE_POINT_FOLLOW:
			if (_rtl_mission_type_handle) {
				estimated_time = _rtl_mission_type_handle->calc_rtl_time_estimate();
			}

			break;

		default:
			break;
		}
	}

	_rtl_time_estimate_pub.publish(estimated_time);
}

void RTL::on_activation()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_mission_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();

	setRtlTypeAndDestination();

	switch (_rtl_type) {
	case RtlType::RTL_DIRECT_MISSION_LAND:	// Fall through
	case RtlType::RTL_MISSION_FAST: // Fall through
	case RtlType::RTL_MISSION_FAST_REVERSE: // Fall through
	case RtlType::RTL_MISSION_SAFE_POINT_FOLLOW:
		if (_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW && _route_safe_point_plan.valid()) {
			if (_route_safe_point_plan.selection.safe_point_found) {
				if (_route_safe_point_plan.selection.direct_to_safe_point) {
					PX4_INFO("RTL type 6 start: within safe point %d acc rad",
						 static_cast<int>(_route_safe_point_plan.selection.safe_point_index));

				} else if (_should_go_straight_to_safe_point) {
					PX4_INFO("RTL type 6 start: branched-off, straight to safe point %d",
						 static_cast<int>(_route_safe_point_plan.selection.safe_point_index));

				} else {
					PX4_INFO("RTL type 6 start: safe_point=%d target=%d branch_off=%d rev=%u",
						 static_cast<int>(_route_safe_point_plan.selection.safe_point_index),
						 static_cast<int>(_route_safe_point_plan.selection.path.first_item_index),
						 static_cast<int>(_route_safe_point_plan.selection.branchOffIndex()),
						 static_cast<unsigned>(_route_safe_point_plan.selection.path.direction_reversed));
				}

			} else {
				PX4_INFO("RTL type 6 fallback to %s target=%d rev=%u",
					 MissionRoutePlanner::goalTypeString(_route_safe_point_plan.selection.goal_type),
					 static_cast<int>(_route_safe_point_plan.selection.path.first_item_index),
					 static_cast<unsigned>(_route_safe_point_plan.selection.path.direction_reversed));
			}
		}

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->setReturnAltMin(_enforce_rtl_alt);
			_rtl_mission_type_handle->run(true);
		}

		_rtl_direct.run(false);

		break;

	case RtlType::RTL_DIRECT:
		_rtl_direct.setReturnAltMin(_enforce_rtl_alt);
		_rtl_direct.run(true);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->run(false);
		}

		break;

	default:
		break;
	}

	_navigator->activate_set_gimbal_neutral_timer(hrt_absolute_time());
}

void RTL::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_mission_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();

	updateDatamanCache();

	switch (_rtl_type) {
	case RtlType::RTL_MISSION_FAST: // Fall through
	case RtlType::RTL_MISSION_FAST_REVERSE: // Fall through
	case RtlType::RTL_DIRECT_MISSION_LAND: // Fall through
	case RtlType::RTL_MISSION_SAFE_POINT_FOLLOW:
		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->run(true);
		}

		_rtl_direct.run(false);
		break;

	case RtlType::RTL_DIRECT:
		_rtl_direct.run(true);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->run(false);
		}

		break;

	default:
		break;
	}

	// Keep publishing remaining time estimates every 2 seconds
	hrt_abstime now{hrt_absolute_time()};

	if ((now - _destination_check_time) > 2_s) {
		_destination_check_time = now;
		publishRemainingTimeEstimate();
	}
}

bool RTL::isLanding()
{
	bool is_landing{false};

	switch (_rtl_type) {
	case RtlType::RTL_MISSION_FAST:
	case RtlType::RTL_MISSION_FAST_REVERSE:
	case RtlType::RTL_DIRECT_MISSION_LAND:
	case RtlType::RTL_MISSION_SAFE_POINT_FOLLOW:
		if (_rtl_mission_type_handle) {
			is_landing = _rtl_mission_type_handle->isLanding();
		}

		break;

	case RtlType::RTL_DIRECT:
		is_landing = _rtl_direct.isLanding();
		break;

	default:
		break;
	}

	return is_landing;
}

/**
 * @brief Recompute the active RTL variant and destination from the current mission, safe points, and cached plan.
 */
void RTL::setRtlTypeAndDestination()
{
	uint8_t safe_point_index = UINT8_MAX;
	RtlType new_rtl_type{RtlType::RTL_DIRECT};
	const MissionRoutePlanner::Plan cached_route_safe_point_plan = _route_safe_point_plan;
	_route_safe_point_plan = {};
	const bool cached_should_go_straight_to_safe_point = _should_go_straight_to_safe_point;
	_should_go_straight_to_safe_point = false;

	// init destination with Home (used also with Type 2 and 4 as backup)
	DestinationType destination_type = DestinationType::DESTINATION_TYPE_HOME;
	PositionYawSetpoint destination;
	destination.lat = _home_pos_sub.get().lat;
	destination.lon = _home_pos_sub.get().lon;
	destination.alt = _home_pos_sub.get().alt;
	destination.yaw = _home_pos_sub.get().yaw;

	loiter_point_s landing_loiter;
	landing_loiter.lat = destination.lat;
	landing_loiter.lon = destination.lon;
	landing_loiter.height_m = NAN;
	loiter_point_s goal_landing_approach{};

	if (_param_rtl_type.get() == RTL_TYPE_MISSION_FAST) {
		if (hasMissionLandStart()) {
			new_rtl_type = RtlType::RTL_MISSION_FAST;

		} else if (_navigator->get_mission_result()->valid) {
			new_rtl_type = RtlType::RTL_MISSION_FAST_REVERSE;

		} else {
			// no valid mission, go direct to home
			new_rtl_type = RtlType::RTL_DIRECT;
		}

	} else if (_param_rtl_type.get() == RTL_TYPE_MISSION_FAST_OR_REVERSE) {
		if (hasMissionLandStart() && reverseIsFurther()) {
			new_rtl_type = RtlType::RTL_MISSION_FAST;

		} else if (_navigator->get_mission_result()->valid) {
			new_rtl_type = RtlType::RTL_MISSION_FAST_REVERSE;

		} else {
			// no valid mission, go direct to home
			new_rtl_type = RtlType::RTL_DIRECT;
		}

	} else if (_param_rtl_type.get() == RTL_TYPE_ROUTE_SAFE_POINT) {
		const bool current_route_direction_reversed = cached_route_safe_point_plan.valid()
				? cached_route_safe_point_plan.selection.path.direction_reversed : false;
		MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();

		const bool mission_route_cache_ready =  mission_route_cache != nullptr
							&& mission_route_cache->safePointsReady()
							&& mission_route_cache->isReady(_mission_sub.get());

		if (_navigator->get_mission_result()->valid && mission_route_cache_ready
		    && evaluateRouteSafePointPlan(*mission_route_cache, cached_route_safe_point_plan,
						  cached_should_go_straight_to_safe_point,
						  _route_safe_point_plan, _should_go_straight_to_safe_point)) {
			applyRouteSafePointPlan(_route_safe_point_plan, current_route_direction_reversed,
						new_rtl_type, destination_type, destination, safe_point_index);

		} else {
			applyRouteSafePointFallback(new_rtl_type, destination_type, destination, safe_point_index);
		}

	} else {
		// RTL_TYPE 0, 1, 3, 5: use closest safe point / home / mission landing via direct path.
		findRtlDestination(destination_type, destination, safe_point_index);

		if (destination_type == DestinationType::DESTINATION_TYPE_MISSION_LAND) {
			new_rtl_type = RtlType::RTL_DIRECT_MISSION_LAND;

		} else {

			new_rtl_type = RtlType::RTL_DIRECT;
		}
	}

	if (new_rtl_type == RtlType::RTL_DIRECT) {
		landing_loiter = selectLandingApproach(destination);

		if (!landing_loiter.isValid()) {
			landing_loiter.lat = destination.lat;
			landing_loiter.lon = destination.lon;
			landing_loiter.height_m = NAN;
		}
	}

	if (new_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW && _route_safe_point_plan.selection.safe_point_found) {
		goal_landing_approach = selectLandingApproach(destination);
	}

	const float rtl_alt = computeReturnAltitude(destination);
	_rtl_direct.setRtlAlt(rtl_alt);
	_rtl_direct.setRtlPosition(destination, landing_loiter);

	const bool new_type_is_mission_based = (new_rtl_type == RtlType::RTL_MISSION_FAST)
					       || (new_rtl_type == RtlType::RTL_MISSION_FAST_REVERSE)
					       || (new_rtl_type == RtlType::RTL_DIRECT_MISSION_LAND)
					       || (new_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	const bool should_init_mission_type = new_type_is_mission_based
					      && ((_rtl_type == new_rtl_type) ? (_rtl_mission_type_handle == nullptr) : true);
	const bool mission_type_initialized = should_init_mission_type ? initRtlMissionType(new_rtl_type, rtl_alt) : true;

	if (mission_type_initialized == false) {
		PX4_ERR("RTL mission executor init failed for type %u, falling back to direct RTL",
			static_cast<unsigned>(new_rtl_type));
		new_rtl_type = RtlType::RTL_DIRECT;
		landing_loiter = selectLandingApproach(destination);

		if (!landing_loiter.isValid()) {
			landing_loiter.lat = destination.lat;
			landing_loiter.lon = destination.lon;
			landing_loiter.height_m = NAN;
		}

		_rtl_direct.setRtlPosition(destination, landing_loiter);
	}

	// setRoutePlan is called unconditionally (even when the type has not changed) so that
	// a replanned route is pushed to the executor without recreating the mission executor instance.
	if (new_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW && _rtl_mission_type_handle) {
		_rtl_mission_type_handle->setRtlAlt(rtl_alt);
		_rtl_mission_type_handle->setRoutePlan(_route_safe_point_plan);
		_rtl_mission_type_handle->setShouldGoStraightToGoal(_should_go_straight_to_safe_point);
		_rtl_mission_type_handle->setGoalLandApproach(goal_landing_approach);
	}

	_rtl_type = new_rtl_type;

	// Publish rtl status
	rtl_status_s rtl_status{};
	MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();
	rtl_status.safe_points_id = mission_route_cache != nullptr ? mission_route_cache->safePointsId() : 0;
	rtl_status.is_evaluation_pending = mission_route_cache != nullptr && mission_route_cache->safePointUpdatePending();
	rtl_status.has_vtol_approach = _home_has_land_approach || _any_safe_point_has_land_approach;
	rtl_status.rtl_type = static_cast<uint8_t>(_rtl_type);
	rtl_status.safe_point_index = safe_point_index;
	rtl_status.timestamp = hrt_absolute_time();
	_rtl_status_pub.publish(rtl_status);
}

MissionRoutePlanner::Config RTL::buildRouteSafePointConfig(bool is_flying_reverse) const
{
	MissionRoutePlanner::Config config{};
	const auto &vehicle_status = _vehicle_status_sub.get();
	const auto *local_position = _navigator->get_local_position();

	config.parameters.vehicle_projection_search_dist =
		vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		? _param_mis_mc_seg_dist.get() : _param_mis_fw_seg_dist.get();
	config.parameters.safe_point_projection_search_dist = _param_rtl_rp_seg_dist.get();
	config.parameters.acceptance_radius = _navigator->get_acceptance_radius();
	config.parameters.direct_acceptance_radius = _navigator->get_default_acceptance_radius();
	config.parameters.home_altitude_amsl = _home_pos_sub.get().alt;
	config.parameters.u_turn_penalty_m = _param_rtl_fw_uturn_pen.get();

	config.state.is_multicopter = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				      && !vehicle_status.in_transition_mode;
	config.state.is_flying_reverse = is_flying_reverse;
	config.state.velocity_valid = (local_position != nullptr)
				      && PX4_ISFINITE(local_position->vx)
				      && PX4_ISFINITE(local_position->vy);
	config.state.velocity_ne(0) = (local_position != nullptr) ? local_position->vx : NAN;
	config.state.velocity_ne(1) = (local_position != nullptr) ? local_position->vy : NAN;
	config.state.is_fixed_wing = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	config.state.in_transition_to_fw = vehicle_status.in_transition_to_fw;
	config.state.require_vtol_approach = vehicle_status.is_vtol
					     && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
					     && (_param_rtl_appr_force.get() == 1);

	config.execution.last_flown_loop_segment = _last_route_safe_point_loop_segment;
	return config;
}

bool RTL::reuseCachedRouteSafePointPlan(const MissionRoutePlanner &planner,
					const MissionRouteCache &mission_route_cache,
					const MissionRoutePlanner::Position &vehicle_position,
					float acceptance_radius,
					const MissionRoutePlanner::Plan &cached_plan,
					bool cached_should_go_straight_to_safe_point,
					MissionRoutePlanner::Plan &new_plan) const
{
	const bool reuse_cached_plan = cached_should_go_straight_to_safe_point
				       && cached_plan.valid()
				       && cached_plan.selection.safe_point_found
				       && planner.closeToBranchOffSegment(vehicle_position, cached_plan.selection,
						       acceptance_radius);

	if (!reuse_cached_plan) {
		return false;
	}

	new_plan = cached_plan;

	PX4_DEBUG("RTL type 6 reusing cached plan");
	return true;
}

bool RTL::evaluateRouteSafePointPlan(const MissionRouteCache &mission_route_cache,
				     const MissionRoutePlanner::Plan &cached_plan,
				     bool cached_should_go_straight_to_safe_point,
				     MissionRoutePlanner::Plan &new_plan,
				     bool &should_go_straight_to_safe_point)
{
	new_plan = {};
	should_go_straight_to_safe_point = false;

	const bool is_flying_reverse = cached_plan.valid()
				       ? cached_plan.selection.path.direction_reversed : false;
	const float home_altitude_amsl = _home_pos_sub.get().alt;
	const MissionRoutePlanner::Config config = buildRouteSafePointConfig(is_flying_reverse);
	_any_safe_point_has_land_approach = mission_route_cache.anySafePointHasVtolLandApproach(home_altitude_amsl);
	const MissionRoutePlanner::Position vehicle_position{_global_pos_sub.get().lat, _global_pos_sub.get().lon, _global_pos_sub.get().alt};
	MissionRoutePlanner planner(mission_route_cache);

	if (reuseCachedRouteSafePointPlan(planner, mission_route_cache, vehicle_position,
					  config.parameters.acceptance_radius,
					  cached_plan, cached_should_go_straight_to_safe_point,
					  new_plan)) {
		should_go_straight_to_safe_point = true;
		return true;
	}

	MissionRoutePlanner::FailureReason failure_reason{MissionRoutePlanner::FailureReason::Unknown};

	if (!planner.planRouteToGoal(vehicle_position, _mission_sub.get().current_seq,
				     config, new_plan, failure_reason)) {
		PX4_ERR("RTL type 6 plan failed: %s", MissionRoutePlanner::failureReasonString(failure_reason));
		return false;
	}

	// This is necessary for takeoff + Return && no safe point found --> we are landing at the takeoff
	// if a rally point was found, let _should_go_straight_to_safe_point handle it to ensure that a rally point far from the
	// takeoff point but projected onto a takeoff point does not result in an immediate land.
	// MissionBase::setupJoinRoute() consumes this hint and applies the final skip-altitude correction.
	if (new_plan.selection.path.in_first_item_acc_rad
	    && new_plan.selection.path.first_item_cmd == NAV_CMD_TAKEOFF
	    && new_plan.selection.goal_type != MissionRoutePlanner::GoalType::SafePoint) {
		new_plan.join_context.skip_altitude_requirement = true;
	}

	should_go_straight_to_safe_point = new_plan.selection.direct_to_safe_point;

	PX4_DEBUG("RTL type 6 plan: goal=%s target=%d rev=%u straight=%u",
		  MissionRoutePlanner::goalTypeString(new_plan.selection.goal_type),
		  static_cast<int>(new_plan.selection.path.first_item_index),
		  static_cast<unsigned>(new_plan.selection.path.direction_reversed),
		  static_cast<unsigned>(should_go_straight_to_safe_point));

	return true;
}

RTL::DestinationType RTL::routePlanDestinationType(MissionRoutePlanner::GoalType goal_type)
{
	switch (goal_type) {
	case MissionRoutePlanner::GoalType::SafePoint:
		return DestinationType::DESTINATION_TYPE_SAFE_POINT;

	case MissionRoutePlanner::GoalType::MissionLand:
		return DestinationType::DESTINATION_TYPE_MISSION_LAND;

	case MissionRoutePlanner::GoalType::MissionTakeoff:
		return DestinationType::DESTINATION_TYPE_MISSION_TAKEOFF;

	case MissionRoutePlanner::GoalType::None:
	default:
		PX4_ERR("Invalid route plan goal type: %u", static_cast<unsigned>(goal_type));
		return DestinationType::DESTINATION_TYPE_MISSION_LAND;
	}
}

void RTL::applyRouteSafePointPlan(const MissionRoutePlanner::Plan &plan,
				  bool current_route_direction_reversed,
				  RtlType &new_rtl_type,
				  DestinationType &destination_type,
				  PositionYawSetpoint &destination,
				  uint8_t &safe_point_index)
{
	const bool direction_will_change = current_route_direction_reversed
					   != plan.selection.path.direction_reversed;

	if (isActive() && direction_will_change && _vehicle_status_sub.get().in_transition_to_fw) {
		vehicle_command_s cmd{};
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		cmd.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		cmd.param2 = 0.f;
		_navigator->publish_vehicle_command(cmd);
		PX4_INFO("RTL direction changed during FT, requesting BT");
	}

	new_rtl_type = RtlType::RTL_MISSION_SAFE_POINT_FOLLOW;
	_last_route_safe_point_loop_segment = plan.projection_context.seg_candidate.segment.validLoop()
					      ? plan.projection_context.seg_candidate.segment
					      : MissionRoutePlanner::Segment{};
	destination_type = routePlanDestinationType(plan.selection.goal_type);
	destination.lat = plan.selection.goal_position.lat;
	destination.lon = plan.selection.goal_position.lon;
	destination.alt = plan.selection.goal_position.alt;
	destination.yaw = NAN;
	safe_point_index = plan.selection.safe_point_found
			   ? static_cast<uint8_t>(constrain(plan.selection.safe_point_index, 0, static_cast<int32_t>(UINT8_MAX)))
			   : UINT8_MAX;
}

void RTL::applyRouteSafePointFallback(RtlType &new_rtl_type,
				      DestinationType &destination_type,
				      PositionYawSetpoint &destination,
				      uint8_t &safe_point_index)
{
	_should_go_straight_to_safe_point = false;
	findRtlDestinationForType(RTL_TYPE_DIRECT_WITH_MISSION_LAND, destination_type, destination, safe_point_index);

	if (destination_type == DestinationType::DESTINATION_TYPE_MISSION_LAND) {
		new_rtl_type = RtlType::RTL_DIRECT_MISSION_LAND;

	} else {
		new_rtl_type = RtlType::RTL_DIRECT;
	}
}

PositionYawSetpoint RTL::findClosestSafePoint(float min_dist, uint8_t &safe_point_index)
{
	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();

	PositionYawSetpoint safe_point{(double)NAN, (double)NAN, NAN, NAN};
	_any_safe_point_has_land_approach = false;

	if (mission_route_cache == nullptr || !mission_route_cache->safePointsReady()) {
		return safe_point;
	}

	for (int current_seq = 0; current_seq < mission_route_cache->safePointCount(); ++current_seq) {
		mission_item_s mission_safe_point;

		if (!mission_route_cache->loadSafePointItem(current_seq, mission_safe_point)) {
			PX4_ERR("dm_read failed");
			continue;
		}

			// Only look at rally points
			if (mission_safe_point.nav_cmd != NAV_CMD_RALLY_POINT) {
				continue;
			}

			// Ignore safepoints which are too close to the homepoint (only if home is an option to return to)
			const bool far_from_home = get_distance_to_next_waypoint(_home_pos_sub.get().lat, _home_pos_sub.get().lon,
						   mission_safe_point.lat, mission_safe_point.lon) > MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES;

			if (far_from_home || (_param_rtl_type.get() == RTL_TYPE_SAFE_POINT_DIRECT)) {
				const float dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, mission_safe_point.lat, mission_safe_point.lon)};

			PositionYawSetpoint safepoint_position;
			setSafepointAsDestination(safepoint_position, mission_safe_point);

			const bool current_safe_point_has_approaches{
				mission_route_cache->hasVtolLandApproach(current_seq, _home_pos_sub.get().alt)
			};

			_any_safe_point_has_land_approach |= current_safe_point_has_approaches;

			if (((dist + MIN_DIST_THRESHOLD) < min_dist)
			    && (!vtol_in_fw_mode || (_param_rtl_appr_force.get() == 0) || current_safe_point_has_approaches)) {
				min_dist = dist;
				safe_point = safepoint_position;
				safe_point_index = current_seq;
			}
		}
	}

	return safe_point;
}

void RTL::findRtlDestinationForType(int rtl_type, DestinationType &destination_type, PositionYawSetpoint &destination,
				    uint8_t &safe_point_index)
{
	const bool vtol_in_rw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	float min_dist = FLT_MAX;

	if (_param_rtl_type.get() != RTL_TYPE_SAFE_POINT_DIRECT) {
		_home_has_land_approach = hasVtolLandApproach(destination);

		const bool prioritize_safe_points_over_home = ((rtl_type == 1) && !vtol_in_rw_mode);
		const bool required_approach_missing_for_home = (vtol_in_fw_mode && (_param_rtl_appr_force.get() == 1) && !_home_has_land_approach);

		// Set minimum distance to maximum value when RTL_TYPE is set to 1 and we are not in RW mode or we force approach landing for vtol in fw and it is not defined for home.
		const bool deprioritize_home = prioritize_safe_points_over_home || required_approach_missing_for_home;

		if (!deprioritize_home) {
			// distance to home position
			min_dist = get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, destination.lat, destination.lon);
		}

		// Mission landing
		if (((rtl_type == 1) || (rtl_type == RTL_TYPE_DIRECT_WITH_MISSION_LAND)
		     || (fabsf(FLT_MAX - min_dist) < FLT_EPSILON)) && hasMissionLandStart()) {
			mission_item_s land_mission_item{};
			int32_t land_index = -1;
			const bool success = mission_route_cache != nullptr && mission_route_cache->getMissionLandItem(land_index, land_mission_item);

			if (!success) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission land item could not be read.\t");
				events::send(events::ID("rtl_failed_to_read_land_item"), events::Log::Error,
					     "Mission land item could not be read");

			} else {
				const float dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, land_mission_item.lat, land_mission_item.lon)};

				if ((dist + MIN_DIST_THRESHOLD) < min_dist) {
					if (rtl_type != 0) {
						min_dist = dist;

					} else {
						// Mission landing is not allowed, but home has no approaches. Still use mission landing.
						min_dist = FLT_MAX;
					}

					setLandPosAsDestination(destination, land_mission_item);
					destination_type = DestinationType::DESTINATION_TYPE_MISSION_LAND;
				}
			}
		}
	}

	// Safe/rally points
	PositionYawSetpoint safe_point = findClosestSafePoint(min_dist, safe_point_index);

	if (safe_point_index != UINT8_MAX) {
		destination = safe_point;
		destination_type = DestinationType::DESTINATION_TYPE_SAFE_POINT;

	} else if (_param_rtl_type.get() == RTL_TYPE_SAFE_POINT_DIRECT) {
		// for RTL_TYPE=5: if no rally point is found fallback to current position
		destination.alt = _global_pos_sub.get().alt;
		destination.lat = _global_pos_sub.get().lat;
		destination.lon = _global_pos_sub.get().lon;
		destination_type = DestinationType::DESTINATION_TYPE_SAFE_POINT;
	}
}

void RTL::findRtlDestination(DestinationType &destination_type, PositionYawSetpoint &destination, uint8_t &safe_point_index)
{
	findRtlDestinationForType(_param_rtl_type.get(), destination_type, destination, safe_point_index);
}

void RTL::setLandPosAsDestination(PositionYawSetpoint &rtl_position, mission_item_s &land_mission_item) const
{
	rtl_position.alt = land_mission_item.altitude_is_relative ?	land_mission_item.altitude +
			   _home_pos_sub.get().alt : land_mission_item.altitude;
	rtl_position.lat = land_mission_item.lat;
	rtl_position.lon = land_mission_item.lon;
}

void RTL::setSafepointAsDestination(PositionYawSetpoint &rtl_position, const mission_item_s &mission_safe_point) const
{
	// There is a safe point closer than home/mission landing
	// TODO: handle all possible mission_safe_point.frame cases
	switch (mission_safe_point.frame) {
	case 0: // MAV_FRAME_GLOBAL
	case 5: // MAV_FRAME_GLOBAL_INT
		rtl_position.lat = mission_safe_point.lat;
		rtl_position.lon = mission_safe_point.lon;
		rtl_position.alt = mission_safe_point.altitude;	// alt of safe point is relative to MSL
		break;

	case 3: // MAV_FRAME_GLOBAL_RELATIVE_ALT
	case 6: // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
		rtl_position.lat = mission_safe_point.lat;
		rtl_position.lon = mission_safe_point.lon;
		rtl_position.alt = mission_safe_point.altitude + _home_pos_sub.get().alt; // alt of safe point is rel to home
		break;

	default:
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unsupported MAV_FRAME\t");
		events::send<uint8_t>(events::ID("rtl_unsupported_mav_frame"), events::Log::Error, "RTL: unsupported MAV_FRAME ({1})",
				      mission_safe_point.frame);
		break;
	}
}

float RTL::computeReturnAltitude(const PositionYawSetpoint &rtl_position) const
{
	if (_param_rtl_cone_ang.get() > 0 && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		// horizontal distance to destination
		const float destination_dist =
			get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, rtl_position.lat, rtl_position.lon);

		// minium rtl altitude to use when outside of horizontal acceptance radius of target position.
		// We choose the minimum height to be two times the distance from the land position in order to
		// avoid the vehicle touching the ground while still moving horizontally.
		const float return_altitude_min_outside_acceptance_rad_amsl = rtl_position.alt + 2.0f * _param_nav_acc_rad.get();

		const float max_return_altitude = rtl_position.alt + _param_rtl_return_alt.get();

		float return_altitude_amsl = max_return_altitude;

		if (destination_dist <= _param_nav_acc_rad.get()) {
			return_altitude_amsl = rtl_position.alt + 2.0f * destination_dist;

		} else {
			if (destination_dist <= _param_rtl_min_dist.get()) {

				// constrain cone half angle to meaningful values. All other cases are already handled above.
				const float cone_half_angle_rad = radians(constrain((float)_param_rtl_cone_ang.get(), 1.0f, 89.0f));

				// minimum altitude we need in order to be within the user defined cone
				const float cone_intersection_altitude_amsl = destination_dist / tanf(cone_half_angle_rad) + rtl_position.alt;

				return_altitude_amsl = min(cone_intersection_altitude_amsl, return_altitude_amsl);
			}

			return_altitude_amsl = max(return_altitude_amsl, return_altitude_min_outside_acceptance_rad_amsl);
		}

		return constrain(return_altitude_amsl, _global_pos_sub.get().alt, max_return_altitude);

	} else {
		// standard behaviour: return altitude above rtl destination
		return max(_global_pos_sub.get().alt, rtl_position.alt + _param_rtl_return_alt.get());
	}
}

bool RTL::initRtlMissionType(RtlType new_rtl_type, float rtl_alt)
{
	if (_rtl_mission_type_handle) {
		delete _rtl_mission_type_handle;
		_rtl_mission_type_handle = nullptr;
	}

	mission_s new_mission = _mission_sub.get();
	bool initialized = false;

	switch (new_rtl_type) {
	case RtlType::RTL_DIRECT_MISSION_LAND:
		_rtl_mission_type_handle = new RtlDirectMissionLand(_navigator);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->setRtlAlt(rtl_alt);
			_rtl_mission_type_handle->initialize();
			initialized = true;
		}

		// RTL type is either direct or mission land have to set it later.
		break;

	case RtlType::RTL_MISSION_FAST:
		_rtl_mission_type_handle = new RtlMissionFast(_navigator, new_mission);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->initialize();
			initialized = true;
		}

		break;

	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_type_handle = new RtlMissionFastReverse(_navigator, new_mission);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->initialize();
			initialized = true;
		}

		break;

	case RtlType::RTL_MISSION_SAFE_POINT_FOLLOW:
		_rtl_mission_type_handle = new RtlMissionSafePointFollow(_navigator, new_mission);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->setRoutePlan(_route_safe_point_plan);
			_rtl_mission_type_handle->setShouldGoStraightToGoal(_should_go_straight_to_safe_point);
			_rtl_mission_type_handle->initialize();
			initialized = true;
		}

		break;

	default:
		break;
	}

	return initialized;
}

void RTL::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

		if (!isActive()) {
			setRtlTypeAndDestination();
		}
	}
}

bool RTL::hasMissionLandStart() const
{
	return _mission_sub.get().land_start_index >= 0 && _mission_sub.get().land_index >= 0
	       && _navigator->get_mission_result()->valid;
}

bool RTL::reverseIsFurther() const
{
	return (_mission_sub.get().land_start_index - _mission_sub.get().current_seq) < _mission_sub.get().current_seq;
}

loiter_point_s RTL::selectLandingApproach(const PositionYawSetpoint &destination) const
{
	loiter_point_s landing_approach{};

	if (!_vehicle_status_sub.get().is_vtol
	    || (_vehicle_status_sub.get().vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
		return landing_approach;
	}

	MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();

	if (mission_route_cache == nullptr) {
		return landing_approach;
	}

	const land_approaches_s vtol_land_approaches =
		mission_route_cache->readVtolLandApproaches(destination, _home_pos_sub.get().alt);

	if (vtol_land_approaches.isAnyApproachValid()) {
		landing_approach = chooseBestLandingApproach(vtol_land_approaches);
	}

	return landing_approach;
}

loiter_point_s RTL::chooseBestLandingApproach(const land_approaches_s &vtol_land_approaches) const
{
	if (!vtol_land_approaches.land_location_lat_lon.isAllFinite()) {
		return loiter_point_s();
	}

	const float wind_direction = atan2f(_wind_sub.get().windspeed_east, _wind_sub.get().windspeed_north);
	int8_t min_index = -1;
	float wind_angle_prev = INFINITY;

	for (int i = 0; i < vtol_land_approaches.num_approaches_max; i++) {

		if (vtol_land_approaches.approaches[i].isValid()) {
			const float wind_angle = wrap_pi(get_bearing_to_next_waypoint(vtol_land_approaches.land_location_lat_lon(0),
							 vtol_land_approaches.land_location_lat_lon(1), vtol_land_approaches.approaches[i].lat,
							 vtol_land_approaches.approaches[i].lon) - wind_direction);

			if (fabsf(wind_angle) < wind_angle_prev) {
				min_index = i;
				wind_angle_prev = fabsf(wind_angle);
			}

		}
	}

	if (min_index >= 0) {
		return vtol_land_approaches.approaches[min_index];

	} else {

		return loiter_point_s();
	}
}
