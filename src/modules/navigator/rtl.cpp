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

static constexpr float MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES{10.0f}; // [m] We don't consider safe points valid if the distance from the current home to the safe point is smaller than this distance
static constexpr float MIN_DIST_THRESHOLD = 2.f;

// Named constants for RTL_TYPE parameter values (must match @value tags in rtl_params.c).
static constexpr int RTL_TYPE_MISSION_FAST = 2;
static constexpr int RTL_TYPE_MISSION_FAST_OR_REVERSE = 4;
static constexpr int RTL_TYPE_SAFE_POINT_DIRECT = 5;
static constexpr int RTL_TYPE_ROUTE_SAFE_POINT = 6;

namespace
{

/**
 * Provider using pre-loaded DatamanCache instances.
 *
 * All reads use loadWait(timeout=0) for cache lookups that return
 * instantly on hit and false on miss.  The caller must ensure both caches
 * are updated (via updateDatamanCache) before invoking the planner.
 */
class RtlRoutePlannerProvider : public RtlRoutePlanner::Provider
{
public:
	RtlRoutePlannerProvider(const mission_s &mission,
				DatamanCache &mission_cache,
				DatamanCache &safe_point_cache,
				const mission_stats_entry_s &safe_point_stats) :
		_mission(mission),
		_mission_cache(mission_cache),
		_safe_point_cache(safe_point_cache),
		_safe_point_stats(safe_point_stats)
	{
	}

	int missionCount() const override
	{
		return _mission.count;
	}

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		return index >= 0
		       && index < _mission.count
		       && _mission_cache.loadWait(static_cast<dm_item_t>(_mission.mission_dataman_id), index,
						  reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item));
	}

	int safePointCount() const override
	{
		return _safe_point_stats.num_items;
	}

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		return index >= 0
		       && index < _safe_point_stats.num_items
		       && _safe_point_cache.loadWait(static_cast<dm_item_t>(_safe_point_stats.dataman_id), index,
						     reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item));
	}

private:
	const mission_s &_mission;
	DatamanCache &_mission_cache;
	DatamanCache &_safe_point_cache;
	const mission_stats_entry_s &_safe_point_stats;
};

} // namespace

RTL::RTL(Navigator *navigator) :
	NavigatorMode(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL),
	ModuleParams(navigator),
	_rtl_direct(navigator)
{
	_rtl_direct.initialize();
}

void RTL::updateDatamanCache()
{
	bool success;

	switch (_dataman_state) {

	case DatamanState::UpdateRequestWait:

		if (_initiate_safe_points_updated) {
			_initiate_safe_points_updated = false;
			_dataman_state	= DatamanState::Read;
		}

		break;

	case DatamanState::Read:

		_dataman_state	= DatamanState::ReadWait;
		success = _dataman_client_safepoint.readAsync(DM_KEY_SAFE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&_stats),
				sizeof(mission_stats_entry_s));

		if (!success) {
			_error_state = DatamanState::Read;
			_dataman_state = DatamanState::Error;
		}

		break;

	case DatamanState::ReadWait:

		_dataman_client_safepoint.update();

		if (_dataman_client_safepoint.lastOperationCompleted(success)) {

			if (!success) {
				_error_state = DatamanState::ReadWait;
				_dataman_state = DatamanState::Error;

			} else if (_opaque_id != _stats.opaque_id) {

				_opaque_id = _stats.opaque_id;
				_safe_points_updated = false;

				// Safe-point set changed: drop any cached route plan that may reference
				// now-stale safe-point positions or indices.
				resetRouteSafePointCache();

				_dataman_cache_safepoint.invalidate();

				if (_dataman_cache_safepoint.size() != _stats.num_items) {
					_dataman_cache_safepoint.resize(_stats.num_items);
				}

				for (int index = 0; index < _dataman_cache_safepoint.size(); ++index) {
					_dataman_cache_safepoint.load(static_cast<dm_item_t>(_stats.dataman_id), index);
				}

				_dataman_state = DatamanState::Load;

			} else {
				_dataman_state = DatamanState::UpdateRequestWait;
				_safe_points_updated = true; // Cache already reflects the current safe-point set (unchanged or empty)
			}
		}

		break;

	case DatamanState::Load:

		_dataman_cache_safepoint.update();

		if (!_dataman_cache_safepoint.isLoading()) {
			_dataman_state = DatamanState::UpdateRequestWait;
			_safe_points_updated = true;
		}

		break;

	case DatamanState::Error:
		PX4_ERR("Safe points update failed! state: %" PRIu8, static_cast<uint8_t>(_error_state));
		_dataman_state = DatamanState::UpdateRequestWait;
		break;

	default:
		break;

	}

	// Mission item cache pre-loading
	if (_mission_id != _mission_sub.get().mission_id) {
		_mission_id = _mission_sub.get().mission_id;
		resetRouteSafePointCache();
		const dm_item_t dm_item = static_cast<dm_item_t>(_mission_sub.get().mission_dataman_id);
		_dataman_cache_landItem.invalidate();

		if (_mission_sub.get().land_index > 0) {
			_dataman_cache_landItem.load(dm_item, _mission_sub.get().land_index);
		}

		// Pre-load all mission items for non-blocking route planning (RTL type 6).
		_dataman_cache_mission.invalidate();
		const int mission_count = _mission_sub.get().count;

		if (_dataman_cache_mission.size() != mission_count) {
			_dataman_cache_mission.resize(mission_count);
		}

		for (int i = 0; i < mission_count; ++i) {
			_dataman_cache_mission.load(dm_item, i);
		}

		_mission_items_updated = (mission_count == 0);
	}

	_dataman_cache_landItem.update();
	_dataman_cache_mission.update();

	if (!_mission_items_updated && !_dataman_cache_mission.isLoading()) {
		_mission_items_updated = true;
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
 * @brief Preserve the currently branched-off SRP state before the nested mode is deactivated.
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
			PX4_DEBUG("RTL type 6 active: goal=%s safe_point=%d target=%d rev=%u direct=%u branch_off[%d,%d]",
				  RtlRoutePlanner::goalTypeString(_route_safe_point_plan.selection.goal_type),
				  static_cast<int>(_route_safe_point_plan.selection.safe_point_index),
				  static_cast<int>(_route_safe_point_plan.selection.path.first_item_index),
				  static_cast<unsigned>(_route_safe_point_plan.selection.path.direction_reversed),
				  static_cast<unsigned>(_should_go_straight_to_safe_point),
				  static_cast<int>(_route_safe_point_plan.selection.branch_off_segment.start.idx),
				  static_cast<int>(_route_safe_point_plan.selection.branch_off_segment.end.idx));
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
 * @brief Recompute the active RTL variant and destination from the current mission, safe points, and cached SRP plan.
 */
void RTL::setRtlTypeAndDestination()
{
	uint8_t safe_point_index = UINT8_MAX;
	RtlType new_rtl_type{RtlType::RTL_DIRECT};
	const RtlRoutePlanner::Plan cached_route_safe_point_plan = _route_safe_point_plan;
	_route_safe_point_plan = {};
	const bool cached_should_go_straight_to_safe_point = _should_go_straight_to_safe_point;

	if (_param_rtl_type.get() != RTL_TYPE_ROUTE_SAFE_POINT) {
		_should_go_straight_to_safe_point = false;
	}

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

		if (_navigator->get_mission_result()->valid && _mission_items_updated && _safe_points_updated) {
			RtlRoutePlannerProvider planner_provider(_mission_sub.get(), _dataman_cache_mission,
					_dataman_cache_safepoint, _stats);
			RtlRoutePlanner planner(planner_provider);
			const auto &vehicle_status = _vehicle_status_sub.get();
			const auto *local_position = _navigator->get_local_position();

			RtlRoutePlanner::Config config{};
			config.vehicle_projection_search_dist = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
								? _param_rtl_mc_seg_dist.get() : _param_rtl_fw_seg_dist.get();
			config.safe_point_projection_search_dist = _param_rtl_rp_seg_dist.get();
			config.acceptance_radius = _navigator->get_acceptance_radius();
			config.direct_acceptance_radius = _navigator->get_default_acceptance_radius();
			config.home_altitude_amsl = _home_pos_sub.get().alt;
			config.is_multicopter = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						&& !vehicle_status.in_transition_mode;
			config.is_flying_reverse = cached_route_safe_point_plan.valid()
						   ? cached_route_safe_point_plan.selection.path.direction_reversed : false;
			config.vehicle_velocity_valid = PX4_ISFINITE(local_position->vx) && PX4_ISFINITE(local_position->vy);
			config.vehicle_velocity_north = local_position->vx;
			config.vehicle_velocity_east = local_position->vy;
			config.vehicle_is_vtol = vehicle_status.is_vtol;
			config.vehicle_is_fixed_wing = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
			config.vehicle_in_transition_to_fw = vehicle_status.in_transition_to_fw;
			config.u_turn_penalty_m = _param_rtl_fw_uturn_pen.get();
			config.last_flown_loop_segment = _last_route_safe_point_loop_segment;

			RtlRoutePlanner::Position vehicle_position{
				_global_pos_sub.get().lat,
				_global_pos_sub.get().lon,
				_global_pos_sub.get().alt
			};

			const bool reuse_cached_plan = cached_should_go_straight_to_safe_point
						       && cached_route_safe_point_plan.valid()
						       && cached_route_safe_point_plan.selection.safe_point_found
						       && planner.closeToBranchOffSegment(vehicle_position, cached_route_safe_point_plan.selection,
								       config.acceptance_radius);

			if (reuse_cached_plan) {
				_route_safe_point_plan = cached_route_safe_point_plan;
				_should_go_straight_to_safe_point = true;

				int32_t current_position_index = _mission_sub.get().current_seq;
				mission_item_s mission_item{};

				while (current_position_index >= 0
				       && planner_provider.loadMissionItem(current_position_index, mission_item)) {
					if (RtlRoutePlanner::itemContainsPosition(mission_item)) {
						_route_safe_point_plan.selection.path.first_item_index = current_position_index;
						_route_safe_point_plan.selection.path.first_item_cmd = mission_item.nav_cmd;
						break;
					}

					--current_position_index;
				}

				PX4_DEBUG("RTL type 6 reusing cached branch-off and flying straight to goal");

			} else {
				RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};

				if (planner.planRouteToGoal(vehicle_position, _mission_sub.get().current_seq,
							    config, _route_safe_point_plan, &failure_reason)) {
					if (_route_safe_point_plan.selection.path.in_first_item_acc_rad
					    && _route_safe_point_plan.selection.goal_type != RtlRoutePlanner::GoalType::SafePoint) {
						_route_safe_point_plan.join_context.skip_altitude_requirement = true;
					}

					_should_go_straight_to_safe_point = cached_should_go_straight_to_safe_point
									    || _route_safe_point_plan.selection.direct_to_safe_point;

					PX4_DEBUG("RTL type 6 planned goal=%s target=%d rev=%u direct=%u bt=%u",
						  RtlRoutePlanner::goalTypeString(_route_safe_point_plan.selection.goal_type),
						  static_cast<int>(_route_safe_point_plan.selection.path.first_item_index),
						  static_cast<unsigned>(_route_safe_point_plan.selection.path.direction_reversed),
						  static_cast<unsigned>(_should_go_straight_to_safe_point),
						  static_cast<unsigned>(_route_safe_point_plan.join_context.vtol_back_transition_required));

				} else {
					PX4_WARN("RTL type 6 planning failed: %s", RtlRoutePlanner::failureReasonString(failure_reason));
				}
			}
		}

		if (_route_safe_point_plan.valid()) {
			const bool direction_will_change = current_route_direction_reversed
							   != _route_safe_point_plan.selection.path.direction_reversed;

			if (isActive() && direction_will_change && _vehicle_status_sub.get().in_transition_to_fw) {
				vehicle_command_s cmd{};
				cmd.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
				cmd.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
				cmd.param2 = 0.f;
				_navigator->publish_vehicle_command(cmd);
				PX4_DEBUG("RTL type 6 direction changed during FT, applying immediate BT");
			}

			new_rtl_type = RtlType::RTL_MISSION_SAFE_POINT_FOLLOW;
			_last_route_safe_point_loop_segment = _route_safe_point_plan.projection_context.projection.segment;
			destination_type = _route_safe_point_plan.selection.safe_point_found
					   ? DestinationType::DESTINATION_TYPE_SAFE_POINT
					   : DestinationType::DESTINATION_TYPE_MISSION_LAND;
			destination.lat = _route_safe_point_plan.selection.goal_position.lat;
			destination.lon = _route_safe_point_plan.selection.goal_position.lon;
			destination.alt = _route_safe_point_plan.selection.goal_position.alt;
			destination.yaw = NAN;
			safe_point_index = _route_safe_point_plan.selection.safe_point_found
					   ? static_cast<uint8_t>(constrain(_route_safe_point_plan.selection.safe_point_index,
							   0, static_cast<int32_t>(UINT8_MAX)))
					   : UINT8_MAX;

		} else {
			_should_go_straight_to_safe_point = false;
			findRtlDestination(destination_type, destination, safe_point_index);

			if (destination_type == DestinationType::DESTINATION_TYPE_MISSION_LAND) {
				new_rtl_type = RtlType::RTL_DIRECT_MISSION_LAND;

			} else {
				new_rtl_type = RtlType::RTL_DIRECT;
			}
		}

	} else {
		// RTL_TYPE 0, 1, 3, 5: use closest safe point / home / mission landing via direct path.
		findRtlDestination(destination_type, destination, safe_point_index);

		if (destination_type == DestinationType::DESTINATION_TYPE_MISSION_LAND) {
			new_rtl_type = RtlType::RTL_DIRECT_MISSION_LAND;

		} else {

			new_rtl_type = RtlType::RTL_DIRECT;

			land_approaches_s rtl_land_approaches{readVtolLandApproaches(destination)};

			// set loiter position to destination initially, overwrite for VTOL if land approaches exist
			landing_loiter.lat = destination.lat;
			landing_loiter.lon = destination.lon;
			landing_loiter.height_m = NAN;

			if (_vehicle_status_sub.get().is_vtol
			    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
			    && rtl_land_approaches.isAnyApproachValid()) {
				landing_loiter = chooseBestLandingApproach(rtl_land_approaches);
			}
		}
	}

	if (new_rtl_type == RtlType::RTL_DIRECT) {
		land_approaches_s rtl_land_approaches{readVtolLandApproaches(destination)};

		landing_loiter.lat = destination.lat;
		landing_loiter.lon = destination.lon;
		landing_loiter.height_m = NAN;

		if (_vehicle_status_sub.get().is_vtol
		    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
		    && rtl_land_approaches.isAnyApproachValid()) {
			landing_loiter = chooseBestLandingApproach(rtl_land_approaches);
		}
	}

	const float rtl_alt = computeReturnAltitude(destination);
	_rtl_direct.setRtlAlt(rtl_alt);
	_rtl_direct.setRtlPosition(destination, landing_loiter);

	const bool new_type_is_mission_based = (new_rtl_type == RtlType::RTL_MISSION_FAST)
					       || (new_rtl_type == RtlType::RTL_MISSION_FAST_REVERSE)
					       || (new_rtl_type == RtlType::RTL_DIRECT_MISSION_LAND)
					       || (new_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);

	if (new_type_is_mission_based && (_rtl_type != new_rtl_type)) {
		initRtlMissionType(new_rtl_type, rtl_alt);
	}

	// setRoutePlan is called unconditionally (even when the type hasn't changed) so that
	// a replanned route is pushed to the executor.  The executor's state machine in
	// RtlMissionSafePointFollow::setRoutePlan handles mid-flight plan updates correctly
	// because it only caches the plan and branch-off index without resetting the stage.
	if (new_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW && _rtl_mission_type_handle) {
		_rtl_mission_type_handle->setRoutePlan(_route_safe_point_plan);
	}

	_rtl_type = new_rtl_type;

	// Publish rtl status
	rtl_status_s rtl_status{};
	rtl_status.safe_points_id = _safe_points_id;
	rtl_status.is_evaluation_pending = _dataman_state != DatamanState::UpdateRequestWait;
	rtl_status.has_vtol_approach = _home_has_land_approach || _one_rally_point_has_land_approach;
	rtl_status.rtl_type = static_cast<uint8_t>(_rtl_type);
	rtl_status.safe_point_index = safe_point_index;
	rtl_status.timestamp = hrt_absolute_time();
	_rtl_status_pub.publish(rtl_status);
}

PositionYawSetpoint RTL::findClosestSafePoint(float min_dist, uint8_t &safe_point_index)
{
	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	PositionYawSetpoint safe_point{(double)NAN, (double)NAN, NAN, NAN};

	if (_safe_points_updated) {
		_one_rally_point_has_land_approach = false;

		for (int current_seq = 0; current_seq < _dataman_cache_safepoint.size(); ++current_seq) {
			mission_item_s mission_safe_point;

			const bool success = _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_stats.dataman_id), current_seq,
					     reinterpret_cast<uint8_t *>(&mission_safe_point),
					     sizeof(mission_item_s), 500_ms);

			if (!success) {
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

				const bool current_safe_point_has_approaches{hasVtolLandApproach(safepoint_position)};

				_one_rally_point_has_land_approach |= current_safe_point_has_approaches;

				if (((dist + MIN_DIST_THRESHOLD) < min_dist)
				    && (!vtol_in_fw_mode || (_param_rtl_appr_force.get() == 0) || current_safe_point_has_approaches)) {
					min_dist = dist;
					safe_point = safepoint_position;
					safe_point_index = current_seq;
				}
			}
		}
	}

	return safe_point;
}

void RTL::findRtlDestination(DestinationType &destination_type, PositionYawSetpoint &destination, uint8_t &safe_point_index)
{
	const bool vtol_in_rw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	float min_dist = FLT_MAX;

	if (_param_rtl_type.get() != RTL_TYPE_SAFE_POINT_DIRECT) {
		_home_has_land_approach = hasVtolLandApproach(destination);

		const bool prioritize_safe_points_over_home = ((_param_rtl_type.get() == 1) && !vtol_in_rw_mode);
		const bool required_approach_missing_for_home = (vtol_in_fw_mode && (_param_rtl_appr_force.get() == 1) && !_home_has_land_approach);

		// Set minimum distance to maximum value when RTL_TYPE is set to 1 and we are not in RW mode or we force approach landing for vtol in fw and it is not defined for home.
		const bool deprioritize_home = prioritize_safe_points_over_home || required_approach_missing_for_home;

		if (!deprioritize_home) {
			// distance to home position
			min_dist = get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, destination.lat, destination.lon);
		}

		// Mission landing
		if (((_param_rtl_type.get() == 1) || (_param_rtl_type.get() == 3) || (fabsf(FLT_MAX - min_dist) < FLT_EPSILON)) && hasMissionLandStart()) {
			mission_item_s land_mission_item;
			const dm_item_t dm_item = static_cast<dm_item_t>(_mission_sub.get().mission_dataman_id);
			bool success = _dataman_cache_landItem.loadWait(dm_item, _mission_sub.get().land_index,
					reinterpret_cast<uint8_t *>(&land_mission_item), sizeof(mission_item_s), 500_ms);

			if (!success) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission land item could not be read.\t");
				events::send(events::ID("rtl_failed_to_read_land_item"), events::Log::Error,
					     "Mission land item could not be read");
			}

			float dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, land_mission_item.lat, land_mission_item.lon)};

			if ((dist + MIN_DIST_THRESHOLD) < min_dist) {
				if (_param_rtl_type.get() != 0) {
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

void RTL::initRtlMissionType(RtlType new_rtl_type, float rtl_alt)
{
	if (_rtl_mission_type_handle) {
		delete _rtl_mission_type_handle;
		_rtl_mission_type_handle = nullptr;
	}

	mission_s new_mission = _mission_sub.get();

	switch (new_rtl_type) {
	case RtlType::RTL_DIRECT_MISSION_LAND:
		_rtl_mission_type_handle = new RtlDirectMissionLand(_navigator);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->setRtlAlt(rtl_alt);
			_rtl_mission_type_handle->initialize();
		}

		// RTL type is either direct or mission land have to set it later.
		break;

	case RtlType::RTL_MISSION_FAST:
		_rtl_mission_type_handle = new RtlMissionFast(_navigator, new_mission);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->initialize();
		}

		break;

	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_type_handle = new RtlMissionFastReverse(_navigator, new_mission);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->initialize();
		}

		break;

	case RtlType::RTL_MISSION_SAFE_POINT_FOLLOW:
		_rtl_mission_type_handle = new RtlMissionSafePointFollow(_navigator, new_mission);

		if (_rtl_mission_type_handle) {
			_rtl_mission_type_handle->setRoutePlan(_route_safe_point_plan);
			_rtl_mission_type_handle->initialize();
		}

		break;

	default:
		break;
	}
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


bool RTL::hasVtolLandApproach(const PositionYawSetpoint &rtl_position) const
{
	return readVtolLandApproaches(rtl_position).isAnyApproachValid();
}

loiter_point_s RTL::chooseBestLandingApproach(const land_approaches_s &vtol_land_approaches)
{
	const float wind_direction = atan2f(_wind_sub.get().windspeed_east, _wind_sub.get().windspeed_north);
	int8_t min_index = -1;
	float wind_angle_prev = INFINITY;

	for (int i = 0; i < vtol_land_approaches.num_approaches_max; i++) {

		if (vtol_land_approaches.approaches[i].isValid()) {
			const float wind_angle = wrap_pi(get_bearing_to_next_waypoint(_home_pos_sub.get().lat,
							 _home_pos_sub.get().lon, vtol_land_approaches.approaches[i].lat,
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

land_approaches_s RTL::readVtolLandApproaches(PositionYawSetpoint rtl_position) const
{

	// go through all mission items in the rally point storage. If we find a mission item of type NAV_CMD_RALLY_POINT
	// which is within MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES of our current home position then treat ALL following mission items of type NAV_CMD_LOITER_TO_ALT which come
	// BEFORE the next mission item of type NAV_CMD_RALLY_POINT as land approaches for the home position
	land_approaches_s vtol_land_approaches{};

	if (!_safe_points_updated) {
		return vtol_land_approaches;
	}

	bool foundHomeLandApproaches = false;
	uint8_t sector_counter = 0;

	for (int current_seq = 0; current_seq < _stats.num_items; ++current_seq) {
		mission_item_s mission_item{};

		bool success_mission_item = _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_stats.dataman_id), current_seq,
					    reinterpret_cast<uint8_t *>(&mission_item),
					    sizeof(mission_item_s));

		if (!success_mission_item) {
			PX4_ERR("dm_read failed");
			break;
		}

		if (mission_item.nav_cmd == NAV_CMD_RALLY_POINT) {

			if (foundHomeLandApproaches) {
				break;
			}

			const float dist_to_safepoint = get_distance_to_next_waypoint(mission_item.lat, mission_item.lon, rtl_position.lat,
							rtl_position.lon);

			if (dist_to_safepoint < MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES) {
				foundHomeLandApproaches = true;
				vtol_land_approaches.land_location_lat_lon = matrix::Vector2d(mission_item.lat, mission_item.lon);
			}

			sector_counter = 0;
		}

		if (foundHomeLandApproaches && mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
			vtol_land_approaches.approaches[sector_counter].lat = mission_item.lat;
			vtol_land_approaches.approaches[sector_counter].lon = mission_item.lon;
			vtol_land_approaches.approaches[sector_counter].height_m = MissionBlock::get_absolute_altitude_for_item(mission_item,
					_home_pos_sub.get().alt);
			vtol_land_approaches.approaches[sector_counter].loiter_radius_m = mission_item.loiter_radius;
			sector_counter++;
		}
	}

	return vtol_land_approaches;
}
