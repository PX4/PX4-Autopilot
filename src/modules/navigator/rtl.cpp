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
static constexpr double NULL_ISLAND_THRESHOLD_DEG{1e-12};

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

	if (_mission_id != _mission_sub.get().mission_id) {
		_mission_id = _mission_sub.get().mission_id;
		const dm_item_t dm_item = static_cast<dm_item_t>(_mission_sub.get().mission_dataman_id);
		_dataman_cache_landItem.invalidate();

		if (_mission_sub.get().land_index > 0) {
			_dataman_cache_landItem.load(dm_item, _mission_sub.get().land_index);
		}
	}

	_dataman_cache_landItem.update();
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
	case RtlType::RTL_MISSION_FAST_REVERSE:
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
	case RtlType::RTL_DIRECT_MISSION_LAND:
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

void RTL::setRtlTypeAndDestination()
{
	uint8_t safe_point_index = UINT8_MAX;
	RtlType new_rtl_type{RtlType::RTL_DIRECT};

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

	if (_param_rtl_type.get() == 2) {
		if (hasMissionLandStart()) {
			new_rtl_type = RtlType::RTL_MISSION_FAST;

		} else if (_navigator->get_mission_result()->valid) {
			new_rtl_type = RtlType::RTL_MISSION_FAST_REVERSE;

		} else {
			// no valid mission, go direct to home
			new_rtl_type = RtlType::RTL_DIRECT;
		}

	} else if (_param_rtl_type.get() == 4) {
		if (hasMissionLandStart() && reverseIsFurther()) {
			new_rtl_type = RtlType::RTL_MISSION_FAST;

		} else if (_navigator->get_mission_result()->valid) {
			new_rtl_type = RtlType::RTL_MISSION_FAST_REVERSE;

		} else {
			// no valid mission, go direct to home
			new_rtl_type = RtlType::RTL_DIRECT;
		}

	} else {
		// check the closest allowed destination.
		findRtlDestination(destination_type, destination, safe_point_index);

		if (destination_type == DestinationType::DESTINATION_TYPE_MISSION_LAND) {
			new_rtl_type = RtlType::RTL_DIRECT_MISSION_LAND;

		} else {

			new_rtl_type = RtlType::RTL_DIRECT;
			landing_loiter = selectLandingApproach(destination);

			if (!landing_loiter.isValid()) {
				landing_loiter.lat = destination.lat;
				landing_loiter.lon = destination.lon;
				landing_loiter.height_m = NAN;
			}
		}
	}

	const float rtl_alt = computeReturnAltitude(destination);
	_rtl_direct.setRtlAlt(rtl_alt);
	_rtl_direct.setRtlPosition(destination, landing_loiter);

	const bool new_type_is_mission_based = (new_rtl_type == RtlType::RTL_MISSION_FAST)
					       || (new_rtl_type == RtlType::RTL_MISSION_FAST_REVERSE)
					       || (new_rtl_type == RtlType::RTL_DIRECT_MISSION_LAND);

	if (new_type_is_mission_based && (_rtl_type != new_rtl_type)) {
		initRtlMissionType(new_rtl_type, rtl_alt);
	}

	_rtl_type = new_rtl_type;

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	// Update destination of geofence avoidance planner. Depending on the
	// RTL type it is the position of the loiter or mission landing.

	GeofenceAvoidancePlanner &planner = _navigator->get_geofence_avoidance_planner();
	matrix::Vector2d planner_destination{(double)NAN, (double)NAN};

	if (new_rtl_type == RtlType::RTL_DIRECT) {
		planner_destination = matrix::Vector2d{landing_loiter.lat, landing_loiter.lon};

	} else if (new_rtl_type == RtlType::RTL_DIRECT_MISSION_LAND && _rtl_mission_type_handle) {
		planner_destination = _rtl_mission_type_handle->getRtlPlannerDestination();
	}

	if (planner_destination.isAllFinite()) {
		planner.updateDestination(planner_destination);
	}

	const vehicle_global_position_s gpos = _global_pos_sub.get();

	const bool gpos_recent = gpos.timestamp > 0 && hrt_elapsed_time(&gpos.timestamp) < 10_s;

	if (gpos_recent) {
		planner.updateStartAndFillPath(matrix::Vector2d(_global_pos_sub.get().lat, _global_pos_sub.get().lon));
	}

#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

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

	PositionYawSetpoint safe_point{static_cast<double>(NAN), static_cast<double>(NAN), NAN, NAN};

	if (_safe_points_updated) {
		_one_rally_point_has_land_approach = false;

		for (int current_seq = 0; current_seq < _stats.num_items; ++current_seq) {
			mission_item_s mission_safe_point;

			const bool success = loadSafePointItemWait(current_seq, mission_safe_point, 500_ms);

			if (!success) {
				PX4_ERR("dm_read failed");
				continue;
			}

			// Only look at rally points
			if (mission_safe_point.nav_cmd != NAV_CMD_RALLY_POINT) {
				continue;
			}

			PositionYawSetpoint safepoint_position{};

			if (!extractValidSafePointPosition(mission_safe_point, _home_pos_sub.get().alt, safepoint_position)) {
				continue;
			}

			// Ignore safepoints which are too close to the homepoint (only if home is an option to return to)
			const bool far_from_home = get_distance_to_next_waypoint(_home_pos_sub.get().lat, _home_pos_sub.get().lon,
						   safepoint_position.lat, safepoint_position.lon) > MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES;

			if (far_from_home || (_param_rtl_type.get() == 5)) {
				const float dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
						 safepoint_position.lat, safepoint_position.lon)};

				const bool current_safe_point_has_approaches{hasVtolLandApproachesAtSafePointIndex(current_seq, _home_pos_sub.get().alt)};

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

bool RTL::loadSafePointItemWait(int seq, mission_item_s &item, hrt_abstime timeout) const
{
	return _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_stats.dataman_id), seq,
			reinterpret_cast<uint8_t *>(&item), sizeof(mission_item_s), timeout);
}

void RTL::findRtlDestination(DestinationType &destination_type, PositionYawSetpoint &destination, uint8_t &safe_point_index)
{
	const bool vtol_in_rw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	float min_dist = FLT_MAX;

	if (_param_rtl_type.get() != 5) {
		_home_has_land_approach = hasVtolLandApproachesNearLocation(destination, _home_pos_sub.get().alt);

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

	} else if (_param_rtl_type.get() == 5) {
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

bool RTL::extractValidSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl,
					PositionYawSetpoint &position) const
{
	if (safe_point_item.nav_cmd != NAV_CMD_RALLY_POINT) {
		return false;
	}

	// TODO: handle all possible safe_point_item.frame cases
	switch (safe_point_item.frame) {
	case NAV_FRAME_GLOBAL:
	case NAV_FRAME_GLOBAL_INT:
		position.alt = safe_point_item.altitude;
		break;

	case NAV_FRAME_GLOBAL_RELATIVE_ALT:
	case NAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
		if (!PX4_ISFINITE(home_altitude_amsl)) {
			return false;
		}

		position.alt = safe_point_item.altitude + home_altitude_amsl; // alt of safe point is rel to home
		break;

	default:
		return false;
	}

	position.lat = safe_point_item.lat;
	position.lon = safe_point_item.lon;
	position.yaw = NAN;

	// basic checks protect against invalid data in case of missing init or cache read errors
	return PX4_ISFINITE(position.lat) && PX4_ISFINITE(position.lon) && PX4_ISFINITE(position.alt)
	       && !((fabs(position.lat) < NULL_ISLAND_THRESHOLD_DEG) &&
		    (fabs(position.lon) < NULL_ISLAND_THRESHOLD_DEG))
	       && (fabs(position.lat) <= 90.0) && (fabs(position.lon) <= 180.0);
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


bool RTL::findAssociatedSafePointIndex(const PositionYawSetpoint &rtl_position, float home_altitude_amsl,
				       int &safe_point_index, mission_item_s &safe_point_item) const
{
	if (!_safe_points_updated) {
		return false;
	}

	// The first nearby valid rally point defines the block that belongs to this destination.
	for (int current_seq = 0; current_seq < _stats.num_items; ++current_seq) {
		mission_item_s mission_item{};

		const bool success = loadSafePointItemWait(current_seq, mission_item, 500_ms);

		if (!success) {
			PX4_ERR("dm_read failed");
			break;
		}

		if (mission_item.nav_cmd != NAV_CMD_RALLY_POINT) {
			continue;
		}

		PositionYawSetpoint safe_point_position{};

		// Skip invalid rally points so later valid rally points are considered
		if (!extractValidSafePointPosition(mission_item, home_altitude_amsl, safe_point_position)) {
			continue;
		}

		const float dist_to_safepoint = get_distance_to_next_waypoint(safe_point_position.lat, safe_point_position.lon,
						rtl_position.lat, rtl_position.lon);

		if (dist_to_safepoint < MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES) {
			safe_point_index = current_seq;
			safe_point_item = mission_item;
			return true;
		}
	}

	return false;
}

bool RTL::scanVtolLandApproachBlock(int safe_point_index, float home_altitude_amsl,
				    land_approaches_s *result) const
{
	uint8_t valid_approach_counter = 0;

	for (int current_seq = safe_point_index + 1; current_seq < _stats.num_items; ++current_seq) {
		mission_item_s mission_item{};

		const bool success = loadSafePointItemWait(current_seq, mission_item, 500_ms);

		if (!success) {
			PX4_ERR("dm_read failed");
			break;
		}

		if (mission_item.nav_cmd == NAV_CMD_RALLY_POINT) {
			// A rally point starts the next block, break
			break;
		}

		if (mission_item.nav_cmd != NAV_CMD_LOITER_TO_ALT
		    || valid_approach_counter >= land_approaches_s::num_approaches_max) {
			continue;
		}

		const loiter_point_s approach = makeVtolLandApproachPoint(mission_item, home_altitude_amsl);

		if (approach.isValid()) {
			if (result) {
				result->approaches[valid_approach_counter] = approach;
				valid_approach_counter++;

			} else {
				return true; // Early exit for the check-only
			}
		}
	}

	return result ? (valid_approach_counter > 0) : false;
}

loiter_point_s RTL::makeVtolLandApproachPoint(const mission_item_s &mission_item, float home_altitude_amsl) const
{
	loiter_point_s approach{};
	approach.lat = mission_item.lat;
	approach.lon = mission_item.lon;
	approach.height_m = MissionBlock::get_absolute_altitude_for_item(mission_item, home_altitude_amsl);
	approach.loiter_radius_m = mission_item.loiter_radius;
	return approach;
}

land_approaches_s RTL::getVtolLandApproachesNearLocation(const PositionYawSetpoint &rtl_position,
		float home_altitude_amsl) const
{
	int safe_point_index = -1;
	mission_item_s safe_point_item{};

	if (!findAssociatedSafePointIndex(rtl_position, home_altitude_amsl, safe_point_index, safe_point_item)) {
		return {};
	}

	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.land_location_lat_lon = matrix::Vector2d(safe_point_item.lat, safe_point_item.lon);
	scanVtolLandApproachBlock(safe_point_index, home_altitude_amsl, &vtol_land_approaches);
	return vtol_land_approaches;
}

bool RTL::hasVtolLandApproachesNearLocation(const PositionYawSetpoint &rtl_position, float home_altitude_amsl) const
{
	int safe_point_index = -1;
	mission_item_s safe_point_item{};

	return findAssociatedSafePointIndex(rtl_position, home_altitude_amsl, safe_point_index, safe_point_item)
	       && hasVtolLandApproachesAtSafePointIndex(safe_point_index, home_altitude_amsl);
}

bool RTL::hasVtolLandApproachesAtSafePointIndex(int safe_point_index, float home_altitude_amsl) const
{
	return scanVtolLandApproachBlock(safe_point_index, home_altitude_amsl, nullptr);
}

loiter_point_s RTL::selectLandingApproach(const PositionYawSetpoint &destination) const
{
	loiter_point_s landing_approach{};

	// Only a VTOL currently flying as fixed wing needs a wind-selected approach loiter.
	if (!_vehicle_status_sub.get().is_vtol
	    || (_vehicle_status_sub.get().vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
		return landing_approach;
	}

	const land_approaches_s vtol_land_approaches = getVtolLandApproachesNearLocation(destination, _home_pos_sub.get().alt);

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
			// The approach circles are defined around the land location.
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
