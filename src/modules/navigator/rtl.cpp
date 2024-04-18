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

RTL::RTL(Navigator *navigator) :
	NavigatorMode(navigator),
	ModuleParams(navigator),
	_rtl_direct(navigator)
{
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

void RTL::on_inactivation()
{
	switch (_rtl_type) {
	case RtlType::RTL_MISSION_FAST: // Fall through
	case RtlType::RTL_MISSION_FAST_REVERSE: // Fall through
	case RtlType::RTL_DIRECT_MISSION_LAND:
		_rtl_mission_type_handle->on_inactivation();
		break;

	case RtlType::RTL_DIRECT:
		_rtl_direct.on_inactivation();
		break;

	default:
		break;
	}
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

	switch (_rtl_type) {
	case RtlType::RTL_MISSION_FAST:
	case RtlType::RTL_MISSION_FAST_REVERSE:
	case RtlType::RTL_DIRECT_MISSION_LAND:
		_rtl_mission_type_handle->on_inactive();
		break;

	case RtlType::RTL_DIRECT:
		_rtl_direct.on_inactive();
		break;

	default:
		break;
	}

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
			estimated_time = _rtl_mission_type_handle->calc_rtl_time_estimate();
			break;

		default:
			break;
		}
	}

	_rtl_time_estimate_pub.publish(estimated_time);
}

void RTL::on_activation()
{
	setRtlTypeAndDestination();

	switch (_rtl_type) {
	case RtlType::RTL_DIRECT_MISSION_LAND:	// Fall through
	case RtlType::RTL_MISSION_FAST: // Fall through
	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_type_handle->setReturnAltMin(_enforce_rtl_alt);
		_rtl_mission_type_handle->on_activation();
		break;

	case RtlType::RTL_DIRECT:
		_rtl_direct.setReturnAltMin(_enforce_rtl_alt);
		_rtl_direct.on_activation();
		break;

	default:
		break;
	}

	// set gimbal to neutral position (level with horizon) to reduce change of damage on landing
	_navigator->acquire_gimbal_control();
	_navigator->set_gimbal_neutral();
	_navigator->release_gimbal_control();
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
	case RtlType::RTL_MISSION_FAST:
	case RtlType::RTL_MISSION_FAST_REVERSE:
	case RtlType::RTL_DIRECT_MISSION_LAND:
		_rtl_mission_type_handle->on_active();
		break;

	case RtlType::RTL_DIRECT:
		_rtl_direct.on_active();
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

void RTL::setRtlTypeAndDestination()
{

	init_rtl_mission_type();

	uint8_t safe_point_index{0U};

	if (_param_rtl_type.get() != 2) {
		// check the closest allowed destination.
		DestinationType destination_type{DestinationType::DESTINATION_TYPE_HOME};
		PositionYawSetpoint rtl_position;
		float rtl_alt;
		findRtlDestination(destination_type, rtl_position, rtl_alt, safe_point_index);

		switch (destination_type) {
		case DestinationType::DESTINATION_TYPE_MISSION_LAND:
			_rtl_type = RtlType::RTL_DIRECT_MISSION_LAND;
			_rtl_mission_type_handle->setRtlAlt(rtl_alt);
			break;

		case DestinationType::DESTINATION_TYPE_SAFE_POINT: // Fallthrough
		case DestinationType::DESTINATION_TYPE_HOME: // Fallthrough
		default:

			loiter_point_s landing_loiter;
			landing_loiter.lat = rtl_position.lat;
			landing_loiter.lon = rtl_position.lon;
			landing_loiter.height_m = NAN;

			land_approaches_s rtl_land_approaches{readVtolLandApproaches(rtl_position)};

			if (_vehicle_status_sub.get().is_vtol
			    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
			    && rtl_land_approaches.isAnyApproachValid()) {
				landing_loiter = chooseBestLandingApproach(rtl_land_approaches);
			}

			_rtl_type = RtlType::RTL_DIRECT;
			_rtl_direct.setRtlAlt(rtl_alt);
			_rtl_direct.setRtlPosition(rtl_position, landing_loiter);

			break;
		}
	}

	// Publish rtl status
	_rtl_status_pub.get().timestamp = hrt_absolute_time();
	_rtl_status_pub.get().safe_points_id = _safe_points_id;
	_rtl_status_pub.get().is_evaluation_pending = _dataman_state != DatamanState::UpdateRequestWait;
	_rtl_status_pub.get().has_vtol_approach = false;

	if ((_param_rtl_type.get() == 0) || (_param_rtl_type.get() == 3)) {
		_rtl_status_pub.get().has_vtol_approach = _home_has_land_approach || _one_rally_point_has_land_approach;

	} else if (_param_rtl_type.get() == 1) {
		_rtl_status_pub.get().has_vtol_approach = _one_rally_point_has_land_approach;
	}

	_rtl_status_pub.get().rtl_type = static_cast<uint8_t>(_rtl_type);
	_rtl_status_pub.get().safe_point_index = safe_point_index;

	_rtl_status_pub.update();

}

void RTL::findRtlDestination(DestinationType &destination_type, PositionYawSetpoint &rtl_position, float &rtl_alt,
			     uint8_t &safe_point_index)
{
	// set destination to home per default, then check if other valid landing spot is closer
	rtl_position.alt = _home_pos_sub.get().alt;
	rtl_position.lat = _home_pos_sub.get().lat;
	rtl_position.lon = _home_pos_sub.get().lon;
	rtl_position.yaw = _home_pos_sub.get().yaw;
	destination_type = DestinationType::DESTINATION_TYPE_HOME;

	const bool vtol_in_rw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	// get distance to home position
	float home_dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, rtl_position.lat, rtl_position.lon)};
	float min_dist;

	_home_has_land_approach = hasVtolLandApproach(rtl_position);

	if (((_param_rtl_type.get() == 1) && !vtol_in_rw_mode) || (vtol_in_fw_mode && (_param_rtl_approach_force.get() == 1)
			&& !_home_has_land_approach)) {
		// Set minimum distance to maximum value when RTL_TYPE is set to 1 and we are not in RW mode or we forces approach landing for vtol in fw and it is not defined for home.
		min_dist = FLT_MAX;

	} else {
		min_dist = home_dist;
	}

	// consider the mission landing if available and allowed
	if (((_param_rtl_type.get() == 1) || (_param_rtl_type.get() == 3) || (fabsf(FLT_MAX - min_dist) < FLT_EPSILON))
	    && hasMissionLandStart()) {
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

			setLandPosAsDestination(rtl_position, land_mission_item);
			destination_type = DestinationType::DESTINATION_TYPE_MISSION_LAND;
		}
	}

	if (_safe_points_updated) {

		_one_rally_point_has_land_approach = false;

		for (int current_seq = 0; current_seq < _dataman_cache_safepoint.size(); ++current_seq) {
			mission_item_s mission_safe_point;

			bool success = _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_stats.dataman_id), current_seq,
					reinterpret_cast<uint8_t *>(&mission_safe_point),
					sizeof(mission_item_s), 500_ms);

			if (!success) {
				PX4_ERR("dm_read failed");
				continue;
			}

			// Ignore safepoints which are too close to the homepoint
			const float dist_to_home = get_distance_to_next_waypoint(_home_pos_sub.get().lat, _home_pos_sub.get().lon,
						   mission_safe_point.lat, mission_safe_point.lon);

			if (mission_safe_point.nav_cmd == NAV_CMD_RALLY_POINT && dist_to_home > MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES) {
				float dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, mission_safe_point.lat, mission_safe_point.lon)};

				PositionYawSetpoint safepoint_position;
				setSafepointAsDestination(safepoint_position, mission_safe_point);

				bool current_safe_point_has_approaches{hasVtolLandApproach(safepoint_position)};

				_one_rally_point_has_land_approach |= current_safe_point_has_approaches;

				if (((dist + MIN_DIST_THRESHOLD) < min_dist) && (!vtol_in_fw_mode || (_param_rtl_approach_force.get() == 0)
						|| current_safe_point_has_approaches)) {
					min_dist = dist;
					rtl_position = safepoint_position;
					destination_type = DestinationType::DESTINATION_TYPE_SAFE_POINT;
					safe_point_index = current_seq;
				}
			}
		}
	}

	if (_param_rtl_cone_half_angle_deg.get() > 0
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		rtl_alt = calculate_return_alt_from_cone_half_angle(rtl_position, (float)_param_rtl_cone_half_angle_deg.get());

	} else {
		rtl_alt = max(_global_pos_sub.get().alt, rtl_position.alt + _param_rtl_return_alt.get());
	}
}

void RTL::setLandPosAsDestination(PositionYawSetpoint &rtl_position, mission_item_s &land_mission_item) const
{
	rtl_position.alt = land_mission_item.altitude_is_relative ?	land_mission_item.altitude +
			   _home_pos_sub.get().alt : land_mission_item.altitude;
	rtl_position.lat = land_mission_item.lat;
	rtl_position.lon = land_mission_item.lon;
	rtl_position.yaw = _home_pos_sub.get().yaw;
}

void RTL::setSafepointAsDestination(PositionYawSetpoint &rtl_position,
				    const mission_item_s &mission_safe_point) const
{
	// There is a safe point closer than home/mission landing
	// TODO: handle all possible mission_safe_point.frame cases
	switch (mission_safe_point.frame) {
	case 0: // MAV_FRAME_GLOBAL
		rtl_position.lat = mission_safe_point.lat;
		rtl_position.lon = mission_safe_point.lon;
		rtl_position.alt = mission_safe_point.altitude;
		rtl_position.yaw = _home_pos_sub.get().yaw;;
		break;

	case 3: // MAV_FRAME_GLOBAL_RELATIVE_ALT
		rtl_position.lat = mission_safe_point.lat;
		rtl_position.lon = mission_safe_point.lon;
		rtl_position.alt = mission_safe_point.altitude + _home_pos_sub.get().alt; // alt of safe point is rel to home
		rtl_position.yaw = _home_pos_sub.get().yaw;;
		break;

	default:
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unsupported MAV_FRAME\t");
		events::send<uint8_t>(events::ID("rtl_unsupported_mav_frame"), events::Log::Error, "RTL: unsupported MAV_FRAME ({1})",
				      mission_safe_point.frame);
		break;
	}
}

float RTL::calculate_return_alt_from_cone_half_angle(const PositionYawSetpoint &rtl_position,
		float cone_half_angle_deg) const
{
	// horizontal distance to destination
	const float destination_dist = get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
				       rtl_position.lat, rtl_position.lon);

	// minium rtl altitude to use when outside of horizontal acceptance radius of target position.
	// We choose the minimum height to be two times the distance from the land position in order to
	// avoid the vehicle touching the ground while still moving horizontally.
	const float return_altitude_min_outside_acceptance_rad_amsl = rtl_position.alt + 2.0f * _param_nav_acc_rad.get();

	float return_altitude_amsl = rtl_position.alt + _param_rtl_return_alt.get();

	if (destination_dist <= _param_nav_acc_rad.get()) {
		return_altitude_amsl = rtl_position.alt + 2.0f * destination_dist;

	} else {

		if (destination_dist <= _param_rtl_min_dist.get()) {

			// constrain cone half angle to meaningful values. All other cases are already handled above.
			const float cone_half_angle_rad = radians(constrain(cone_half_angle_deg, 1.0f, 89.0f));

			// minimum altitude we need in order to be within the user defined cone
			const float cone_intersection_altitude_amsl = destination_dist / tanf(cone_half_angle_rad) + rtl_position.alt;

			return_altitude_amsl = min(cone_intersection_altitude_amsl, return_altitude_amsl);
		}

		return_altitude_amsl = max(return_altitude_amsl, return_altitude_min_outside_acceptance_rad_amsl);
	}

	return max(return_altitude_amsl, _global_pos_sub.get().alt);
}

void RTL::init_rtl_mission_type()
{
	RtlType new_rtl_mission_type{RtlType::RTL_DIRECT_MISSION_LAND};

	if (_param_rtl_type.get() == 2) {
		if (hasMissionLandStart()) {
			new_rtl_mission_type = RtlType::RTL_MISSION_FAST;

		} else {
			new_rtl_mission_type = RtlType::RTL_MISSION_FAST_REVERSE;
		}
	}

	if (_set_rtl_mission_type == new_rtl_mission_type) {
		return;
	}

	if (_rtl_mission_type_handle) {
		delete _rtl_mission_type_handle;
		_rtl_mission_type_handle = nullptr;
		_set_rtl_mission_type = RtlType::NONE;
	}

	switch (new_rtl_mission_type) {
	case RtlType::RTL_DIRECT_MISSION_LAND:
		_rtl_mission_type_handle = new RtlDirectMissionLand(_navigator);
		_set_rtl_mission_type = RtlType::RTL_DIRECT_MISSION_LAND;
		// RTL type is either direct or mission land have to set it later.
		break;

	case RtlType::RTL_MISSION_FAST:
		_rtl_mission_type_handle = new RtlMissionFast(_navigator);
		_set_rtl_mission_type = RtlType::RTL_MISSION_FAST;
		_rtl_type = RtlType::RTL_MISSION_FAST;
		break;

	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_type_handle = new RtlMissionFastReverse(_navigator);
		_set_rtl_mission_type = RtlType::RTL_MISSION_FAST_REVERSE;
		_rtl_type = RtlType::RTL_MISSION_FAST_REVERSE;
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
