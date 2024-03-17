/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include "FeasibilityChecker.hpp"
#include <systemlib/mavlink_log.h>
#include <px4_platform_common/events.h>
#include <drivers/drv_pwm_output.h>
#include "../mission_block.h"
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

FeasibilityChecker::FeasibilityChecker() :
	ModuleParams(nullptr)
{
}

void FeasibilityChecker::reset()
{
	_mission_validity_failed = false;
	_takeoff_failed = false;
	_land_pattern_validity_failed = false;
	_distance_first_waypoint_failed = false;
	_distance_between_waypoints_failed = false;
	_fixed_wing_land_approach_failed = false;
	_takeoff_land_available_failed = false;

	_found_item_with_position = false;
	_has_vtol_takeoff = false;
	_has_takeoff = false;

	_landing_valid = false;
	_do_land_start_index = -1;
	_landing_approach_index = -1;
	_mission_item_previous = {};

	_first_waypoint_found = false;

	_last_lat = (double)NAN;
	_last_lon = (double)NAN;
	_last_cmd = -1;
}

void FeasibilityChecker::updateData()
{
	home_position_s home = {};

	if (_home_pos_sub.updated()) {
		_home_pos_sub.update(&home);

		if (home.valid_hpos) {
			_home_lat_lon = matrix::Vector2d(home.lat, home.lon);

		} else {
			_home_lat_lon = matrix::Vector2d((double)NAN, (double)NAN);
		}

		if (home.valid_alt) {
			_home_alt_msl = home.alt;

		} else {
			_home_alt_msl = NAN;
		}
	}

	vehicle_status_s status = {};

	if (_status_sub.updated()) {
		_status_sub.copy(&status);

		if (status.is_vtol) {
			_vehicle_type = VehicleType::Vtol;

		} else if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			_vehicle_type = VehicleType::RotaryWing;

		} else if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			_vehicle_type = VehicleType::FixedWing;

		} else {
			_vehicle_type = VehicleType::Other;
		}
	}

	vehicle_land_detected_s land_detected = {};

	if (_land_detector_sub.updated()) {
		_land_detector_sub.copy(&land_detected);
		_is_landed = land_detected.landed;
	}

	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position = {};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_current_position_lat_lon = matrix::Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_rtl_status_sub.updated()) {
		rtl_status_s rtl_status = {};
		_rtl_status_sub.copy(&rtl_status);
		_has_vtol_approach = rtl_status.has_vtol_approach;
	}

	param_t handle = param_find("FW_LND_ANG");

	if (handle != PARAM_INVALID) {
		param_get(handle, &_param_fw_lnd_ang);
	}

	handle = param_find("MIS_DIST_1WP");

	if (handle != PARAM_INVALID) {
		param_get(handle, &_param_mis_dist_1wp);
	}

	handle = param_find("NAV_ACC_RAD");

	if (handle != PARAM_INVALID) {
		param_get(handle, &_param_nav_acc_rad);
	}

	handle = param_find("MIS_TKO_LAND_REQ");

	if (handle != PARAM_INVALID) {
		param_get(handle, &_param_mis_takeoff_land_req);
	}
}

bool FeasibilityChecker::processNextItem(mission_item_s &mission_item, const int current_index, const int total_count)
{
	if (current_index == 0) {
		reset();
		updateData();
	}

	if (!_mission_validity_failed) {
		_mission_validity_failed = !checkMissionItemValidity(mission_item, current_index);
	}

	if (_mission_validity_failed) {
		// if a mission item is not valid then abort the other checks
		return false;
	}

	doCommonChecks(mission_item, current_index);

	if (_vehicle_type == VehicleType::Vtol) {
		doVtolChecks(mission_item, current_index, total_count - 1);

	} else if (_vehicle_type == VehicleType::FixedWing) {
		doFixedWingChecks(mission_item, current_index, total_count - 1);

	} else if (_vehicle_type == VehicleType::RotaryWing) {
		doMulticopterChecks(mission_item, current_index);
	}

	if (current_index == total_count - 1) {
		_takeoff_land_available_failed = !checkTakeoffLandAvailable();
	}

	_mission_item_previous = mission_item;

	return true;
}

void FeasibilityChecker::doCommonChecks(mission_item_s &mission_item, const int current_index)
{

	if (!_distance_between_waypoints_failed) {
		_distance_between_waypoints_failed = !checkDistancesBetweenWaypoints(mission_item);
	}

	if (!_distance_first_waypoint_failed) {
		_distance_first_waypoint_failed = !checkHorizontalDistanceToFirstWaypoint(mission_item);
	}

	if (!_takeoff_failed) {
		_takeoff_failed = !checkTakeoff(mission_item);
	}

	if (!_items_fit_to_vehicle_type_failed) {
		_items_fit_to_vehicle_type_failed = !checkItemsFitToVehicleType(mission_item);
	}
}

void FeasibilityChecker::doVtolChecks(mission_item_s &mission_item, const int current_index, const int last_index)
{
	if (!_land_pattern_validity_failed) {
		_land_pattern_validity_failed = !checkLandPatternValidity(mission_item, current_index, last_index);
	}

}

void FeasibilityChecker::doFixedWingChecks(mission_item_s &mission_item, const int current_index, const int last_index)
{
	if (!_land_pattern_validity_failed) {
		_land_pattern_validity_failed = !checkLandPatternValidity(mission_item, current_index, last_index);
	}

	if (!_fixed_wing_land_approach_failed) {
		_fixed_wing_land_approach_failed = !checkFixedWindLandApproach(mission_item, current_index);
	}

}

void FeasibilityChecker::doMulticopterChecks(mission_item_s &mission_item, const int current_index)
{
	// this flag is used for the checkTakeoffLandAvailable check at the very end
	_landing_valid |= mission_item.nav_cmd == NAV_CMD_LAND;
}

bool FeasibilityChecker::checkMissionItemValidity(mission_item_s &mission_item, const int current_index)
{
	/* reject relative alt without home set */
	if (mission_item.altitude_is_relative && !PX4_ISFINITE(_home_alt_msl)
	    && MissionBlock::item_contains_position(mission_item)) {



		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: No home pos, WP %d uses rel alt\t", current_index + 1);
		events::send<int16_t>(events::ID("navigator_mis_no_home_rel_alt"), {events::Log::Error, events::LogInternal::Info},
				      "Mission rejected: No home position, waypoint {1} uses relative altitude",
				      current_index + 1);
		return false;

	}

	// check if we find unsupported items and reject mission if so
	if (mission_item.nav_cmd != NAV_CMD_IDLE &&
	    mission_item.nav_cmd != NAV_CMD_WAYPOINT &&
	    mission_item.nav_cmd != NAV_CMD_LOITER_UNLIMITED &&
	    mission_item.nav_cmd != NAV_CMD_LOITER_TIME_LIMIT &&
	    mission_item.nav_cmd != NAV_CMD_RETURN_TO_LAUNCH &&
	    mission_item.nav_cmd != NAV_CMD_LAND &&
	    mission_item.nav_cmd != NAV_CMD_TAKEOFF &&
	    mission_item.nav_cmd != NAV_CMD_LOITER_TO_ALT &&
	    mission_item.nav_cmd != NAV_CMD_VTOL_TAKEOFF &&
	    mission_item.nav_cmd != NAV_CMD_VTOL_LAND &&
	    mission_item.nav_cmd != NAV_CMD_DELAY &&
	    mission_item.nav_cmd != NAV_CMD_CONDITION_GATE &&
	    mission_item.nav_cmd != NAV_CMD_DO_WINCH &&
	    mission_item.nav_cmd != NAV_CMD_DO_GRIPPER &&
	    mission_item.nav_cmd != NAV_CMD_DO_JUMP &&
	    mission_item.nav_cmd != NAV_CMD_DO_CHANGE_SPEED &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_HOME &&
	    mission_item.nav_cmd != NAV_CMD_DO_LAND_START &&
	    mission_item.nav_cmd != NAV_CMD_DO_TRIGGER_CONTROL &&
	    mission_item.nav_cmd != NAV_CMD_DO_DIGICAM_CONTROL &&
	    mission_item.nav_cmd != NAV_CMD_IMAGE_START_CAPTURE &&
	    mission_item.nav_cmd != NAV_CMD_IMAGE_STOP_CAPTURE &&
	    mission_item.nav_cmd != NAV_CMD_VIDEO_START_CAPTURE &&
	    mission_item.nav_cmd != NAV_CMD_VIDEO_STOP_CAPTURE &&
	    mission_item.nav_cmd != NAV_CMD_DO_CONTROL_VIDEO &&
	    mission_item.nav_cmd != NAV_CMD_DO_MOUNT_CONFIGURE &&
	    mission_item.nav_cmd != NAV_CMD_DO_MOUNT_CONTROL &&
	    mission_item.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW &&
	    mission_item.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_ACTUATOR &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_ROI &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_ROI_LOCATION &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_ROI_NONE &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_DIST &&
	    mission_item.nav_cmd != NAV_CMD_OBLIQUE_SURVEY &&
	    mission_item.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL &&
	    mission_item.nav_cmd != NAV_CMD_SET_CAMERA_MODE &&
	    mission_item.nav_cmd != NAV_CMD_SET_CAMERA_ZOOM &&
	    mission_item.nav_cmd != NAV_CMD_SET_CAMERA_FOCUS &&
	    mission_item.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION) {

		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: item %i: unsupported cmd: %d\t",
				     (int)(current_index + 1),
				     (int)mission_item.nav_cmd);
		events::send<uint16_t, uint16_t>(events::ID("navigator_mis_unsup_cmd"), {events::Log::Error, events::LogInternal::Warning},
						 "Mission rejected: item {1}: unsupported command: {2}", current_index + 1, mission_item.nav_cmd);
		return false;
	}

	// check if the mission starts with a land command while the vehicle is landed
	if ((current_index == 0) && mission_item.nav_cmd == NAV_CMD_LAND && _is_landed) {

		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: starts with landing\t");
		events::send(events::ID("navigator_mis_starts_w_landing"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: starts with landing");
		return false;
	}

	return true;
}

bool FeasibilityChecker::checkTakeoff(mission_item_s &mission_item)
{

	// look for a takeoff waypoint
	if (mission_item.nav_cmd == NAV_CMD_TAKEOFF || mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {
		// make sure that the altitude of the waypoint is above the home altitude
		const float takeoff_alt = mission_item.altitude_is_relative
					  ? mission_item.altitude
					  : mission_item.altitude - _home_alt_msl;

		if (takeoff_alt < FLT_EPSILON) {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Takeoff altitude below home altitude!\t");
			events::send<float>(events::ID("navigator_mis_takeoff_too_low"), {events::Log::Error, events::LogInternal::Info},
					    "Mission rejected: takeoff altitude too low! Minimum: {1:.1m_v}",
					    mission_item.altitude_is_relative ? 0.0f : _home_alt_msl);
			return false;
		}

		if (!_has_vtol_takeoff) {
			_has_vtol_takeoff = mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF;
		}

		if (!_has_takeoff) {
			_has_takeoff = true;
		}


		if (_found_item_with_position) {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: takeoff not first waypoint item\t");
			events::send(events::ID("navigator_mis_takeoff_not_first"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: takeoff is not the first waypoint item");
			return false;
		}
	}

	if (!_found_item_with_position) {
		_found_item_with_position = (mission_item.nav_cmd != NAV_CMD_IDLE &&
					     mission_item.nav_cmd != NAV_CMD_DELAY &&
					     mission_item.nav_cmd != NAV_CMD_DO_JUMP &&
					     mission_item.nav_cmd != NAV_CMD_DO_CHANGE_SPEED &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_HOME &&
					     mission_item.nav_cmd != NAV_CMD_DO_LAND_START &&
					     mission_item.nav_cmd != NAV_CMD_DO_TRIGGER_CONTROL &&
					     mission_item.nav_cmd != NAV_CMD_DO_DIGICAM_CONTROL &&
					     mission_item.nav_cmd != NAV_CMD_IMAGE_START_CAPTURE &&
					     mission_item.nav_cmd != NAV_CMD_IMAGE_STOP_CAPTURE &&
					     mission_item.nav_cmd != NAV_CMD_VIDEO_START_CAPTURE &&
					     mission_item.nav_cmd != NAV_CMD_VIDEO_STOP_CAPTURE &&
					     mission_item.nav_cmd != NAV_CMD_DO_CONTROL_VIDEO &&
					     mission_item.nav_cmd != NAV_CMD_DO_MOUNT_CONFIGURE &&
					     mission_item.nav_cmd != NAV_CMD_DO_MOUNT_CONTROL &&
					     mission_item.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW &&
					     mission_item.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_ROI &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_ROI_LOCATION &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_ROI_NONE &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_DIST &&
					     mission_item.nav_cmd != NAV_CMD_OBLIQUE_SURVEY &&
					     mission_item.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL &&
					     mission_item.nav_cmd != NAV_CMD_SET_CAMERA_MODE &&
					     mission_item.nav_cmd != NAV_CMD_SET_CAMERA_ZOOM &&
					     mission_item.nav_cmd != NAV_CMD_SET_CAMERA_FOCUS &&
					     mission_item.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION);
	}

	return true;
}

bool FeasibilityChecker::checkFixedWindLandApproach(mission_item_s &mission_item, const int current_index)
{
	if (mission_item.nav_cmd == NAV_CMD_LAND && current_index > 0) {

		if (MissionBlock::item_contains_position(_mission_item_previous)) {

			const float land_alt_amsl = mission_item.altitude_is_relative ? mission_item.altitude +
						    _home_alt_msl : mission_item.altitude;
			const float entrance_alt_amsl = _mission_item_previous.altitude_is_relative ? _mission_item_previous.altitude +
							_home_alt_msl : _mission_item_previous.altitude;
			const float relative_approach_altitude = entrance_alt_amsl - land_alt_amsl;

			if (relative_approach_altitude < FLT_EPSILON) {
				mavlink_log_critical(_mavlink_log_pub,
						     "Mission rejected: the approach waypoint must be above the landing point.\t");
				events::send(events::ID("navigator_mis_approach_wp_below_land"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: the approach waypoint must be above the landing point");
				return false;
			}

			float landing_approach_distance;

			if (_mission_item_previous.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
				// assume this is a fixed-wing landing pattern with orbit to alt followed
				// by tangent exit to landing approach and touchdown at landing waypoint

				const float distance_orbit_center_to_land = get_distance_to_next_waypoint(_mission_item_previous.lat,
						_mission_item_previous.lon, mission_item.lat, mission_item.lon);
				const float orbit_radius = fabsf(_mission_item_previous.loiter_radius);

				if (distance_orbit_center_to_land <= orbit_radius) {
					mavlink_log_critical(_mavlink_log_pub,
							     "Mission rejected: the landing point must be outside the orbit radius.\t");
					events::send(events::ID("navigator_mis_land_wp_inside_orbit_radius"), {events::Log::Error, events::LogInternal::Info},
						     "Mission rejected: the landing point must be outside the orbit radius");
					return false;
				}

				landing_approach_distance = sqrtf(distance_orbit_center_to_land * distance_orbit_center_to_land - orbit_radius *
								  orbit_radius);

			} else if (_mission_item_previous.nav_cmd == NAV_CMD_WAYPOINT) {
				// approaching directly from waypoint position

				const float waypoint_distance = get_distance_to_next_waypoint(_mission_item_previous.lat, _mission_item_previous.lon,
								mission_item.lat, mission_item.lon);
				landing_approach_distance = waypoint_distance;

			} else {
				mavlink_log_critical(_mavlink_log_pub,
						     "Mission rejected: unsupported landing approach entrance waypoint type. Only ORBIT_TO_ALT or WAYPOINT allowed.\t");
				events::send(events::ID("navigator_mis_unsupported_landing_approach_wp"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: unsupported landing approach entrance waypoint type. Only ORBIT_TO_ALT or WAYPOINT allowed");
				return false;
			}

			const float glide_slope = relative_approach_altitude / landing_approach_distance;

			// respect user setting as max glide slope, but account for floating point
			// rounding on next check with small (arbitrary) 0.1 deg buffer, as the
			// landing angle parameter is what is typically used for steepest glide
			// in landing config
			const float max_glide_slope = tanf(math::radians(_param_fw_lnd_ang + 0.1f));

			if (glide_slope > max_glide_slope) {

				const uint8_t land_angle_left_of_decimal = (uint8_t)_param_fw_lnd_ang;
				const uint8_t land_angle_first_after_decimal = (uint8_t)((_param_fw_lnd_ang - floorf(
							_param_fw_lnd_ang)) * 10.0f);

				mavlink_log_critical(_mavlink_log_pub,
						     "Mission rejected: the landing glide slope is steeper than the vehicle setting of %d.%d degrees.\t",
						     (int)land_angle_left_of_decimal, (int)land_angle_first_after_decimal);
				events::send<uint8_t, uint8_t>(events::ID("navigator_mis_glide_slope_too_steep"), {events::Log::Error, events::LogInternal::Info},
							       "Mission rejected: the landing glide slope is steeper than the vehicle setting of {1}.{2} degrees",
							       land_angle_left_of_decimal, land_angle_first_after_decimal);

				const uint32_t acceptable_entrance_alt = (uint32_t)(max_glide_slope * landing_approach_distance);
				const uint32_t acceptable_landing_dist = (uint32_t)ceilf(relative_approach_altitude / max_glide_slope);

				mavlink_log_critical(_mavlink_log_pub,
						     "Reduce the glide slope, lower the entrance altitude %d meters, or increase the landing approach distance %d meters.\t",
						     (int)acceptable_entrance_alt, (int)acceptable_landing_dist);
				events::send<uint32_t, uint32_t>(events::ID("navigator_mis_correct_glide_slope"), {events::Log::Error, events::LogInternal::Info},
								 "Reduce the glide slope, lower the entrance altitude {1} meters, or increase the landing approach distance {2} meters",
								 acceptable_entrance_alt, acceptable_landing_dist);

				return false;
			}

			_landing_valid = true;

		}
	}

	return true;
}

bool FeasibilityChecker::checkLandPatternValidity(mission_item_s &mission_item, const int current_index,
		const int last_index)
{

	// if DO_LAND_START found then require valid landing AFTER
	if (mission_item.nav_cmd == NAV_CMD_DO_LAND_START) {
		if (_do_land_start_index >= 0) {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: more than one land start.\t");
			events::send(events::ID("navigator_mis_multiple_land"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: more than one land start commands");
			return false;

		}

		_do_land_start_index = current_index;
	}

	const bool land_start_found = _do_land_start_index >= 0;

	if (mission_item.nav_cmd == NAV_CMD_LAND || mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {

		if (current_index > 0) {
			_landing_approach_index = current_index - 1;

		} else {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: starts with land waypoint.\t");
			events::send(events::ID("navigator_mis_starts_w_landing2"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: starts with landing");
			return false;
		}

	} else if (mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
		if (land_start_found && _do_land_start_index < current_index) {
			mavlink_log_critical(_mavlink_log_pub,
					     "Mission rejected: land start item before RTL item not possible.\t");
			events::send(events::ID("navigator_mis_land_before_rtl"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: land start item before RTL item is not possible");
			return false;
		}
	}

	if (current_index == last_index && land_start_found && (_do_land_start_index > _landing_approach_index)) {
		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: invalid land start.\t");
		events::send(events::ID("navigator_mis_invalid_land"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: invalid land start");
		return false;
	}

	_landing_valid = _landing_approach_index >= 0;

	/* No landing waypoints or no waypoints */
	return true;
}

bool FeasibilityChecker::checkTakeoffLandAvailable()
{
	bool result = true;

	switch (_param_mis_takeoff_land_req) {
	case 0:
		result = true;
		break;

	case 1:
		result = _has_takeoff;

		if (!result) {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Takeoff waypoint required.\t");
			events::send(events::ID("navigator_mis_takeoff_missing"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: Takeoff waypoint required");
			return false;
		}

		break;

	case 2:
		result = _landing_valid;

		if (!result) {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Landing waypoint/pattern required.\t");
			events::send(events::ID("navigator_mis_land_missing"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: Landing waypoint/pattern required");
		}

		break;

	case 3:
		result = _has_takeoff && _landing_valid;

		if (!result) {
			mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Takeoff or Landing item missing.\t");
			events::send(events::ID("navigator_mis_takeoff_or_land_missing"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: Takeoff or Landing item missing");
		}

		break;

	case 4:
		result = hasMissionBothOrNeitherTakeoffAndLanding();

		break;

	case 5:
		if (!_is_landed && !_has_vtol_approach) {
			result = _landing_valid;

			if (!result) {
				mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Landing waypoint/pattern required.");
				events::send(events::ID("feasibility_mis_in_air_landing_req"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: Landing waypoint/pattern required");
			}

		} else {
			result = hasMissionBothOrNeitherTakeoffAndLanding();
		}

		break;

	default:
		result = true;
		break;
	}

	return result;
}

bool FeasibilityChecker::hasMissionBothOrNeitherTakeoffAndLanding()
{
	bool result{_has_takeoff == _landing_valid};

	if (!result && (_has_takeoff)) {
		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Add Landing item or remove Takeoff.\t");
		events::send(events::ID("navigator_mis_add_land_or_rm_to"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: Add Landing item or remove Takeoff");

	} else if (!result && (_landing_valid)) {
		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Add Takeoff item or remove Landing.\t");
		events::send(events::ID("navigator_mis_add_to_or_rm_land"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: Add Takeoff item or remove Landing");
	}

	return result;
}

bool FeasibilityChecker::checkHorizontalDistanceToFirstWaypoint(mission_item_s &mission_item)
{
	if (_param_mis_dist_1wp > FLT_EPSILON &&
	    (_current_position_lat_lon.isAllFinite()) && !_first_waypoint_found &&
	    MissionBlock::item_contains_position(mission_item)) {

		_first_waypoint_found = true;

		float dist_to_1wp_from_current_pos = 1e6f;

		if (_current_position_lat_lon.isAllFinite()) {
			dist_to_1wp_from_current_pos = get_distance_to_next_waypoint(
							       mission_item.lat, mission_item.lon,
							       _current_position_lat_lon(0), _current_position_lat_lon(1));
		}

		if (dist_to_1wp_from_current_pos < _param_mis_dist_1wp) {

			return true;

		} else {
			/* item is too far from current position */
			mavlink_log_critical(_mavlink_log_pub,
					     "First waypoint too far away: %dm, %d max\t",
					     (int)dist_to_1wp_from_current_pos, (int)_param_mis_dist_1wp);
			events::send<uint32_t, uint32_t>(events::ID("navigator_mis_first_wp_too_far"), {events::Log::Error, events::LogInternal::Info},
							 "First waypoint too far away: {1m} (maximum: {2m})", (uint32_t)dist_to_1wp_from_current_pos,
							 (uint32_t)_param_mis_dist_1wp);

			return false;
		}
	}

	return true;
}

bool FeasibilityChecker::checkDistancesBetweenWaypoints(const mission_item_s &mission_item)
{
	/* check only items with valid lat/lon */
	if (!MissionBlock::item_contains_position(mission_item)) {
		return true;
	}

	/* Compare it to last waypoint if already available. */
	if (PX4_ISFINITE(_last_lat) && PX4_ISFINITE(_last_lon)) {
		/* check distance from current position to item */
		const float dist_between_waypoints = get_distance_to_next_waypoint(
				mission_item.lat, mission_item.lon,
				_last_lat, _last_lon);


		if (dist_between_waypoints < 0.05f &&
		    (mission_item.nav_cmd == NAV_CMD_CONDITION_GATE || _last_cmd == NAV_CMD_CONDITION_GATE)) {

			/* Waypoints and gate are at the exact same position, which indicates an
			 * invalid mission and makes calculating the direction from one waypoint
			 * to another impossible. */
			mavlink_log_critical(_mavlink_log_pub,
					     "Distance between waypoint and gate too close: %d meters\t",
					     (int)dist_between_waypoints);
			events::send<float, float>(events::ID("navigator_mis_wp_gate_too_close"), {events::Log::Error, events::LogInternal::Info},
						   "Distance between waypoint and gate too close: {1:.3m} (minimum: {2:.3m})", dist_between_waypoints, 0.05f);


			return false;
		}
	}

	_last_lat = mission_item.lat;
	_last_lon = mission_item.lon;
	_last_cmd = mission_item.nav_cmd;

	/* We ran through all waypoints and have not found any distances between waypoints that are too far. */
	return true;
}

bool FeasibilityChecker::checkItemsFitToVehicleType(const mission_item_s &mission_item)
{
	if (_vehicle_type != VehicleType::Vtol &&
	    (mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF || mission_item.nav_cmd == NAV_CMD_VTOL_LAND
	     || mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION)) {

		mavlink_log_critical(_mavlink_log_pub, "Mission rejected: Mission contains VTOL items but vehicle is not a VTOL\t");
		events::send(events::ID("navigator_mis_vtol_items"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: Mission contains VTOL items but vehicle is not a VTOL");

		return false;
	}

	return true;
}
