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

#include <float.h>

#include "rtl_direct.h"
#include "navigator.h"
#include <px4_platform_common/events.h>

#include <lib/geo/geo.h>

using namespace math;

RtlDirect::RtlDirect(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_destination.lat = static_cast<double>(NAN);
	_destination.lon = static_cast<double>(NAN);
	_land_approach.lat = static_cast<double>(NAN);
	_land_approach.lon = static_cast<double>(NAN);
	_land_approach.height_m = NAN;

	_param_mpc_z_v_auto_up = param_find("MPC_Z_V_AUTO_UP");
	_param_mpc_z_v_auto_dn = param_find("MPC_Z_V_AUTO_DN");
	_param_mpc_land_speed = param_find("MPC_LAND_SPEED");
	_param_fw_climb_rate = param_find("FW_T_CLMB_R_SP");
	_param_fw_sink_rate = param_find("FW_T_SINK_R_SP");
	_param_fw_airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_param_mpc_xy_cruise = param_find("MPC_XY_CRUISE");
	_param_rover_cruise_speed = param_find("GND_SPEED_THR_SC");
}

void RtlDirect::on_inactivation()
{
	if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}

	_rtl_state = RTLState::IDLE;
}

void RtlDirect::on_activation()
{
	_global_pos_sub.update();
	_land_detected_sub.update();
	_vehicle_status_sub.update();

	parameters_update();

	_rtl_state = getActivationLandState();

	// reset cruising speed and throttle to default for RTL
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

	set_rtl_item();

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: start return at %d m (%d m above destination)\t",
			 (int)ceilf(_rtl_alt), (int)ceilf(_rtl_alt - _destination.alt));
	events::send<int32_t, int32_t>(events::ID("vrtl_return_at"), events::Log::Info,
				       "RTL: start return at {1m_v} ({2m_v} above destination)",
				       (int32_t)ceilf(_rtl_alt), (int32_t)ceilf(_rtl_alt - _destination.alt));
}

void RtlDirect::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	parameters_update();

	if (_rtl_state != RTLState::IDLE && is_mission_item_reached_or_completed()) {
		set_rtl_item();
	}

	if (_rtl_state == RTLState::LAND && _param_rtl_pld_md.get() > 0) {
		// Need to update the position and type on the current setpoint triplet.
		_navigator->get_precland()->on_active();

	} else if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}
}

void RtlDirect::setRtlPosition(PositionYawSetpoint rtl_position, loiter_point_s loiter_pos)
{
	_home_pos_sub.update();

	parameters_update();

	// Only allow to set a new approach if the mode is not activated yet.
	if (!isActive()) {
		_land_approach = loiter_pos;
		_destination = rtl_position;
		_force_heading = false;

		// Input sanitation
		if (!PX4_ISFINITE(_destination.lat) || !PX4_ISFINITE(_destination.lon)) {
			// We don't have a valid rtl position, use the home position instead.
			_destination.lat = _home_pos_sub.get().lat;
			_destination.lon = _home_pos_sub.get().lon;
			_destination.alt = _home_pos_sub.get().alt;
			_destination.yaw = _home_pos_sub.get().yaw;
		}

		if (!PX4_ISFINITE(_destination.alt)) {
			// Not a valid rtl land altitude. Assume same altitude as home position.
			_destination.alt = _home_pos_sub.get().alt;
		}

		if (!PX4_ISFINITE(_land_approach.lat) || !PX4_ISFINITE(_land_approach.lon)) {
			_land_approach.lat = _destination.lat;
			_land_approach.lon = _destination.lon;

		} else {
			const float dist_to_destination{get_distance_to_next_waypoint(_land_approach.lat, _land_approach.lon, _destination.lat, _destination.lon)};

			if (dist_to_destination > _navigator->get_acceptance_radius()) {
				_force_heading = true;
			}
		}

		if (!PX4_ISFINITE(_land_approach.height_m)) {
			_land_approach.height_m = _destination.alt + _param_rtl_descend_alt.get();
		}

		if (!PX4_ISFINITE(_land_approach.loiter_radius_m) || fabsf(_land_approach.loiter_radius_m) <= FLT_EPSILON) {
			_land_approach.loiter_radius_m = _param_rtl_loiter_rad.get();
		}
	}
}

void RtlDirect::set_rtl_item()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
				       _global_pos_sub.get().lat, _global_pos_sub.get().lon);
	const float loiter_altitude = math::min(_land_approach.height_m, _rtl_alt);

	const bool is_close_to_destination = destination_dist < _param_rtl_min_dist.get();

	switch (_rtl_state) {
	case RTLState::CLIMBING: {
			PositionYawSetpoint pos_yaw_sp {
				.lat = _global_pos_sub.get().lat,
				.lon = _global_pos_sub.get().lon,
				.alt = _rtl_alt,
				.yaw = _param_wv_en.get() ? NAN : _navigator->get_local_position()->heading,
			};
			setLoiterToAltMissionItem(_mission_item, pos_yaw_sp, _navigator->get_loiter_radius());

			_rtl_state = RTLState::MOVE_TO_LOITER;
			break;
		}

	case RTLState::MOVE_TO_LOITER: {
			PositionYawSetpoint pos_yaw_sp {
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = _rtl_alt,
			};

			// For FW flight:set to LOITER_TIME (with 0s loiter time), such that the loiter (orbit) status
			// can be displayed on groundstation and the WP is accepted once within loiter radius
			if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				pos_yaw_sp.yaw = NAN;
				setLoiterHoldMissionItem(_mission_item, pos_yaw_sp, 0.f, _land_approach.loiter_radius_m);

			} else {
				// already set final yaw if close to destination and weather vane is disabled
				pos_yaw_sp.yaw = (is_close_to_destination && !_param_wv_en.get()) ? _destination.yaw : NAN;
				setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);
			}

			_rtl_state = RTLState::LOITER_DOWN;

			break;
		}

	case RTLState::LOITER_DOWN: {
			PositionYawSetpoint pos_yaw_sp{
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = loiter_altitude,
				.yaw = !_param_wv_en.get() ? _destination.yaw : NAN, // set final yaw if weather vane is disabled
			};

			setLoiterToAltMissionItem(_mission_item, pos_yaw_sp, _land_approach.loiter_radius_m);

			pos_sp_triplet->next.valid = true;
			pos_sp_triplet->next.lat = _destination.lat;
			pos_sp_triplet->next.lon = _destination.lon;
			pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_LAND;

			if (_force_heading) {
				_mission_item.force_heading = true;
			}

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			_rtl_state = RTLState::LOITER_HOLD;

			break;
		}

	case RTLState::LOITER_HOLD: {
			PositionYawSetpoint pos_yaw_sp {
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = loiter_altitude,
				.yaw = !_param_wv_en.get() ? _destination.yaw : NAN, // set final yaw if weather vane is disabled
			};

			setLoiterHoldMissionItem(_mission_item, pos_yaw_sp, _param_rtl_land_delay.get(), _land_approach.loiter_radius_m);

			if (_param_rtl_land_delay.get() < -FLT_EPSILON) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering\t");
				events::send(events::ID("rtl_completed_loiter"), events::Log::Info, "RTL: completed, loitering");
			}

			if (_vehicle_status_sub.get().is_vtol
			    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				_rtl_state = RTLState::MOVE_TO_LAND;

			} else {
				_rtl_state = RTLState::MOVE_TO_LAND_HOVER;
			}

			break;
		}

	case RTLState::MOVE_TO_LAND: {

			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.alt = loiter_altitude;
			pos_yaw_sp.yaw = NAN;

			setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);

			// Prepare for transition
			_mission_item.vtol_back_transition = true;
			_mission_item.force_heading = false;

			// set previous item location to loiter location such that vehicle tracks line between loiter
			// location and land location after exiting the loiter circle
			pos_sp_triplet->previous.lat = _land_approach.lat;
			pos_sp_triplet->previous.lon = _land_approach.lon;
			pos_sp_triplet->previous.alt = _mission_item.altitude;
			pos_sp_triplet->previous.valid = true;

			_rtl_state = RTLState::TRANSITION_TO_MC;

			break;
		}

	case RTLState::TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

			_rtl_state = RTLState::MOVE_TO_LAND_HOVER;

			break;
		}

	case RTLState::MOVE_TO_LAND_HOVER: {
			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.alt = loiter_altitude;
			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled

			setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);
			_navigator->reset_position_setpoint(pos_sp_triplet->previous);

			_rtl_state = RTLState::LAND;

			break;
		}

	case RTLState::LAND: {
			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled
			setLandMissionItem(_mission_item, pos_yaw_sp);

			_mission_item.land_precision = _param_rtl_pld_md.get();

			startPrecLand(_mission_item.land_precision);

			_rtl_state = RTLState::IDLE;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at destination\t");
			events::send(events::ID("rtl_land_at_destination"), events::Log::Info, "RTL: land at destination");
			break;
		}

	case RTLState::IDLE: {
			set_idle_item(&_mission_item);
			_navigator->mode_completed(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!MissionBlock::item_contains_position(_mission_item)) {
		issue_command(_mission_item);

	} else {
		// Convert mission item to current position setpoint and make it valid.
		if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
			_navigator->set_position_setpoint_triplet_updated();
		}
	}
}

RtlDirect::RTLState RtlDirect::getActivationLandState()
{
	_land_detected_sub.update();

	RTLState land_state;

	if (_land_detected_sub.get().landed) {
		// For safety reasons don't go into RTL if landed.
		land_state = RTLState::IDLE;

	} else if ((_global_pos_sub.get().alt < _rtl_alt) || _enforce_rtl_alt) {
		land_state = RTLState::CLIMBING;

	} else {
		land_state = RTLState::MOVE_TO_LOITER;
	}

	return land_state;
}

rtl_time_estimate_s RtlDirect::calc_rtl_time_estimate()
{
	_global_pos_sub.update();

	rtl_time_estimate_s rtl_time_estimate{};

	RTLState start_state_for_estimate;

	if (isActive()) {
		start_state_for_estimate = _rtl_state;

	} else {
		start_state_for_estimate = getActivationLandState();
	}

	// Calculate RTL time estimate only when there is a valid home position
	// TODO: Also check if vehicle position is valid
	if (!_navigator->home_global_position_valid()) {
		rtl_time_estimate.valid = false;

	} else {
		rtl_time_estimate.valid = true;
		rtl_time_estimate.time_estimate = 0.f;

		const float loiter_altitude = min(_land_approach.height_m, _rtl_alt);

		// Sum up time estimate for various segments of the landing procedure
		switch (start_state_for_estimate) {
		case RTLState::CLIMBING: {
				// Climb segment is only relevant if the drone is below return altitude
				const float climb_dist = _global_pos_sub.get().alt < _rtl_alt ? (_rtl_alt - _global_pos_sub.get().alt) : 0;

				if (climb_dist > FLT_EPSILON) {
					rtl_time_estimate.time_estimate += climb_dist / getClimbRate();
				}
			}

		// FALLTHROUGH
		case RTLState::MOVE_TO_LOITER:

			// Add cruise segment to home
			rtl_time_estimate.time_estimate += get_distance_to_next_waypoint(
					_land_approach.lat, _land_approach.lon, _global_pos_sub.get().lat, _global_pos_sub.get().lon) / getCruiseGroundSpeed();

		// FALLTHROUGH
		case RTLState::LOITER_DOWN: {
				// when descending, the target altitude is stored in the current mission item
				float initial_altitude = 0.f;

				if (start_state_for_estimate == RTLState::LOITER_DOWN) {
					// Take current vehicle altitude as the starting point for calculation
					initial_altitude = _global_pos_sub.get().alt;  // TODO: Check if this is in the right frame

				} else {
					// Take the return altitude as the starting point for the calculation
					initial_altitude = _rtl_alt; // CLIMB and RETURN
				}

				// Add descend segment (first landing phase: return alt to loiter alt)
				rtl_time_estimate.time_estimate += fabsf(initial_altitude - loiter_altitude) / getDescendRate();
			}

		// FALLTHROUGH
		case RTLState::LOITER_HOLD:
			// Add land delay (the short pause for deploying landing gear)
			// TODO: Check if landing gear is deployed or not
			rtl_time_estimate.time_estimate += _param_rtl_land_delay.get();

		// FALLTHROUGH
		case RTLState::MOVE_TO_LAND:
		case RTLState::TRANSITION_TO_MC:
		case RTLState::MOVE_TO_LAND_HOVER: {
				// Add cruise segment to home
				float move_to_land_dist{0.f};

				if (start_state_for_estimate >= RTLState::MOVE_TO_LAND) {
					move_to_land_dist = get_distance_to_next_waypoint(
								    _destination.lat, _destination.lon, _global_pos_sub.get().lat, _global_pos_sub.get().lon);

				} else {
					move_to_land_dist = get_distance_to_next_waypoint(
								    _destination.lat, _destination.lon, _land_approach.lat, _land_approach.lon);
				}

				if (move_to_land_dist > FLT_EPSILON) {
					rtl_time_estimate.time_estimate += move_to_land_dist / getCruiseGroundSpeed();
				}
			}

		// FALLTHROUGH
		case RTLState::LAND: {
				float initial_altitude;

				// Add land segment (second landing phase) which comes after LOITER
				if (start_state_for_estimate == RTLState::LAND) {
					// If we are in this phase, use the current vehicle altitude  instead
					// of the altitude paramteter to get a continous time estimate
					initial_altitude = _global_pos_sub.get().alt;


				} else {
					// If this phase is not active yet, simply use the loiter altitude,
					// which is where the LAND phase will start
					initial_altitude = loiter_altitude;
				}

				// Prevent negative times when close to the ground
				if (initial_altitude > _destination.alt) {
					rtl_time_estimate.time_estimate += (initial_altitude - _destination.alt) / getHoverLandSpeed();
				}
			}

			break;

		case RTLState::IDLE:
			// Remaining time is 0
			break;
		}

		// Prevent negative durations as phyiscally they make no sense. These can
		// occur during the last phase of landing when close to the ground.
		rtl_time_estimate.time_estimate = math::max(0.f, rtl_time_estimate.time_estimate);

		// Use actual time estimate to compute the safer time estimate with additional scale factor and a margin
		rtl_time_estimate.safe_time_estimate = _param_rtl_time_factor.get() * rtl_time_estimate.time_estimate
						       + _param_rtl_time_margin.get();
	}

	// return message
	rtl_time_estimate.timestamp = hrt_absolute_time();

	return rtl_time_estimate;
}

float RtlDirect::getCruiseSpeed()
{
	float ret = 1e6f;

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_xy_cruise == PARAM_INVALID || param_get(_param_mpc_xy_cruise, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (_param_fw_airspeed_trim == PARAM_INVALID || param_get(_param_fw_airspeed_trim, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		if (_param_rover_cruise_speed == PARAM_INVALID || param_get(_param_rover_cruise_speed, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

float RtlDirect::getHoverLandSpeed()
{
	float ret = 1e6f;

	if (_param_mpc_land_speed == PARAM_INVALID || param_get(_param_mpc_land_speed, &ret) != PX4_OK) {
		ret = 1e6f;
	}

	return ret;
}

matrix::Vector2f RtlDirect::get_wind()
{
	_wind_sub.update();
	matrix::Vector2f wind;

	if (hrt_absolute_time() - _wind_sub.get().timestamp < 1_s) {
		wind(0) = _wind_sub.get().windspeed_north;
		wind(1) = _wind_sub.get().windspeed_east;
	}

	return wind;
}

float RtlDirect::getClimbRate()
{
	float ret = 1e6f;

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_z_v_auto_up == PARAM_INVALID || param_get(_param_mpc_z_v_auto_up, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		if (_param_fw_climb_rate == PARAM_INVALID || param_get(_param_fw_climb_rate, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

float RtlDirect::getDescendRate()
{
	float ret = 1e6f;

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_z_v_auto_dn == PARAM_INVALID || param_get(_param_mpc_z_v_auto_dn, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (_param_fw_sink_rate == PARAM_INVALID || param_get(_param_fw_sink_rate, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

float RtlDirect::getCruiseGroundSpeed()
{
	float cruise_speed = getCruiseSpeed();

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		const vehicle_global_position_s &global_position = *_navigator->get_global_position();
		matrix::Vector2f wind = get_wind();

		matrix::Vector2f to_destination_vec;
		get_vector_to_next_waypoint(global_position.lat, global_position.lon, _destination.lat, _destination.lon,
					    &to_destination_vec(0), &to_destination_vec(1));

		const matrix::Vector2f to_home_dir = to_destination_vec.unit_or_zero();

		const float wind_towards_home = wind.dot(to_home_dir);
		const float wind_across_home = matrix::Vector2f(wind - to_home_dir * wind_towards_home).norm();


		// Note: use fminf so that we don't _rely_ on wind towards home to make RTL more efficient
		const float ground_speed = sqrtf(cruise_speed * cruise_speed - wind_across_home * wind_across_home) + fminf(
						   0.f, wind_towards_home);

		cruise_speed = ground_speed;
	}

	return cruise_speed;
}

void RtlDirect::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}
