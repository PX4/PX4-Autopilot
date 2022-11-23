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

#include "rtl_direct.h"
#include "navigator.h"
#include <dataman/dataman.h>
#include <px4_platform_common/events.h>

#include <lib/geo/geo.h>


static constexpr float DELAY_SIGMA = 0.01f;

using namespace time_literals;
using namespace math;

RtlDirect::RtlDirect(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_param_mpc_z_v_auto_up = param_find("MPC_Z_V_AUTO_UP");
	_param_mpc_z_v_auto_dn = param_find("MPC_Z_V_AUTO_DN");
	_param_mpc_land_speed = param_find("MPC_LAND_SPEED");
	_param_fw_climb_rate = param_find("FW_T_CLMB_R_SP");
	_param_fw_sink_rate = param_find("FW_T_SINK_R_SP");
	_param_fw_airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_param_mpc_xy_cruise = param_find("MPC_XY_CRUISE");
	_param_rover_cruise_speed = param_find("GND_SPEED_THR_SC");
}

void RtlDirect::on_activation(bool enforce_rtl_alt)
{
	_global_pos_sub.update();
	_local_pos_sub.update();
	_land_detected_sub.update();
	_vehicle_status_sub.update();
	_wind_sub.update();

	if (_land_detected_sub.get().landed) {
		// For safety reasons don't go into RTL if landed.
		_rtl_state = RTL_STATE_LANDED;

	} else if ((_global_pos_sub.get().alt < _destination.alt + _param_rtl_return_alt.get()) || enforce_rtl_alt) {

		// If lower than return altitude, climb up first.
		// If rtl_alt_min is true then forcing altitude change even if above.
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		// Otherwise go start with climb
		_rtl_state = RTL_STATE_RETURN;
	}

	// reset cruising speed and throttle to default for RTL
	_navigator->set_cruising_speed();
	_navigator->set_cruising_throttle();

	set_rtl_item();

	MissionBlock::on_active();
}

void RtlDirect::on_active()
{
	_global_pos_sub.update();
	_local_pos_sub.update();
	_land_detected_sub.update();
	_vehicle_status_sub.update();
	_wind_sub.update();

	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached_or_completed()) {
		advance_rtl();
		set_rtl_item();
	}

	if (_rtl_state == RTL_STATE_LAND && _param_rtl_pld_md.get() > 0) {
		// Need to update the position and type on the current setpoint triplet.
		_navigator->get_precland()->on_active();
	}
}

void RtlDirect::set_rtl_item()
{
	_navigator->set_can_loiter_at_sp(false);

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
				       _global_pos_sub.get().lat, _global_pos_sub.get().lon);
	const float loiter_altitude = math::min(_destination.alt + _param_rtl_descend_alt.get(), _rtl_alt);

	const RTLHeadingMode rtl_heading_mode = static_cast<RTLHeadingMode>(_param_rtl_hdg_md.get());

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

			// do not use LOITER_TO_ALT for rotary wing mode as it would then always climb to at least MIS_LTRMIN_ALT,
			// even if current climb altitude is below (e.g. RTL immediately after take off)
			if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
			}

			_mission_item.lat = _global_pos_sub.get().lat;
			_mission_item.lon = _global_pos_sub.get().lon;
			_mission_item.altitude = _rtl_alt;
			_mission_item.altitude_is_relative = false;

			if (rtl_heading_mode != RTLHeadingMode::RTL_DESTINATION_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above destination)\t",
					 (int)ceilf(_rtl_alt), (int)ceilf(_rtl_alt - _destination.alt));
			events::send<int32_t, int32_t>(events::ID("rtl_climb_to"), events::Log::Info,
						       "RTL: climb to {1m_v} ({2m_v} above destination)",
						       (int32_t)ceilf(_rtl_alt), (int32_t)ceilf(_rtl_alt - _destination.alt));
			break;
		}

	case RTL_STATE_RETURN: {

			// For FW flight:set to LOITER_TIME (with 0s loiter time), such that the loiter (orbit) status
			// can be displayed on groundstation and the WP is accepted once within loiter radius
			if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;


			} else {
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			}

			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = _rtl_alt; // Don't change altitude
			_mission_item.altitude_is_relative = false;

			if (rtl_heading_mode == RTLHeadingMode::RTL_NAVIGATION_HEADING &&
			    destination_dist > _param_rtl_min_dist.get()) {
				_mission_item.yaw = get_bearing_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, _destination.lat,
						    _destination.lon);

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_DESTINATION_HEADING ||
				   destination_dist < _param_rtl_min_dist.get()) {
				// Use destination yaw if close to _destination.
				_mission_item.yaw = _destination.yaw;

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_CURRENT_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.loiter_radius = _param_rtl_loiter_rad.get();

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above destination)\t",
					 (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));
			events::send<int32_t, int32_t>(events::ID("rtl_return_at"), events::Log::Info,
						       "RTL: return at {1m_v} ({2m_v} above destination)",
						       (int32_t)ceilf(_mission_item.altitude), (int32_t)ceilf(_mission_item.altitude - _destination.alt));

			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			// Except for vtol which might be still off here and should point towards this location.
			const float d_current = get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
						_mission_item.lat, _mission_item.lon);

			if (_vehicle_status_sub.get().is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
						    _mission_item.lat, _mission_item.lon);

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_CURRENT_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.loiter_radius = _param_rtl_loiter_rad.get();

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above destination)\t",
					 (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));
			events::send<int32_t, int32_t>(events::ID("rtl_descend_to"), events::Log::Info,
						       "RTL: descend to {1m_v} ({2m_v} above destination)",
						       (int32_t)ceilf(_mission_item.altitude), (int32_t)ceilf(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			const bool autocontinue = (_param_rtl_land_delay.get() > FLT_EPSILON);

			if (autocontinue) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs\t",
						 (double)_param_rtl_land_delay.get());
				events::send<float>(events::ID("rtl_loiter"), events::Log::Info, "RTL: loiter {1:.1}s", _param_rtl_land_delay.get());

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering\t");
				events::send(events::ID("rtl_completed_loiter"), events::Log::Info, "RTL: completed, loitering");
			}

			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;    // Don't change altitude.
			_mission_item.altitude_is_relative = false;

			if (rtl_heading_mode == RTLHeadingMode::RTL_CURRENT_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = max(_param_rtl_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autocontinue;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.loiter_radius = _param_rtl_loiter_rad.get();

			_navigator->set_can_loiter_at_sp(true);

			break;
		}

	case RTL_STATE_HEAD_TO_CENTER: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			if (rtl_heading_mode == RTLHeadingMode::RTL_NAVIGATION_HEADING) {
				_mission_item.yaw = get_bearing_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, _destination.lat,
						    _destination.lon);

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_DESTINATION_HEADING) {
				_mission_item.yaw = _destination.yaw;

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_CURRENT_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;
			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case RTL_MOVE_TO_LAND_HOVER_VTOL: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			if (rtl_heading_mode == RTLHeadingMode::RTL_NAVIGATION_HEADING) {
				_mission_item.yaw = get_bearing_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, _destination.lat,
						    _destination.lon);

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_DESTINATION_HEADING) {
				_mission_item.yaw = _destination.yaw;

			} else if (rtl_heading_mode == RTLHeadingMode::RTL_CURRENT_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.origin = ORIGIN_ONBOARD;
			break;
		}

	case RTL_STATE_LAND: {
			// Land at destination.
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = _destination.alt;
			_mission_item.altitude_is_relative = false;

			if (rtl_heading_mode == RTLHeadingMode::RTL_CURRENT_HEADING) {
				_mission_item.yaw = _local_pos_sub.get().heading;

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.land_precision = _param_rtl_pld_md.get();

			if (_mission_item.land_precision > 0u && _mission_item.land_precision <= 2u) {
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

				if (_mission_item.land_precision == 1) {
					_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

				} else if (_mission_item.land_precision == 2) {
					_navigator->get_precland()->set_mode(PrecLandMode::Required);
				}

				_navigator->get_precland()->on_activation();
			}

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at destination\t");
			events::send(events::ID("rtl_land_at_destination"), events::Log::Info, "RTL: land at destination");
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!MissionBlock::item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// Convert mission item to current position setpoint and make it valid.
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void RtlDirect::advance_rtl()
{
	// determines if the vehicle should loiter above land
	const bool descend_and_loiter = _param_rtl_land_delay.get() < -DELAY_SIGMA || _param_rtl_land_delay.get() > DELAY_SIGMA;

	// vehicle is a vtol and currently in fixed wing mode
	const bool vtol_in_fw_mode = _vehicle_status_sub.get().is_vtol
				     && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		if (vtol_in_fw_mode || descend_and_loiter) {
			_rtl_state = RTL_STATE_DESCEND;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_DESCEND:

		if (descend_and_loiter) {
			_rtl_state = RTL_STATE_LOITER;

		} else if (vtol_in_fw_mode) {
			_rtl_state = RTL_STATE_HEAD_TO_CENTER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:

		if (vtol_in_fw_mode) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_HEAD_TO_CENTER:

		_rtl_state = RTL_STATE_TRANSITION_TO_MC;

		break;

	case RTL_STATE_TRANSITION_TO_MC:

		_rtl_state = RTL_MOVE_TO_LAND_HOVER_VTOL;

		break;

	case RTL_MOVE_TO_LAND_HOVER_VTOL:

		_rtl_state = RTL_STATE_LAND;

		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}

rtl_time_estimate_s RtlDirect::calc_rtl_time_estimate()
{
	rtl_time_estimate_s rtl_time_estimate{};

	RTLState start_state_for_estimate{RTL_STATE_NONE};

	if (isActive()) {
		start_state_for_estimate = _rtl_state;
	}

	// Calculate RTL time estimate only when there is a valid home position
	// TODO: Also check if vehicle position is valid
	if (!_navigator->home_global_position_valid()) {
		rtl_time_estimate.valid = false;

	} else {
		rtl_time_estimate.valid = true;

		// Sum up time estimate for various segments of the landing procedure
		switch (start_state_for_estimate) {
		case RTL_STATE_NONE:
		case RTL_STATE_CLIMB: {
				// Climb segment is only relevant if the drone is below return altitude
				const float climb_dist = _global_pos_sub.get().alt < _rtl_alt ? (_rtl_alt - _global_pos_sub.get().alt) : 0;

				if (climb_dist > 0) {
					rtl_time_estimate.time_estimate += climb_dist / getClimbRate();
				}
			}

		// FALLTHROUGH
		case RTL_STATE_RETURN:

			// Add cruise segment to home
			rtl_time_estimate.time_estimate += get_distance_to_next_waypoint(
					_destination.lat, _destination.lon, _global_pos_sub.get().lat, _global_pos_sub.get().lon) / getCruiseGroundSpeed();

		// FALLTHROUGH
		case RTL_STATE_HEAD_TO_CENTER:
		case RTL_STATE_TRANSITION_TO_MC:
		case RTL_STATE_DESCEND: {
				// when descending, the target altitude is stored in the current mission item
				float initial_altitude = 0;
				float loiter_altitude = 0;

				if (start_state_for_estimate == RTL_STATE_DESCEND) {
					// Take current vehicle altitude as the starting point for calculation
					initial_altitude = _global_pos_sub.get().alt;  // TODO: Check if this is in the right frame
					loiter_altitude = _mission_item.altitude;  // Next waypoint = loiter


				} else {
					// Take the return altitude as the starting point for the calculation
					initial_altitude = _rtl_alt; // CLIMB and RETURN
					loiter_altitude = math::min(_destination.alt + _param_rtl_descend_alt.get(), _rtl_alt);
				}

				// Add descend segment (first landing phase: return alt to loiter alt)
				rtl_time_estimate.time_estimate += fabsf(initial_altitude - loiter_altitude) / getDescendRate();
			}

		// FALLTHROUGH
		case RTL_STATE_LOITER:
			// Add land delay (the short pause for deploying landing gear)
			// TODO: Check if landing gear is deployed or not
			rtl_time_estimate.time_estimate += _param_rtl_land_delay.get();

		// FALLTHROUGH
		case RTL_MOVE_TO_LAND_HOVER_VTOL:
		case RTL_STATE_LAND: {
				float initial_altitude;

				// Add land segment (second landing phase) which comes after LOITER
				if (start_state_for_estimate == RTL_STATE_LAND) {
					// If we are in this phase, use the current vehicle altitude  instead
					// of the altitude paramteter to get a continous time estimate
					initial_altitude = _global_pos_sub.get().alt;


				} else {
					// If this phase is not active yet, simply use the loiter altitude,
					// which is where the LAND phase will start
					const float loiter_altitude = math::min(_destination.alt + _param_rtl_descend_alt.get(), _rtl_alt);
					initial_altitude = loiter_altitude;
				}

				// Prevent negative times when close to the ground
				if (initial_altitude > _destination.alt) {
					rtl_time_estimate.time_estimate += (initial_altitude - _destination.alt) / getHoverLandSpeed();
				}

			}

			break;

		case RTL_STATE_LANDED:
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

	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_xy_cruise == PARAM_INVALID || param_get(_param_mpc_xy_cruise, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (_param_fw_airspeed_trim == PARAM_INVALID || param_get(_param_fw_airspeed_trim, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
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

	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_z_v_auto_up == PARAM_INVALID || param_get(_param_mpc_z_v_auto_up, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		if (_param_fw_climb_rate == PARAM_INVALID || param_get(_param_fw_climb_rate, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

float RtlDirect::getDescendRate()
{
	float ret = 1e6f;

	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_z_v_auto_dn == PARAM_INVALID || param_get(_param_mpc_z_v_auto_dn, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (_param_fw_sink_rate == PARAM_INVALID || param_get(_param_fw_sink_rate, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

float RtlDirect::getCruiseGroundSpeed()
{
	float cruise_speed = getCruiseSpeed();

	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		matrix::Vector2f wind = get_wind();

		matrix::Vector2f to_destination_vec;
		get_vector_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, _destination.lat, _destination.lon,
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
