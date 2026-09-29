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
 * @file goto_mode.cpp
 */

#include "goto_mode.h"
#include "navigator.h"

#include <drivers/drv_hrt.h>

using namespace time_literals;

Goto::Goto(Navigator *navigator) :
	NavigatorMode(navigator, vehicle_status_s::NAVIGATION_STATE_GOTO)
{
	_param_handle_mpc_yaw_mode = param_find("MPC_YAW_MODE");
}

void
Goto::on_activation()
{
	// Goto flies via goto_setpoint only. Keep the triplet invalid, so the next mode (e.g. Hold on a pause)
	// starts from the current position instead of whatever the previous mode left there.
	_navigator->reset_triplets();
	_navigator->reset_cruising_speed();
	_target_valid = false;
	_locked_heading = NAN;

	if (_param_handle_mpc_yaw_mode != PARAM_INVALID) {
		param_get(_param_handle_mpc_yaw_mode, &_param_mpc_yaw_mode);
	}

	applyPendingTarget();

	if (!_target_valid) {
		// Entered without a fresh target (e.g. rejected by the geofence): hold instead of flying back.
		setTargetToStopPoint();
	}

	publishGotoSetpoint();
}

void
Goto::on_active()
{
	applyPendingTarget();
	publishGotoSetpoint();
}

void
Goto::on_inactive()
{
	_target_valid = false;
}

void
Goto::setTarget(double lat, double lon, float alt, float heading, float cruising_speed)
{
	_pending_target.lat = lat;
	_pending_target.lon = lon;
	_pending_target.alt = alt;
	_pending_target.heading = heading;
	_pending_target.cruising_speed = cruising_speed;
	_pending_target_timestamp = hrt_absolute_time();
}

void
Goto::setAltitude(float alt)
{
	if (_target_valid) {
		setTarget(_target.lat, _target.lon, alt, _target.heading, _navigator->get_cruising_speed());
	}
}

bool
Goto::getTarget(double &lat, double &lon, float &alt, float &heading) const
{
	if (!_target_valid) {
		return false;
	}

	lat = _target.lat;
	lon = _target.lon;
	alt = _target.alt;
	heading = _target.heading;
	return true;
}

void
Goto::applyPendingTarget()
{
	if (_pending_target_timestamp == 0) {
		return;
	}

	// Stale if the mode switch requested with the target didn't happen right away
	if (hrt_elapsed_time(&_pending_target_timestamp) < 500_ms) {
		_target = _pending_target;
		_target_valid = true;
		_locked_heading = NAN;

		if (PX4_ISFINITE(_target.cruising_speed) && (_target.cruising_speed > 0.f)) {
			_navigator->set_cruising_speed(_target.cruising_speed);

		} else {
			_navigator->reset_cruising_speed();
		}
	}

	_pending_target_timestamp = 0;
}

void
Goto::setTargetToStopPoint()
{
	_target.lat = _navigator->get_global_position()->lat;
	_target.lon = _navigator->get_global_position()->lon;
	_navigator->preproject_stop_point(_target.lat, _target.lon);
	_target.alt = _navigator->get_global_position()->alt;
	_target.heading = NAN;
	_target_valid = true;
}

float
Goto::headingFromYawMode()
{
	// MPC_YAW_MODE values, see FlightTaskAuto
	static constexpr int32_t kTowardsHome = 1;
	static constexpr int32_t kAwayFromHome = 2;
	static constexpr int32_t kYawFixed = 5;

	const vehicle_local_position_s &local_pos = *_navigator->get_local_position();

	if (!local_pos.heading_good_for_control) {
		_locked_heading = NAN;
		return NAN;
	}

	const vehicle_global_position_s &global_pos = *_navigator->get_global_position();
	const home_position_s &home = *_navigator->get_home_position();

	// Point from -> to. Towards the target for towards waypoint (with or without yaw first) and along
	// trajectory, as Goto flies a straight line to the target.
	double from_lat = global_pos.lat;
	double from_lon = global_pos.lon;
	double to_lat = _target.lat;
	double to_lon = _target.lon;
	bool has_direction = true;

	switch (_param_mpc_yaw_mode) {
	case kTowardsHome:
		to_lat = home.lat;
		to_lon = home.lon;
		has_direction = home.valid_hpos;
		break;

	case kAwayFromHome:
		from_lat = home.lat;
		from_lon = home.lon;
		to_lat = global_pos.lat;
		to_lon = global_pos.lon;
		has_direction = home.valid_hpos;
		break;

	case kYawFixed:
		has_direction = false;
		break;

	default:
		break;
	}

	// Only point while outside the acceptance radius, then lock to prevent excessive yawing
	if (has_direction
	    && get_distance_to_next_waypoint(from_lat, from_lon, to_lat, to_lon) > _navigator->get_acceptance_radius()) {
		_locked_heading = NAN;
		return get_bearing_to_next_waypoint(from_lat, from_lon, to_lat, to_lon);
	}

	if (!PX4_ISFINITE(_locked_heading)) {
		_locked_heading = local_pos.heading;
	}

	return _locked_heading;
}

void
Goto::publishGotoSetpoint()
{
	const vehicle_local_position_s &local_pos = *_navigator->get_local_position();

	if (!_target_valid || !local_pos.xy_global || !local_pos.z_global) {
		// No EKF origin: nothing to convert the global target into.
		return;
	}

	// Re-projecting every cycle keeps the target fixed in the world across EKF origin resets.
	if (!_geo_projection.isInitialized()
	    || (_geo_projection.getProjectionReferenceTimestamp() != local_pos.ref_timestamp)) {
		_geo_projection.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);
	}

	goto_setpoint_s goto_setpoint{};
	_geo_projection.project(_target.lat, _target.lon, goto_setpoint.position[0], goto_setpoint.position[1]);
	goto_setpoint.position[2] = -(_target.alt - local_pos.ref_alt);
	const float heading = PX4_ISFINITE(_target.heading) ? _target.heading : headingFromYawMode();
	goto_setpoint.flag_control_heading = PX4_ISFINITE(heading);
	goto_setpoint.heading = heading;

	const float cruising_speed = _navigator->get_cruising_speed();
	goto_setpoint.flag_set_max_horizontal_speed = PX4_ISFINITE(cruising_speed) && (cruising_speed > 0.f);
	goto_setpoint.max_horizontal_speed = cruising_speed;
	goto_setpoint.flag_set_max_vertical_speed = false;
	goto_setpoint.flag_set_max_heading_rate = false;
	goto_setpoint.timestamp = hrt_absolute_time();

	_goto_setpoint_pub.publish(goto_setpoint);
}
