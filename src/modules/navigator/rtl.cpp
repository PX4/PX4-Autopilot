/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file navigator_rtl.cpp
 * Helper class to access RTL
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include "rtl.h"

RTL::RTL(Navigator *navigator, const char *name) :
	Mission(navigator, name),
	_mavlink_fd(-1),
	_rtl_state(RTL_STATE_NONE),
	_home_position({}),
	_loiter_radius(50),
	_acceptance_radius(50),
	_param_loiter_rad(this, "LOITER_RAD"),
	_param_return_alt(this, "RETURN_ALT"),
	_param_descend_alt(this, "DESCEND_ALT"),
	_param_land_delay(this, "LAND_DELAY")
{
	/* load initial params */
	updateParams();
	/* initial reset */
	reset();
}

RTL::~RTL()
{
}

bool
RTL::update(struct position_setpoint_triplet_s *pos_sp_triplet)
{
	bool updated = false;

	return updated;
}

void
RTL::reset()
{

}

void
RTL::set_home_position(const home_position_s *new_home_position)
{
	memcpy(&_home_position, new_home_position, sizeof(home_position_s));
}

bool
RTL::get_current_rtl_item(const vehicle_global_position_s *global_position, mission_item_s *new_mission_item)
{
	/* Open mavlink fd */
	if (_mavlink_fd < 0) {
		/* try to open the mavlink log device every once in a while */
		_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	}

	/* decide if we need climb */
	if (_rtl_state == RTL_STATE_NONE) {
		if (global_position->alt < _home_position.alt + _param_return_alt.get()) {
			_rtl_state = RTL_STATE_CLIMB;

		} else {
			_rtl_state = RTL_STATE_RETURN;
		}
	}

	/* if switching directly to return state, set altitude setpoint to current altitude */
	if (_rtl_state == RTL_STATE_RETURN) {
		new_mission_item->altitude_is_relative = false;
		new_mission_item->altitude = global_position->alt;
	}

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

		float climb_alt = _home_position.alt + _param_return_alt.get();

		/* TODO understand and fix this */
		// if (_vstatus.condition_landed) {
		// 	climb_alt = fmaxf(climb_alt, _global_pos.alt + _parameters.rtl_alt);
		// }

		new_mission_item->lat = global_position->lat;
		new_mission_item->lon = global_position->lon;
		new_mission_item->altitude_is_relative = false;
		new_mission_item->altitude = climb_alt;
		new_mission_item->yaw = NAN;
		new_mission_item->loiter_radius = _loiter_radius;
		new_mission_item->loiter_direction = 1;
		new_mission_item->nav_cmd = NAV_CMD_WAYPOINT;
		new_mission_item->acceptance_radius = _acceptance_radius;
		new_mission_item->time_inside = 0.0f;
		new_mission_item->pitch_min = 0.0f;
		new_mission_item->autocontinue = true;
		new_mission_item->origin = ORIGIN_ONBOARD;

		mavlink_log_info(_mavlink_fd, "#audio: RTL: climb to %d meters above home",
			(int)(climb_alt - _home_position.alt));
		break;
	}
	case RTL_STATE_RETURN: {

		new_mission_item->lat = _home_position.lat;
		new_mission_item->lon = _home_position.lon;

		/* TODO: add this again */
		// don't change altitude
		// if (_pos_sp_triplet.previous.valid) {
		// 	/* if previous setpoint is valid then use it to calculate heading to home */
		// 	new_mission_item->yaw = get_bearing_to_next_waypoint(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon, new_mission_item->lat, new_mission_item->lon);

		// } else {
		// 	/* else use current position */
		// 	new_mission_item->yaw = get_bearing_to_next_waypoint(_global_pos.lat, _global_pos.lon, new_mission_item->lat, new_mission_item->lon);
		// }
		new_mission_item->loiter_radius = _loiter_radius;
		new_mission_item->loiter_direction = 1;
		new_mission_item->nav_cmd = NAV_CMD_WAYPOINT;
		new_mission_item->acceptance_radius = _acceptance_radius;
		new_mission_item->time_inside = 0.0f;
		new_mission_item->pitch_min = 0.0f;
		new_mission_item->autocontinue = true;
		new_mission_item->origin = ORIGIN_ONBOARD;

		mavlink_log_info(_mavlink_fd, "#audio: RTL: return at %d meters above home",
			(int)(new_mission_item->altitude - _home_position.alt));
		break;
	}

	case RTL_STATE_DESCEND: {

		new_mission_item->lat = _home_position.lat;
		new_mission_item->lon = _home_position.lon;
		new_mission_item->altitude_is_relative = false;
		new_mission_item->altitude = _home_position.alt + _param_descend_alt.get();
		new_mission_item->yaw = NAN;
		new_mission_item->loiter_radius = _loiter_radius;
		new_mission_item->loiter_direction = 1;
		new_mission_item->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
		new_mission_item->acceptance_radius = _acceptance_radius;
		new_mission_item->time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
		new_mission_item->pitch_min = 0.0f;
		new_mission_item->autocontinue = _param_land_delay.get() > -0.001f;
		new_mission_item->origin = ORIGIN_ONBOARD;

		mavlink_log_info(_mavlink_fd, "#audio: RTL: descend to %d meters above home",
			(int)(new_mission_item->altitude - _home_position.alt));
		break;
	}

	case RTL_STATE_LAND: {

		new_mission_item->lat = _home_position.lat;
		new_mission_item->lon = _home_position.lon;
		new_mission_item->altitude_is_relative = false;
		new_mission_item->altitude = _home_position.alt;
		new_mission_item->yaw = NAN;
		new_mission_item->loiter_radius = _loiter_radius;
		new_mission_item->loiter_direction = 1;
		new_mission_item->nav_cmd = NAV_CMD_LAND;
		new_mission_item->acceptance_radius = _acceptance_radius;
		new_mission_item->time_inside = 0.0f;
		new_mission_item->pitch_min = 0.0f;
		new_mission_item->autocontinue = true;
		new_mission_item->origin = ORIGIN_ONBOARD;

		mavlink_log_info(_mavlink_fd, "#audio: RTL: land at home");
		break;
	}

	case RTL_STATE_FINISHED: {
		/* nothing to do, report fail */
		return false;
	}

	default:
		return false;
	}

	return true;
}

bool
RTL::get_next_rtl_item(mission_item_s *new_mission_item)
{
	/* TODO implement */
	return false;
}

void
RTL::move_to_next()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_DESCEND;
		break;

	case RTL_STATE_DESCEND:
		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < 0) {
			_rtl_state = RTL_STATE_FINISHED;
		} else {
			_rtl_state = RTL_STATE_LAND;
		}
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_FINISHED;
		break;

	case RTL_STATE_FINISHED:
		break;

	default:
		break;
	}
}
