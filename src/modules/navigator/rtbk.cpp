/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file src/modules/navigator/rtbk.cpp
 *
 * discrible
 *
 * @author tang liang  <tangliang@qbao.com>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/brkpoint.h>

#include "navigator.h"
#include "rtbk.h"

RTBK::RTBK(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_rtbk_state(RTBK_STATE_NONE),
	_param_bp_valid_time(this, "RTBK_BP_VLD_T", false)
{
	updateParams();
	on_inactive();
}

RTBK::~RTBK()
{
}

void RTBK::on_inactive()
{
	if (!_navigator->get_can_loiter_at_sp()) {
		_rtbk_state = RTBK_STATE_NONE;
	}
}

void RTBK::on_activation()
{
	if (_rtbk_state == RTBK_STATE_NONE) {
		float min_abs_alt = _navigator->get_global_position()->alt  + 5.0f;
		if(_navigator->get_land_detected()->landed || _navigator->get_global_position()->alt < min_abs_alt) {
			_rtbk_state = RTBK_STATE_TAKEOFF;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							"RTBK_STATE_TAKEOFF");

		} else {
			_rtbk_state = RTBK_STATE_RETURN;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							"RTBK_STATE_RETURN");
		}
	}

	set_rtbk_item();
}

void RTBK::on_active()
{
	if (_rtbk_state != RTBK_STATE_RETURN && is_mission_item_reached()) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							"advance rtbk");
		advance_rtbk();
		set_rtbk_item();
	}
}

void RTBK::set_rtbk_item()
{
	updateParams();

	float abs_altitude = _navigator->get_home_position()->alt  + 5.0f;
	struct brkpoint_s bp = {};

	if (dm_read(DM_KEY_BRKPOINT, 0, &bp, sizeof(brkpoint_s)) == sizeof(brkpoint_s)) {
	  mavlink_log_critical(_navigator->get_mavlink_log_pub(),
				"brkpoint: %.6f %.6f %.2f %s", bp.lon, bp.lat, (double)bp.alt, (double)bp.alt, bp.time);
	}

	switch (_rtbk_state) {
	case RTBK_STATE_TAKEOFF:
		if (abs_altitude < _navigator->get_global_position()->alt) {
			abs_altitude = _navigator->get_global_position()->alt;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							"Already higher than takeoff altitude");
		}

		set_takeoff_item(&_mission_item, abs_altitude);
		_mission_item.time_inside = 2.0f;

		break;

	case RTBK_STATE_RETURN:
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.yaw = NAN;
		//_mission_item.lat = _navigator->get_home_position()->lat;
		//_mission_item.lon = _navigator->get_home_position()->lon;
		_mission_item.lat = 47.3979471;
		_mission_item.lon = 8.5445531;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_global_position()->alt;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							"case RTBK_STATE_RETURN");

		break;

	default:
		break;
	}

	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

    // convert mission item to current setpoint
    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
    pos_sp_triplet->previous.valid = false;
    mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
    pos_sp_triplet->current.yaw = _navigator->get_home_position()->yaw;
    pos_sp_triplet->current.yaw_valid = true;
    pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void RTBK::advance_rtbk()
{
	switch (_rtbk_state) {
	case RTBK_STATE_TAKEOFF:
		_rtbk_state = RTBK_STATE_RETURN;
		break;
	default:
		break;
	}
}
