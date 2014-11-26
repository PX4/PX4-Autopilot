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
 * @file rcloss.cpp
 * Helper class for RC Loss Mode according to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "datalinkloss.h"

#define DELAY_SIGMA	0.01f

RCLoss::RCLoss(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_loitertime(this, "LT"),
	_rcl_state(RCL_STATE_NONE)
{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

RCLoss::~RCLoss()
{
}

void
RCLoss::on_inactive()
{
	/* reset RCL state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_rcl_state = RCL_STATE_NONE;
	}
}

void
RCLoss::on_activation()
{
	_rcl_state = RCL_STATE_NONE;
	updateParams();
	advance_rcl();
	set_rcl_item();
}

void
RCLoss::on_active()
{
	if (is_mission_item_reached()) {
		updateParams();
		advance_rcl();
		set_rcl_item();
	}
}

void
RCLoss::set_rcl_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	set_previous_pos_setpoint();
	_navigator->set_can_loiter_at_sp(false);

	switch (_rcl_state) {
	case RCL_STATE_LOITER: {
		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.altitude = _navigator->get_global_position()->alt;
		_mission_item.altitude_is_relative = false;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = _param_loitertime.get() < 0.0f ? 0.0f : _param_loitertime.get();
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		_navigator->set_can_loiter_at_sp(true);
		break;
	}
	case RCL_STATE_TERMINATE: {
		/* Request flight termination from the commander */
		_navigator->get_mission_result()->flight_termination = true;
		_navigator->publish_mission_result();
		warnx("rc not recovered: request flight termination");
		pos_sp_triplet->previous.valid = false;
		pos_sp_triplet->current.valid = false;
		pos_sp_triplet->next.valid = false;
		break;
	}
	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
RCLoss::advance_rcl()
{
	switch (_rcl_state) {
	case RCL_STATE_NONE:
		if (_param_loitertime.get() > 0.0f) {
			warnx("RC loss, OBC mode, loiter");
			mavlink_log_info(_navigator->get_mavlink_fd(), "#audio: rc loss, loitering");
			_rcl_state = RCL_STATE_LOITER;
		} else {
			warnx("RC loss, OBC mode, slip loiter, terminate");
			mavlink_log_info(_navigator->get_mavlink_fd(), "#audio: rc loss, terminating");
			_rcl_state = RCL_STATE_TERMINATE;
			_navigator->get_mission_result()->stay_in_failsafe = true;
			_navigator->publish_mission_result();
			reset_mission_item_reached();
		}
		break;
	case RCL_STATE_LOITER:
		_rcl_state = RCL_STATE_TERMINATE;
		warnx("time is up, no RC regain, terminating");
		mavlink_log_info(_navigator->get_mavlink_fd(), "#audio: RC not regained, terminating");
		_navigator->get_mission_result()->stay_in_failsafe = true;
		_navigator->publish_mission_result();
		reset_mission_item_reached();
		break;
	case RCL_STATE_TERMINATE:
		warnx("rcl end");
		_rcl_state = RCL_STATE_END;
		break;
	default:
		break;
	}
}
