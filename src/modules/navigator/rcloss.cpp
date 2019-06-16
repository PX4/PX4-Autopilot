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

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "datalinkloss.h"

RCLoss::RCLoss(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator),
	_rcl_state(RCL_STATE_NONE)
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
	advance_rcl();
	set_rcl_item();
}

void
RCLoss::on_active()
{
	if (is_mission_item_reached()) {
		advance_rcl();
		set_rcl_item();
	}
}

void
RCLoss::set_rcl_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous = pos_sp_triplet->current;
	_navigator->set_can_loiter_at_sp(false);

	switch (_rcl_state) {
	case RCL_STATE_LOITER: {
			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;
			_mission_item.altitude = _navigator->get_global_position()->alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _navigator->get_global_position()->yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = _param_nav_gpsf_lt.get() < 0.0f ? 0.0f : _param_nav_gpsf_lt.get();
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);
			break;
		}

	case RCL_STATE_TERMINATE: {
			/* Request flight termination from the commander */
			_navigator->get_mission_result()->flight_termination = true;
			_navigator->set_mission_result_updated();
			warnx("RC not recovered: request flight termination");
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
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
RCLoss::advance_rcl()
{
	switch (_rcl_state) {
	case RCL_STATE_NONE:
		if (_param_nav_gpsf_lt.get() > 0.0f) {
			warnx("RC loss, OBC mode, loiter");
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC loss, loitering");
			_rcl_state = RCL_STATE_LOITER;

		} else {
			warnx("RC loss, OBC mode, slip loiter, terminate");
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC loss, terminating");
			_rcl_state = RCL_STATE_TERMINATE;
			_navigator->get_mission_result()->stay_in_failsafe = true;
			_navigator->set_mission_result_updated();
			reset_mission_item_reached();
		}

		break;

	case RCL_STATE_LOITER:
		_rcl_state = RCL_STATE_TERMINATE;
		warnx("time is up, no RC regain, terminating");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC not regained, terminating");
		_navigator->get_mission_result()->stay_in_failsafe = true;
		_navigator->set_mission_result_updated();
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
