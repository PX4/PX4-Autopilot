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
 * @file gpsfailure.cpp
 * Helper class for gpsfailure mode according to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "gpsfailure.h"

#define DELAY_SIGMA	0.01f

GpsFailure::GpsFailure(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_loitertime(this, "LT"),
	_param_openlooploiter_roll(this, "R"),
	_param_openlooploiter_pitch(this, "P"),
	_param_openlooploiter_thrust(this, "TR"),
	_gpsf_state(GPSF_STATE_NONE),
	_timestamp_activation(0)
{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

GpsFailure::~GpsFailure()
{
}

void
GpsFailure::on_inactive()
{
	/* reset GPSF state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_gpsf_state = GPSF_STATE_NONE;
	}
}

void
GpsFailure::on_activation()
{
	_gpsf_state = GPSF_STATE_NONE;
	_timestamp_activation = hrt_absolute_time();
	updateParams();
	advance_gpsf();
	set_gpsf_item();
}

void
GpsFailure::on_active()
{

	switch (_gpsf_state) {
	case GPSF_STATE_LOITER: {
		/* Position controller does not run in this mode:
		 * navigator has to publish an attitude setpoint */
		_navigator->get_att_sp()->roll_body = M_DEG_TO_RAD_F * _param_openlooploiter_roll.get();
		_navigator->get_att_sp()->pitch_body = M_DEG_TO_RAD_F * _param_openlooploiter_pitch.get();
		_navigator->get_att_sp()->thrust = _param_openlooploiter_thrust.get();
		_navigator->publish_att_sp();

		/* Measure time */
		hrt_abstime elapsed = hrt_elapsed_time(&_timestamp_activation);

		//warnx("open loop loiter, posctl enabled %u, elapsed %.1fs, thrust %.2f",
				//_navigator->get_control_mode()->flag_control_position_enabled, elapsed * 1e-6, (double)_param_openlooploiter_thrust.get());
		if (elapsed > _param_loitertime.get() * 1e6f) {
			/* no recovery, adavance the state machine */
			warnx("gps not recovered, switch to next state");
			advance_gpsf();
		}
		break;
	}
	case GPSF_STATE_TERMINATE:
		set_gpsf_item();
		advance_gpsf();
		break;
	default:
		break;
	}
}

void
GpsFailure::set_gpsf_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Set pos sp triplet to invalid to stop pos controller */
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.valid = false;
	pos_sp_triplet->next.valid = false;

	switch (_gpsf_state) {
	case GPSF_STATE_TERMINATE: {
		/* Request flight termination from the commander */
		_navigator->get_mission_result()->flight_termination = true;
		_navigator->set_mission_result_updated();
		warnx("gps fail: request flight termination");
	}
	default:
		break;
	}

	reset_mission_item_reached();
	_navigator->set_position_setpoint_triplet_updated();
}

void
GpsFailure::advance_gpsf()
{
	updateParams();

	switch (_gpsf_state) {
	case GPSF_STATE_NONE:
		_gpsf_state = GPSF_STATE_LOITER;
		warnx("gpsf loiter");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "GPS failed: open loop loiter");
		break;
	case GPSF_STATE_LOITER:
		_gpsf_state = GPSF_STATE_TERMINATE;
		warnx("gpsf terminate");
		mavlink_log_emergency(_navigator->get_mavlink_log_pub(), "no gps recovery, termination");
		warnx("mavlink sent");
		break;
	case GPSF_STATE_TERMINATE:
		warnx("gpsf end");
		_gpsf_state = GPSF_STATE_END;
	default:
		break;
	}
}
