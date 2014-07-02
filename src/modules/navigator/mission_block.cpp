/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mission_block.cpp
 *
 * Helper class to use mission items
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <float.h>

#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>

#include "navigator.h"
#include "mission_block.h"


MissionBlock::MissionBlock(Navigator *navigator, const char *name) :
	NavigatorMode(navigator, name),
	_waypoint_position_reached(false),
	_waypoint_yaw_reached(false),
	_time_first_inside_orbit(0),
	_mission_item({0}),
	_mission_item_valid(false)
{
}

MissionBlock::~MissionBlock()
{
}

bool
MissionBlock::is_mission_item_reached()
{
	if (_mission_item.nav_cmd == NAV_CMD_LAND) {
		return _navigator->get_vstatus()->condition_landed;
	}

	/* TODO: count turns */
	if ((/*_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||*/
	     _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {
		return false;
	}

	hrt_abstime now = hrt_absolute_time();

	if (!_waypoint_position_reached) {

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		float altitude_amsl = _mission_item.altitude_is_relative
				      ? _mission_item.altitude + _navigator->get_home_position()->alt
			              : _mission_item.altitude;

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, altitude_amsl,
				                          _navigator->get_global_position()->lat,
							  _navigator->get_global_position()->lon,
							  _navigator->get_global_position()->alt,
				&dist_xy, &dist_z);

		if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF && _navigator->get_vstatus()->is_rotary_wing) {
			/* require only altitude for takeoff for multicopter */
			if (_navigator->get_global_position()->alt >
				altitude_amsl - _navigator->get_acceptance_radius()) {
				_waypoint_position_reached = true;
			}
		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
			/* for takeoff mission items use the parameter for the takeoff acceptance radius */
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius()) {
				_waypoint_position_reached = true;
			}
		} else {
			/* for normal mission items used their acceptance radius */
			if (dist >= 0.0f && dist <= _mission_item.acceptance_radius) {
				_waypoint_position_reached = true;
			}
		}
	}

	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		/* TODO: removed takeoff, why? */
		if (_navigator->get_vstatus()->is_rotary_wing && isfinite(_mission_item.yaw)) {

			/* check yaw if defined only for rotary wing except takeoff */
			float yaw_err = _wrap_pi(_mission_item.yaw - _navigator->get_global_position()->yaw);

			if (fabsf(yaw_err) < 0.2f) { /* TODO: get rid of magic number */
				_waypoint_yaw_reached = true;
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	/* check if the current waypoint was reached */
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		if (_time_first_inside_orbit == 0) {
			_time_first_inside_orbit = now;

			// if (_mission_item.time_inside > 0.01f) {
			// 	mavlink_log_info(_mavlink_fd, "#audio: waypoint reached, wait for %.1fs",
			// 		(double)_mission_item.time_inside);
			// }
		}

		/* check if the MAV was long enough inside the waypoint orbit */
		if (now - _time_first_inside_orbit >= (hrt_abstime)_mission_item.time_inside * 1e6f) {
			return true;
		}
	}
	return false;
}

void
MissionBlock::reset_mission_item_reached()
{
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_first_inside_orbit = 0;
}

void
MissionBlock::mission_item_to_position_setpoint(const struct mission_item_s *item, struct position_setpoint_s *sp)
{
	sp->valid = true;
	sp->lat = item->lat;
	sp->lon = item->lon;
	sp->alt = item->altitude_is_relative ? item->altitude + _navigator->get_home_position()->alt : item->altitude;
	sp->yaw = item->yaw;
	sp->loiter_radius = item->loiter_radius;
	sp->loiter_direction = item->loiter_direction;
	sp->pitch_min = item->pitch_min;

	switch (item->nav_cmd) {
	case NAV_CMD_IDLE:
		sp->type = SETPOINT_TYPE_IDLE;
		break;

	case NAV_CMD_TAKEOFF:
		sp->type = SETPOINT_TYPE_TAKEOFF;
		break;

	case NAV_CMD_LAND:
		sp->type = SETPOINT_TYPE_LAND;
		break;

	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_TURN_COUNT:
	case NAV_CMD_LOITER_UNLIMITED:
		sp->type = SETPOINT_TYPE_LOITER;
		break;

	default:
		sp->type = SETPOINT_TYPE_POSITION;
		break;
	}
}

void
MissionBlock::set_previous_pos_setpoint(struct position_setpoint_triplet_s *pos_sp_triplet)
{
    /* reuse current setpoint as previous setpoint */
    if (pos_sp_triplet->current.valid) {
        memcpy(&pos_sp_triplet->previous, &pos_sp_triplet->current, sizeof(struct position_setpoint_s));
    }
}

bool
MissionBlock::set_loiter_item(struct position_setpoint_triplet_s *pos_sp_triplet)
{
    /* don't change setpoint if 'can_loiter_at_sp' flag set */
	if (!(_navigator->get_can_loiter_at_sp() && pos_sp_triplet->current.valid)) {
		/* use current position */
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.yaw = NAN;	/* NAN means to use current yaw */

	    _navigator->set_can_loiter_at_sp(true);
	}

    if (pos_sp_triplet->current.type != SETPOINT_TYPE_LOITER
            || (fabsf(pos_sp_triplet->current.loiter_radius - _navigator->get_loiter_radius()) > FLT_EPSILON)
            || pos_sp_triplet->current.loiter_direction != 1
            || pos_sp_triplet->previous.valid
            || !pos_sp_triplet->current.valid
            || pos_sp_triplet->next.valid) {
        /* position setpoint triplet should be updated */
        pos_sp_triplet->current.type = SETPOINT_TYPE_LOITER;
        pos_sp_triplet->current.loiter_radius = _navigator->get_loiter_radius();
        pos_sp_triplet->current.loiter_direction = 1;

        pos_sp_triplet->previous.valid = false;
        pos_sp_triplet->current.valid = true;
        pos_sp_triplet->next.valid = false;
        return true;
    }

    return false;
}
