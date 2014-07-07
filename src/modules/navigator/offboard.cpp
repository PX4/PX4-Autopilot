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
 * @file offboard.cpp
 *
 * Helper class for offboard commands
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "navigator.h"
#include "offboard.h"

Offboard::Offboard(Navigator *navigator, const char *name) :
	NavigatorMode(navigator, name),
	_offboard_control_sp({0})
{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

Offboard::~Offboard()
{
}

bool
Offboard::on_active(struct position_setpoint_triplet_s *pos_sp_triplet)
{
	bool updated;
	orb_check(_navigator->get_offboard_control_sp_sub(), &updated);
	if (updated) {
		update_offboard_control_setpoint();
	}

	bool changed = false;

	/* copy offboard setpoints to the corresponding topics */
	if (_offboard_control_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED) {
		/* We accept position control only if none of the directions is ignored (as pos_sp_triplet does not
		 * support deactivation of individual directions) */
		if (_navigator->get_control_mode()->flag_control_position_enabled &&
				(!offboard_control_sp_ignore_position(_offboard_control_sp, 0) &&
				 !offboard_control_sp_ignore_position(_offboard_control_sp, 1) &&
				 !offboard_control_sp_ignore_position(_offboard_control_sp, 2))) {
			/* position control */
			pos_sp_triplet->current.x = _offboard_control_sp.position[0];
			pos_sp_triplet->current.y = _offboard_control_sp.position[1];
			//pos_sp_triplet->current.yaw = _offboard_control_sp.position[2];
			//XXX: copy yaw
			pos_sp_triplet->current.z = -_offboard_control_sp.position[2];

			pos_sp_triplet->current.type = SETPOINT_TYPE_OFFBOARD;
			pos_sp_triplet->current.valid = true;
			pos_sp_triplet->current.position_valid = true;

			changed = true;

		}
		/* We accept velocity control only if none of the directions is ignored (as pos_sp_triplet does not
		 * support deactivation of individual directions) */
		if (_navigator->get_control_mode()->flag_control_velocity_enabled &&
				(!offboard_control_sp_ignore_velocity(_offboard_control_sp, 0) &&
				 !offboard_control_sp_ignore_velocity(_offboard_control_sp, 1) &&
				 !offboard_control_sp_ignore_velocity(_offboard_control_sp, 2))) {
			/* velocity control */
			pos_sp_triplet->current.vx = _offboard_control_sp.velocity[0];
			pos_sp_triplet->current.vy = _offboard_control_sp.velocity[1];
//			pos_sp_triplet->current.yawspeed = _offboard_control_sp.velocity[;
//			//XXX: copy yaw speed
			pos_sp_triplet->current.vz = _offboard_control_sp.velocity[2];

			pos_sp_triplet->current.type = SETPOINT_TYPE_OFFBOARD;
			pos_sp_triplet->current.valid = true;
			pos_sp_triplet->current.velocity_valid = true;

			changed = true;
		}

		//XXX: map acceleration setpoint once supported in setpoint triplet
	}

	return changed;
}

void
Offboard::on_inactive()
{
}

void
Offboard::update_offboard_control_setpoint()
{
	orb_copy(ORB_ID(offboard_control_setpoint), _navigator->get_offboard_control_sp_sub(), &_offboard_control_sp);

}
