/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file precland_mode.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland_mode.h"
#include "navigator.h"

#include <uORB/topics/position_setpoint_triplet.h>

PrecLandMode::PrecLandMode(Navigator *navigator) :
	MissionBlock(navigator)
{

}

void
PrecLandMode::on_activation()
{
	_global_pos_sub.update();
	// Get the landing position from current position_setpoint, else use the current vehicle position.
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	PrecLand::LandingPosition2D approximate_landing_pos{ .lat = pos_sp_triplet->current.lat,
			.lon = pos_sp_triplet->current.lon};

	if (!pos_sp_triplet->current.valid) {
		PX4_WARN("No valid landing position for precision landing. Using current position");
		approximate_landing_pos.lat = _global_pos_sub.get().lat;
		approximate_landing_pos.lon = _global_pos_sub.get().lon;
	}

	_prec_land.initialize(approximate_landing_pos);
}

void
PrecLandMode::on_active()
{
	_local_pos_sub.update();

	_prec_land.setAcceptanceRadius(_navigator->get_acceptance_radius());
	_prec_land.update();

	PrecLand::Output prec_land_output{_prec_land.getOutput()};

	_mission_item.nav_cmd = prec_land_output.nav_cmd;
	_mission_item.lat = prec_land_output.pos_hor.lat;
	_mission_item.lon = prec_land_output.pos_hor.lon;
	_mission_item.altitude = prec_land_output.alt;
	_mission_item.altitude_is_relative = false;
	_mission_item.yaw = _local_pos_sub.get().heading;


	mission_apply_limitation(_mission_item);

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}

}


