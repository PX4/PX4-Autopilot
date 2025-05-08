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
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include "land.h"
#include "navigator.h"

Land::Land(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
{
}

void
Land::on_activation()
{
	/* set current mission item to Land */
	set_land_item(&_mission_item);
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	/* convert mission item to current setpoint */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _navigator->get_local_position()->xy_global) { // only execute if global position is valid
		_navigator->preproject_stop_point(_mission_item.lat, _mission_item.lon);
	}

	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();

	// reset cruising speed to default
	_navigator->reset_cruising_speed();

	// set gimbal to neutral position (level with horizon) to reduce change of damage on landing
	_navigator->acquire_gimbal_control();
	_navigator->set_gimbal_neutral();
	_navigator->release_gimbal_control();

}

void
Land::on_active()
{
	/* for VTOL update landing location during back transition */
	if (_navigator->get_vstatus()->is_vtol
	    && _navigator->get_vstatus()->in_transition_mode
	    && _navigator->get_local_position()->xy_global) {
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		// create a wp in front of the VTOL while in back-transition, based on MPC settings that will apply in MC phase afterwards
		_navigator->preproject_stop_point(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);
		_navigator->set_position_setpoint_triplet_updated();
	}


	if (_navigator->get_land_detected()->landed) {
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();
		_navigator->mode_completed(getNavigatorStateId());
		set_idle_item(&_mission_item);

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}

	/* check if landing needs to be aborted */
	if (_navigator->abort_landing()) {

		// send reposition cmd to get out of land mode (will loiter at current position and altitude)
		vehicle_command_s vehicle_command{};
		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
		vehicle_command.param1 = -1.f; // Default speed
		vehicle_command.param2 = 1.f; // Modes should switch, not setting this is unsupported
		vehicle_command.param5 = _navigator->get_global_position()->lat;
		vehicle_command.param6 = _navigator->get_global_position()->lon;
		// as we don't know the landing point altitude assume the worst case (abort at 0m above ground),
		// and thus always climb MIS_LND_ABRT_ALT
		vehicle_command.param7 = _navigator->get_global_position()->alt + _navigator->get_landing_abort_min_alt();

		_navigator->publish_vehicle_command(vehicle_command);
	}
}
