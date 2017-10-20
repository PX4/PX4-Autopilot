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
 * @file precland.cpp
 *
 * Helper class to do precision landing with a beacon
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#define SEC2USEC 1000000.0f

#define STATE_TIMEOUT 10000000 // [us] Maximum time to spend in any state

PrecLand::PrecLand(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_beaconPositionSub(0),
	_beacon_position_valid(false),
	_state_start_time(0),
	_search_cnt(0),
	_approach_alt(0),
	_param_timeout(this, "PLD_BTOUT", false),
	_param_hacc_rad(this, "PLD_HACC_RAD", false),
	_param_final_approach_alt(this, "PLD_FAPPR_ALT", false),
	_param_search_alt(this, "PLD_SRCH_ALT", false),
	_param_search_timeout(this, "PLD_SRCH_TOUT", false),
	_param_max_searches(this, "PLD_MAX_SRCH", false)
{
	/* load initial params */
	updateParams();

}

PrecLand::~PrecLand()
{
}

void
PrecLand::on_inactive()
{
}

void
PrecLand::on_activation()
{
	// We need to subscribe here and not in the constructor because constructor is called before the navigator task is spawned
	if (!_beaconPositionSub)
	{
		_beaconPositionSub = orb_subscribe(ORB_ID(beacon_position));
	}

	_state = PrecLandState::Start;
	_search_cnt = 0;
	
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->next.valid = false;

	// Check that the current position setpoint is valid and close to current position
	bool reset_to_cur_pos = false;
	if (!pos_sp_triplet->current.valid)
	{
		reset_to_cur_pos = true;
	} else {
		float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		if (dist > _navigator->get_acceptance_radius())
		{
			reset_to_cur_pos = true;
		}
	}

	if (reset_to_cur_pos)
	{
		PX4_WARN("Resetting landing position to current position");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
	}

	_first_sp = pos_sp_triplet->current; // store the current setpoint for later
	pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
	pos_sp_triplet->previous.valid = true;

	switch_to_state_start();

}

void
PrecLand::on_active()
{
	// get new beacon measurement
	bool updated = false;
	orb_check(_beaconPositionSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(beacon_position), _beaconPositionSub, &_beacon_position);
		_beacon_position_valid = true;
	}

	if (hrt_absolute_time() - _beacon_position.timestamp > (uint64_t)(_param_timeout.get()*SEC2USEC))
	{
		_beacon_position_valid = false;
	}

	switch(_state) {
		case PrecLandState::Start:
			run_state_start();
			break;
		case PrecLandState::HorizontalApproach:
			run_state_horizontal_approach();
			break;
		case PrecLandState::DescendAboveBeacon:
			run_state_descend_above_beacon();
			break;
		case PrecLandState::FinalApproach:
			run_state_final_approach();
			break;
		case PrecLandState::Search:
			run_state_search();
			break;
		case PrecLandState::Fallback:
			run_state_fallback();
			break;
		default:
			// unknown state
			break;
	}

}

void
PrecLand::run_state_start()
{
	// check if beacon visible and go to horizontal approach
	if (switch_to_state_horizontal_approach())
	{
		return;
	}

	// if no, go to search (after 2 seconds)
	if (hrt_absolute_time() - _state_start_time > 2000000)
	{
		// check if we've reached the start point
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		
		if (dist < _navigator->get_acceptance_radius())
		{
			if (!switch_to_state_search())
			{
				if (!switch_to_state_fallback())
				{
					PX4_ERR("Can't switch to search or fallback landing");
				}
			}
		}
	}
}

void
PrecLand::run_state_horizontal_approach()
{
	vehicle_local_position_s * vehicle_local_position = _navigator->get_local_position();

	// check if beacon visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach))
	{
		PX4_WARN("Lost beacon while landig.");
		if (!switch_to_state_start())
		{
			if (!switch_to_state_fallback())
			{
				PX4_ERR("Can't switch to fallback landing");
			}
		}
		return;
	}

	// if close enough for descent above beacon go to descend above beacon
	if (switch_to_state_descend_above_beacon())
	{
		return;
	}

	if (hrt_absolute_time() - _state_start_time > STATE_TIMEOUT){
		PX4_ERR("Precision landing took too long during horizontal approach phase.");
		if (!switch_to_state_fallback())
		{
			PX4_ERR("Can't switch to fallback landing");
		}
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}
	double lat, lon;
	map_projection_reproject(&_map_ref, _beacon_position.x_abs, _beacon_position.y_abs, &lat, &lon); // XXX need to transform to GPS coords because mc_pos_control only looks at that

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	pos_sp_triplet->current.alt = _approach_alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	_navigator->set_position_setpoint_triplet_updated();

}

void
PrecLand::run_state_descend_above_beacon()
{
	vehicle_local_position_s * vehicle_local_position = _navigator->get_local_position();

	// check if beacon visible
	if (!check_state_conditions(PrecLandState::DescendAboveBeacon))
	{
		if (!switch_to_state_final_approach())
		{
			PX4_WARN("Lost beacon while landing");
			if (!switch_to_state_start())
			{
				if (!switch_to_state_fallback())
				{
					PX4_ERR("Can't switch to fallback landing");
				}
			}
		}
		return;
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}
	double lat, lon;
	map_projection_reproject(&_map_ref, _beacon_position.x_abs, _beacon_position.y_abs, &lat, &lon); // XXX need to transform to GPS coords because mc_pos_control only looks at that

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;

	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

	_navigator->set_position_setpoint_triplet_updated();

}

void
PrecLand::run_state_final_approach()
{
	// nothing to do, will land
}

void
PrecLand::run_state_search()
{
	// try to switch to horizontal approach
	if (switch_to_state_horizontal_approach())
	{
		return;
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_search_timeout.get()*SEC2USEC)
	{
		PX4_WARN("Search timed out");
		if (!switch_to_state_fallback())
		{
			PX4_ERR("Can't switch to fallback landing");
		}
	}
}

void
PrecLand::run_state_fallback()
{
	// nothing to do, will land
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start))
	{
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current = _first_sp;
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		_navigator->set_position_setpoint_triplet_updated();
		_search_cnt++;
		
		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();
		return true;
	}
	
	return false;
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach))
	{
		_approach_alt = _navigator->get_global_position()->alt;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}
	
	return false;
}

bool
PrecLand::switch_to_state_descend_above_beacon()
{
	if (check_state_conditions(PrecLandState::DescendAboveBeacon))
	{
		_state = PrecLandState::DescendAboveBeacon;
		_state_start_time = hrt_absolute_time();
		return true;
	}
	
	return false;
}

bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach))
	{
		vehicle_status_s *vstatus = _navigator->get_vstatus();
		vehicle_command_s cmd = {
			.timestamp = 0,
			.param5 = NAN,
			.param6 = NAN,
			/* minimum pitch */
			.param1 = NAN,
			.param2 = NAN,
			.param3 = NAN,
			.param4 = NAN,
			.param7 = NAN,
			.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND,
			.target_system = (uint8_t)vstatus->system_id,
			.target_component = (uint8_t)vstatus->component_id
		};

		orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);

		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}
	return false;
}

bool
PrecLand::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude.");
	vehicle_local_position_s * vehicle_local_position = _navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current = _first_sp;
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_search_alt.get();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_fallback()
{
	PX4_WARN("Falling back to normal land.");
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current = _first_sp; // should this be at current position?
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	vehicle_local_position_s * vehicle_local_position = _navigator->get_local_position();

	switch(state) {
		case PrecLandState::Start:
			return _search_cnt <= _param_max_searches.get();

		case PrecLandState::HorizontalApproach:
			return _beacon_position_valid && _beacon_position.abs_pos_valid;
		
		case PrecLandState::DescendAboveBeacon:
			// if we're already in this state, only leave it if beacon becomes unusable, don't care about horizontall offset to beacon
			if (_state == PrecLandState::DescendAboveBeacon)
			{
				// if we're close to the ground, we're more critical of beacon timeouts so we quickly go into descend
				if (check_state_conditions(PrecLandState::FinalApproach))
				{
					return hrt_absolute_time() - _beacon_position.timestamp < 500000; // 0.5s
				} else {
					return _beacon_position_valid && _beacon_position.abs_pos_valid;
				}
			} else {
				// if not already in this state, need to be above beacon to enter it
				return _beacon_position_valid && _beacon_position.abs_pos_valid
				&& fabsf(_beacon_position.x_rel) < _param_hacc_rad.get() && fabsf(_beacon_position.x_rel) < _param_hacc_rad.get();
			}
		
		case PrecLandState::FinalApproach:
			return vehicle_local_position->dist_bottom_valid && vehicle_local_position->dist_bottom < _param_final_approach_alt.get();
		
		case PrecLandState::Search:
			return true;
		
		case PrecLandState::Fallback:
			return true;
		
		default:
			return false;
	}
}