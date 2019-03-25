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
 * @file datalinkloss.cpp
 * Helper class for Data Link Loss Mode according to the OBC rules
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
#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "datalinkloss.h"

DataLinkLoss::DataLinkLoss(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator),
	_dll_state(DLL_STATE_NONE)
{
}

void
DataLinkLoss::on_inactive()
{
	/* reset DLL state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_dll_state = DLL_STATE_NONE;
	}
}

void
DataLinkLoss::on_activation()
{
	_dll_state = DLL_STATE_NONE;
	advance_dll();
	set_dll_item();
}

void
DataLinkLoss::on_active()
{
	if (is_mission_item_reached()) {
		advance_dll();
		set_dll_item();
	}
}

void
DataLinkLoss::set_dll_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous = pos_sp_triplet->current;
	_navigator->set_can_loiter_at_sp(false);

	switch (_dll_state) {
	case DLL_STATE_FLYTOCOMMSHOLDWP: {
			_mission_item.lat = (double)(_param_nav_dll_ch_lat.get()) * 1.0e-7;
			_mission_item.lon = (double)(_param_nav_dll_ch_lon.get()) * 1.0e-7;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _param_nav_dll_ch_alt.get();
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = _param_nav_dll_ch_t.get() < 0.0f ? 0.0f : _param_nav_dll_ch_t.get();
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);
			break;
		}

	case DLL_STATE_FLYTOAIRFIELDHOMEWP: {
			_mission_item.lat = (double)(_param_nav_ah_lat.get()) * 1.0e-7;
			_mission_item.lon = (double)(_param_nav_ah_lon.get()) * 1.0e-7;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _param_nav_ah_alt.get();
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.time_inside = _param_nav_dll_ah_t.get() < 0.0f ? 0.0f : _param_nav_dll_ah_t.get();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);
			break;
		}

	case DLL_STATE_TERMINATE: {
			/* Request flight termination from the commander */
			_navigator->get_mission_result()->flight_termination = true;
			_navigator->set_mission_result_updated();
			reset_mission_item_reached();
			warnx("not switched to manual: request flight termination");
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
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
DataLinkLoss::advance_dll()
{
	switch (_dll_state) {
	case DLL_STATE_NONE:

		/* Check the number of data link losses. If above home fly home directly */
		/* if number of data link losses limit is not reached fly to comms hold wp */
		if (_navigator->get_vstatus()->data_link_lost_counter > _param_nav_dll_n.get()) {
			warnx("%d data link losses, limit is %d, fly to airfield home",
			      _navigator->get_vstatus()->data_link_lost_counter, _param_nav_dll_n.get());
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "too many DL losses, fly to airfield home");
			_navigator->get_mission_result()->stay_in_failsafe = true;
			_navigator->set_mission_result_updated();
			reset_mission_item_reached();
			_dll_state = DLL_STATE_FLYTOAIRFIELDHOMEWP;

		} else {
			if (!_param_nav_dll_chsk.get()) {
				warnx("fly to comms hold, datalink loss counter: %d", _navigator->get_vstatus()->data_link_lost_counter);
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "fly to comms hold");
				_dll_state = DLL_STATE_FLYTOCOMMSHOLDWP;

			} else {
				/* comms hold wp not active, fly to airfield home directly */
				warnx("Skipping comms hold wp. Flying directly to airfield home");
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "fly to airfield home, comms hold skipped");
				_dll_state = DLL_STATE_FLYTOAIRFIELDHOMEWP;
			}
		}

		break;

	case DLL_STATE_FLYTOCOMMSHOLDWP:
		warnx("fly to airfield home");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "fly to airfield home");
		_dll_state = DLL_STATE_FLYTOAIRFIELDHOMEWP;
		_navigator->get_mission_result()->stay_in_failsafe = true;
		_navigator->set_mission_result_updated();
		reset_mission_item_reached();
		break;

	case DLL_STATE_FLYTOAIRFIELDHOMEWP:
		_dll_state = DLL_STATE_TERMINATE;
		warnx("time is up, state should have been changed manually by now");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "no manual control, terminating");
		_navigator->get_mission_result()->stay_in_failsafe = true;
		_navigator->set_mission_result_updated();
		reset_mission_item_reached();
		break;

	case DLL_STATE_TERMINATE:
		warnx("dll end");
		_dll_state = DLL_STATE_END;
		break;

	default:
		break;
	}
}
