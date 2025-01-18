/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file RoverLandDetector.cpp
 * Land detection algorithm for Rovers
 *
 * @author Roman Bapst <bapstroma@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include "RoverLandDetector.h"

namespace land_detector
{

bool RoverLandDetector::_get_ground_contact_state()
{
	return true;
}

bool RoverLandDetector::_get_landed_state()
{
	_update_topics();
	const float distance_to_home = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
				       _home_position(0), _home_position(1));

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
	    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {
		return true; // If Landing has been requested then say we have landed.

	} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
		   && distance_to_home < _param_nav_acc_rad.get() && _param_rtl_land_delay.get() > -FLT_EPSILON) {
		return true; // If the rover reaches the home position during RTL we say we have landed.

	} else {
		return !_armed;  // If we are armed we are not landed.
	}
}

void RoverLandDetector::_update_topics()
{
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = matrix::Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = matrix::Vector2d(home_position.lat, home_position.lon);
	}
}

void RoverLandDetector::_set_hysteresis_factor(const int factor)
{
	_landed_hysteresis.set_hysteresis_time_from(true, 0_ms);
}

} // namespace land_detector
