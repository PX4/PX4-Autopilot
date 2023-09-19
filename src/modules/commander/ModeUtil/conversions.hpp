/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/events.h>
#include <uORB/topics/vehicle_status.h>

#include <stdint.h>

using navigation_mode_t = events::px4::enums::navigation_mode_t;

namespace mode_util
{

static inline navigation_mode_t navigation_mode(uint8_t nav_state)
{
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: return navigation_mode_t::manual;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL: return navigation_mode_t::altctl;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL: return navigation_mode_t::posctl;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION: return navigation_mode_t::auto_mission;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER: return navigation_mode_t::auto_loiter;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: return navigation_mode_t::auto_rtl;

	case vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW: return navigation_mode_t::position_slow;

	case vehicle_status_s::NAVIGATION_STATE_ACRO: return navigation_mode_t::acro;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: return navigation_mode_t::offboard;

	case vehicle_status_s::NAVIGATION_STATE_STAB: return navigation_mode_t::stab;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF: return navigation_mode_t::auto_takeoff;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND: return navigation_mode_t::auto_land;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET: return navigation_mode_t::auto_follow_target;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND: return navigation_mode_t::auto_precland;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT: return navigation_mode_t::orbit;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: return navigation_mode_t::auto_vtol_takeoff;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL1: return navigation_mode_t::external1;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL2: return navigation_mode_t::external2;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL3: return navigation_mode_t::external3;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL4: return navigation_mode_t::external4;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL5: return navigation_mode_t::external5;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL6: return navigation_mode_t::external6;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL7: return navigation_mode_t::external7;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL8: return navigation_mode_t::external8;
	}

	static_assert(vehicle_status_s::NAVIGATION_STATE_MAX == 31, "update navigation mode map");

	return navigation_mode_t::unknown;
}

} // namespace mode_util
