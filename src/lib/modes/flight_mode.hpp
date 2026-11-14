/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <uORB/topics/vehicle_status.h>

#include <stdint.h>

namespace mode_util
{

/**
 * @return nav_state for a COM_FLTMODEn parameter value, or -1 if unassigned/unknown.
 *
 * See COM_FLTMODE${i} in src/modules/commander/module.yaml
 */
static inline int8_t navStateFromFlightModeParam(int32_t param_value)
{
	switch (param_value) {
	case 0: return vehicle_status_s::NAVIGATION_STATE_MANUAL;

	case 1: return vehicle_status_s::NAVIGATION_STATE_ALTCTL;

	case 2: return vehicle_status_s::NAVIGATION_STATE_POSCTL;

	case 3: return vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

	case 4: return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	case 5: return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

	case 6: return vehicle_status_s::NAVIGATION_STATE_ACRO;

	case 7: return vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

	case 8: return vehicle_status_s::NAVIGATION_STATE_STAB;

	case 9: return vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW;

	case 10: return vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;

	case 11: return vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

	case 12: return vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;

	case 13: return vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;

	case 14: return vehicle_status_s::NAVIGATION_STATE_ORBIT;

	case 15: return vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF;

	case 16: return vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE;

	case 17: return vehicle_status_s::NAVIGATION_STATE_MANUAL_PARKING;

	case 100: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;

	case 101: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL2;

	case 102: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL3;

	case 103: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL4;

	case 104: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL5;

	case 105: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL6;

	case 106: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL7;

	case 107: return vehicle_status_s::NAVIGATION_STATE_EXTERNAL8;
	}

	return -1;
}

} // namespace mode_util
