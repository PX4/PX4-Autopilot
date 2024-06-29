/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#include <mavlink/common/mavlink.h>

namespace open_drone_id_translations
{

static inline MAV_ODID_UA_TYPE odidTypeForMavType(uint8_t system_type)
{
	switch (system_type) {
	case MAV_TYPE_GENERIC: return MAV_ODID_UA_TYPE_OTHER;

	case MAV_TYPE_FIXED_WING: return MAV_ODID_UA_TYPE_AEROPLANE;

	case MAV_TYPE_QUADROTOR: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_COAXIAL: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_HELICOPTER: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_ANTENNA_TRACKER: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_GCS: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_AIRSHIP: return MAV_ODID_UA_TYPE_AIRSHIP;

	case MAV_TYPE_FREE_BALLOON: return MAV_ODID_UA_TYPE_FREE_BALLOON;

	case MAV_TYPE_ROCKET: return MAV_ODID_UA_TYPE_ROCKET;

	case MAV_TYPE_GROUND_ROVER: return MAV_ODID_UA_TYPE_GROUND_OBSTACLE;

	case MAV_TYPE_SURFACE_BOAT: return MAV_ODID_UA_TYPE_OTHER;

	case MAV_TYPE_SUBMARINE: return MAV_ODID_UA_TYPE_OTHER;

	case MAV_TYPE_HEXAROTOR: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_OCTOROTOR: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_TRICOPTER: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_FLAPPING_WING: return MAV_ODID_UA_TYPE_ORNITHOPTER;

	case MAV_TYPE_KITE: return MAV_ODID_UA_TYPE_KITE;

	case MAV_TYPE_ONBOARD_CONTROLLER: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_VTOL_TAILSITTER_DUOROTOR: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_VTOL_TAILSITTER_QUADROTOR: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_VTOL_TILTROTOR: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_VTOL_FIXEDROTOR: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_VTOL_TAILSITTER: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_VTOL_TILTWING: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_VTOL_RESERVED5: return MAV_ODID_UA_TYPE_HYBRID_LIFT;

	case MAV_TYPE_GIMBAL: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_ADSB: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_PARAFOIL: return MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE;

	case MAV_TYPE_DODECAROTOR: return MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;

	case MAV_TYPE_CAMERA: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_CHARGING_STATION: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_FLARM: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_SERVO: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_ODID: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_DECAROTOR: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_BATTERY: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_PARACHUTE: return MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE;

	case MAV_TYPE_LOG: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_OSD: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_IMU: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_GPS: return MAV_ODID_UA_TYPE_NONE;

	case MAV_TYPE_WINCH: return MAV_ODID_UA_TYPE_NONE;

	default: return MAV_ODID_UA_TYPE_OTHER;
	}
}


} // open_drone_id_translations
