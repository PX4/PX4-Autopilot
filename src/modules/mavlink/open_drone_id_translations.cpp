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

#include "open_drone_id_translations.hpp"
#include <uORB/topics/vehicle_status.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;


namespace open_drone_id_translations
{

MAV_ODID_UA_TYPE odidTypeForMavType(uint8_t system_type)
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

MAV_ODID_SPEED_ACC odidSpeedAccForVariance(float s_variance_m_s)
{
	// TODO: should this be stddev, so square root of variance?
	// speed_accuracy
	if (s_variance_m_s < 0.3f) {
		return MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND;

	} else if (s_variance_m_s < 1.f) {
		return MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND;

	} else if (s_variance_m_s < 3.f) {
		return MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND;

	} else if (s_variance_m_s < 10.f) {
		return MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND;

	} else {
		return MAV_ODID_SPEED_ACC_UNKNOWN;
	}
}

MAV_ODID_HOR_ACC odidHorAccForEph(float eph)
{
	if (eph < 1.f) {
		return MAV_ODID_HOR_ACC_1_METER;

	} else if (eph < 3.f) {
		return MAV_ODID_HOR_ACC_3_METER;

	} else if (eph < 10.f) {
		return MAV_ODID_HOR_ACC_10_METER;

	} else if (eph < 30.f) {
		return MAV_ODID_HOR_ACC_30_METER;

	} else {
		return MAV_ODID_HOR_ACC_UNKNOWN;
	}
}

MAV_ODID_VER_ACC odidVerAccForEpv(float epv)
{
	if (epv < 1.f) {
		return MAV_ODID_VER_ACC_1_METER;

	} else if (epv < 3.f) {
		return MAV_ODID_VER_ACC_3_METER;

	} else if (epv < 10.f) {
		return MAV_ODID_VER_ACC_10_METER;

	} else if (epv < 25.f) {
		return MAV_ODID_VER_ACC_25_METER;

	} else if (epv < 45.f) {
		return MAV_ODID_VER_ACC_45_METER;

	} else if (epv < 150.f) {
		return MAV_ODID_VER_ACC_150_METER;

	} else {
		return MAV_ODID_VER_ACC_UNKNOWN;
	}
}


MAV_ODID_TIME_ACC odidTimeForElapsed(uint64_t elapsed)
{
	if (elapsed < 100_ms) {
		return MAV_ODID_TIME_ACC_0_1_SECOND;

	} else if (elapsed < 200_ms) {
		return MAV_ODID_TIME_ACC_0_2_SECOND;

	} else if (elapsed < 300_ms) {
		return MAV_ODID_TIME_ACC_0_3_SECOND;

	} else if (elapsed < 400_ms) {
		return MAV_ODID_TIME_ACC_0_4_SECOND;

	} else if (elapsed < 500_ms) {
		return MAV_ODID_TIME_ACC_0_5_SECOND;

	} else if (elapsed < 600_ms) {
		return MAV_ODID_TIME_ACC_0_6_SECOND;

	} else if (elapsed < 700_ms) {
		return MAV_ODID_TIME_ACC_0_7_SECOND;

	} else if (elapsed < 800_ms) {
		return MAV_ODID_TIME_ACC_0_8_SECOND;

	} else if (elapsed < 900_ms) {
		return MAV_ODID_TIME_ACC_0_9_SECOND;

	} else if (elapsed < 1000_ms) {
		return MAV_ODID_TIME_ACC_1_0_SECOND;

	} else if (elapsed < 1100_ms) {
		return MAV_ODID_TIME_ACC_1_1_SECOND;

	} else if (elapsed < 1200_ms) {
		return MAV_ODID_TIME_ACC_1_2_SECOND;

	} else if (elapsed < 1300_ms) {
		return MAV_ODID_TIME_ACC_1_3_SECOND;

	} else if (elapsed < 1400_ms) {
		return MAV_ODID_TIME_ACC_1_4_SECOND;

	} else if (elapsed < 1500_ms) {
		return MAV_ODID_TIME_ACC_1_5_SECOND;

	} else {
		return MAV_ODID_TIME_ACC_UNKNOWN;
	}
}

} // open_drone_id_translations
