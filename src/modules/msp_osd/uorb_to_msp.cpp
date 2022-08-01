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

/* uorb_to_msp.cpp
 *
 * Implementation file for UORB -> MSP conversion functions.
 */

#include "uorb_to_msp.hpp"

namespace msp_osd {

msp_status_BF_t construct_STATUS(const vehicle_status_s& vehicle_status) {

	// initialize result
	msp_status_BF_t status_BF = {0};

	if (vehicle_status.arming_state == vehicle_status.ARMING_STATE_ARMED) {
		status_BF.flight_mode_flags |= ARM_ACRO_BF;

		switch (vehicle_status.nav_state) {
		case vehicle_status.NAVIGATION_STATE_MANUAL:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status.NAVIGATION_STATE_ACRO:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status.NAVIGATION_STATE_STAB:
			status_BF.flight_mode_flags |= STAB_BF;
			break;

		case vehicle_status.NAVIGATION_STATE_AUTO_RTL:
			status_BF.flight_mode_flags |= RESC_BF;
			break;

		case vehicle_status.NAVIGATION_STATE_TERMINATION:
			status_BF.flight_mode_flags |= FS_BF;
			break;

		default:
			status_BF.flight_mode_flags = 0;
			break;
		}
	}

	status_BF.arming_disable_flags_count = 1;
	status_BF.arming_disable_flags  = !(vehicle_status.arming_state == vehicle_status.ARMING_STATE_ARMED);
	return status_BF;
}

msp_analog_t construct_ANALOG(const battery_status_s& battery_status, const input_rc_s& input_rc) {

	// initialize result
	msp_analog_t analog {0};

	analog.vbat = battery_status.voltage_v * 10; // bottom right... v * 10
	analog.rssi = (uint16_t)((input_rc.rssi * 1023.0f) / 100.0f);
	analog.amperage = battery_status.current_a * 100; // main amperage
	analog.mAhDrawn = battery_status.discharged_mah; // unused

	return analog;
}

msp_battery_state_t construct_BATTERY_STATE(const battery_status_s& battery_status) {

	// initialize result
	msp_battery_state_t battery_state = {0};

	// MSP_BATTERY_STATE
	battery_state.amperage = battery_status.current_a; // not used?
	battery_state.batteryVoltage = (uint16_t)(battery_status.voltage_v * 400.0f);  // OK
	battery_state.mAhDrawn = battery_status.discharged_mah ; // OK
	battery_state.batteryCellCount = battery_status.cell_count;
	battery_state.batteryCapacity = battery_status.capacity; // not used?

	// Voltage color 0==white, 1==red
	if (battery_status.voltage_v < 14.4f) {
		battery_state.batteryState = 1;

	} else {
		battery_state.batteryState = 0;
	}

	battery_state.legacyBatteryVoltage = battery_status.voltage_v * 10;
	return battery_state;
}

msp_raw_gps_t construct_RAW_GPS(const struct vehicle_gps_position_s& vehicle_gps_position,
				const struct airspeed_validated_s& airspeed_validated) {

	// initialize result
	msp_raw_gps_t raw_gps {0};

	if (vehicle_gps_position.fix_type >= 2) {
		raw_gps.lat = vehicle_gps_position.lat;
		raw_gps.lon = vehicle_gps_position.lon;
		raw_gps.alt =  vehicle_gps_position.alt / 10;
		//raw_gps.groundCourse = vehicle_gps_position_struct

	} else {
		raw_gps.lat = 0;
		raw_gps.lon = 0;
		raw_gps.alt = 0;
	}

	if (vehicle_gps_position.fix_type == 0
	    || vehicle_gps_position.fix_type == 1) {
		raw_gps.fixType = MSP_GPS_NO_FIX;

	} else if (vehicle_gps_position.fix_type == 2) {
		raw_gps.fixType = MSP_GPS_FIX_2D;

	} else if (vehicle_gps_position.fix_type >= 3 && vehicle_gps_position.fix_type <= 5) {
		raw_gps.fixType = MSP_GPS_FIX_3D;

	} else {
		raw_gps.fixType = MSP_GPS_NO_FIX;
	}

	//raw_gps.hdop = vehicle_gps_position_struct.hdop
	raw_gps.numSat = vehicle_gps_position.satellites_used;

	if (airspeed_validated.airspeed_sensor_measurement_valid
	    && airspeed_validated.indicated_airspeed_m_s != NAN
	    && airspeed_validated.indicated_airspeed_m_s > 0) {
		raw_gps.groundSpeed = airspeed_validated.indicated_airspeed_m_s * 100;

	} else {
		raw_gps.groundSpeed = 0;
	}

	return raw_gps;
}

msp_altitude_t construct_ALTITUDE(const struct vehicle_gps_position_s& vehicle_gps_position,
				  const struct estimator_status_s& estimator_status,
				  const struct vehicle_local_position_s& vehicle_local_position) {

	// initialize result
	msp_altitude_t altitude {0};

	if (vehicle_gps_position.fix_type >= 2) {
		altitude.estimatedActualPosition = vehicle_gps_position.alt / 10;

	} else {
		altitude.estimatedActualPosition = 0;
	}

	if (estimator_status.solution_status_flags & (1 << 5)) {
		altitude.estimatedActualVelocity = -vehicle_local_position.vz * 10; //m/s to cm/s

	} else {
		altitude.estimatedActualVelocity = 0;
	}

	return altitude;
}

} // namespace msp_osd
