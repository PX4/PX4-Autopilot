/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 *           @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
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
 * @file vehicle_status.h
 * Definition of the vehicle_status uORB topic.
 *
 * Published the state machine and the system status bitfields
 * (see SYS_STATUS mavlink message), published only by commander app.
 *
 * All apps should write to subsystem_info:
 *
 *  (any app) --> subsystem_info (published) --> (commander app state machine)  --> vehicle_status --> (mavlink app)
 */

#ifndef VEHICLE_STATUS_H_
#define VEHICLE_STATUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics @{
 */

/* State Machine */
typedef enum
{
	SYSTEM_STATE_PREFLIGHT = 0,
	SYSTEM_STATE_STANDBY = 1,
	SYSTEM_STATE_GROUND_READY = 2,
	SYSTEM_STATE_MANUAL = 3,
	SYSTEM_STATE_STABILIZED = 4,
	SYSTEM_STATE_AUTO = 5,
	SYSTEM_STATE_MISSION_ABORT = 6,
	SYSTEM_STATE_EMCY_LANDING = 7,
	SYSTEM_STATE_EMCY_CUTOFF = 8,
	SYSTEM_STATE_GROUND_ERROR = 9,
	SYSTEM_STATE_REBOOT= 10,

} commander_state_machine_t;

enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128,
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32,
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16,
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8,
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4,
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2,
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
}; /**< Same as MAV_MODE_FLAG of MAVLink 1.0 protocol */

enum VEHICLE_FLIGHT_MODE {
	VEHICLE_FLIGHT_MODE_MANUAL,
	VEHICLE_FLIGHT_MODE_STABILIZED,
	VEHICLE_FLIGHT_MODE_AUTO
};

/**
 * state machine / state of vehicle.
 *
 * Encodes the complete system state and is set by the commander app.
 */
struct vehicle_status_s
{
	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter;   /**< incremented by the writing thread everytime new data is stored */
	uint64_t timestamp; /**< in microseconds since system start, is set whenever the writing thread stores new data */

	commander_state_machine_t state_machine;	/**< Current flight state, main state machine */
	enum VEHICLE_FLIGHT_MODE flight_mode;		/**< Current flight mode, as defined by mode switch */

	uint8_t mode;

	bool control_manual_enabled;			/**< true if manual input is mixed in */
	bool control_rates_enabled;			/**< true if rates are stabilized */
	bool control_attitude_enabled;			/**< true if attitude stabilization is mixed in */
	bool control_speed_enabled;			/**< true if speed (implies direction) is controlled */
	bool control_position_enabled;			/**< true if position is controlled */

	bool preflight_gyro_calibration;		/**< true if gyro calibration is requested */
	bool preflight_mag_calibration;			/**< true if mag calibration is requested */

	bool rc_signal_lost;				/**< true if RC reception is terminally lost */
	bool rc_signal_cutting_off;			/**< true if RC reception is weak / cutting off */
	uint64_t rc_signal_lost_interval;		/**< interval in microseconds since when no RC signal is available */

	/* see SYS_STATUS mavlink message for the following */
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;
	uint16_t load;
	uint16_t voltage_battery;
	int16_t current_battery;
	int8_t battery_remaining;
	uint16_t drop_rate_comm;
	uint16_t errors_comm;
	uint16_t errors_count1;
	uint16_t errors_count2;
	uint16_t errors_count3;
	uint16_t errors_count4;

//	bool remote_manual; /**< set to true by the commander when the manual-switch on the remote is set to manual */
	bool gps_valid;     /**< set to true by the commander app if the quality of the gps signal is good enough to use it in the position estimator */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_status);

#endif
