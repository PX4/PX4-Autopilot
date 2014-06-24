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

#include <navigator/navigator_state.h>

/**
 * @addtogroup topics @{
 */

/* main state machine */
typedef enum {
	MAIN_STATE_MANUAL = 0,
	MAIN_STATE_ALTCTL,
	MAIN_STATE_POSCTL,
	MAIN_STATE_AUTO,
	MAIN_STATE_ACRO,
	MAIN_STATE_MAX
} main_state_t;

// If you change the order, add or remove arming_state_t states make sure to update the arrays
// in state_machine_helper.cpp as well.
typedef enum {
	ARMING_STATE_INIT = 0,
	ARMING_STATE_STANDBY,
	ARMING_STATE_ARMED,
	ARMING_STATE_ARMED_ERROR,
	ARMING_STATE_STANDBY_ERROR,
	ARMING_STATE_REBOOT,
	ARMING_STATE_IN_AIR_RESTORE,
	ARMING_STATE_MAX
} arming_state_t;

typedef enum {
	HIL_STATE_OFF = 0,
	HIL_STATE_ON
} hil_state_t;

typedef enum {
	FAILSAFE_STATE_NORMAL = 0,		/**< Normal operation */
	FAILSAFE_STATE_RTL,				/**< Return To Launch */
	FAILSAFE_STATE_LAND,			/**< Land without position control */
	FAILSAFE_STATE_TERMINATION,		/**< Disable motors and use parachute, can't be recovered */
	FAILSAFE_STATE_MAX
} failsafe_state_t;

enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128,
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32,
	VEHICLE_MODE_FLAG_STABILIZED_ENABLED = 16,
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8,
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4,
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2,
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
}; /**< Same as MAV_MODE_FLAG of MAVLink 1.0 protocol */

/**
 * Should match 1:1 MAVLink's MAV_TYPE ENUM
 */
enum VEHICLE_TYPE {
	VEHICLE_TYPE_GENERIC = 0, /* Generic micro air vehicle. | */
	VEHICLE_TYPE_FIXED_WING = 1, /* Fixed wing aircraft. | */
	VEHICLE_TYPE_QUADROTOR = 2, /* Quadrotor | */
	VEHICLE_TYPE_COAXIAL = 3, /* Coaxial helicopter | */
	VEHICLE_TYPE_HELICOPTER = 4, /* Normal helicopter with tail rotor. | */
	VEHICLE_TYPE_ANTENNA_TRACKER = 5, /* Ground installation | */
	VEHICLE_TYPE_GCS = 6, /* Operator control unit / ground control station | */
	VEHICLE_TYPE_AIRSHIP = 7, /* Airship, controlled | */
	VEHICLE_TYPE_FREE_BALLOON = 8, /* Free balloon, uncontrolled | */
	VEHICLE_TYPE_ROCKET = 9, /* Rocket | */
	VEHICLE_TYPE_GROUND_ROVER = 10, /* Ground rover | */
	VEHICLE_TYPE_SURFACE_BOAT = 11, /* Surface vessel, boat, ship | */
	VEHICLE_TYPE_SUBMARINE = 12, /* Submarine | */
	VEHICLE_TYPE_HEXAROTOR = 13, /* Hexarotor | */
	VEHICLE_TYPE_OCTOROTOR = 14, /* Octorotor | */
	VEHICLE_TYPE_TRICOPTER = 15, /* Octorotor | */
	VEHICLE_TYPE_FLAPPING_WING = 16, /* Flapping wing | */
	VEHICLE_TYPE_KITE = 17, /* Kite | */
	VEHICLE_TYPE_ENUM_END = 18, /*  | */
};

enum VEHICLE_BATTERY_WARNING {
	VEHICLE_BATTERY_WARNING_NONE = 0,	/**< no battery low voltage warning active */
	VEHICLE_BATTERY_WARNING_LOW,	/**< warning of low voltage */
	VEHICLE_BATTERY_WARNING_CRITICAL	/**< alerting of critical voltage */
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * state machine / state of vehicle.
 *
 * Encodes the complete system state and is set by the commander app.
 */
struct vehicle_status_s {
	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter;   /**< incremented by the writing thread everytime new data is stored */
	uint64_t timestamp; /**< in microseconds since system start, is set whenever the writing thread stores new data */

	main_state_t main_state;				/**< main state machine */
	unsigned int set_nav_state;	/**< set navigation state machine to specified value */
	uint64_t set_nav_state_timestamp;	/**< timestamp of latest change of set_nav_state */
	arming_state_t arming_state;			/**< current arming state */
	hil_state_t hil_state;					/**< current hil state */
	failsafe_state_t failsafe_state;		/**< current failsafe state */

	int32_t system_type;				/**< system type, inspired by MAVLink's VEHICLE_TYPE enum */
	int32_t	system_id;				/**< system id, inspired by MAVLink's system ID field */
	int32_t component_id;				/**< subsystem / component id, inspired by MAVLink's component ID field */

	bool is_rotary_wing;

	bool condition_battery_voltage_valid;
	bool condition_system_in_air_restore;	/**< true if we can restore in mid air */
	bool condition_system_sensors_initialized;
	bool condition_system_returned_to_home;
	bool condition_auto_mission_available;
	bool condition_global_position_valid;		/**< set to true by the commander app if the quality of the gps signal is good enough to use it in the position estimator */
	bool condition_launch_position_valid;		/**< indicates a valid launch position */
	bool condition_home_position_valid;		/**< indicates a valid home position (a valid home position is not always a valid launch) */
	bool condition_local_position_valid;
	bool condition_local_altitude_valid;
	bool condition_airspeed_valid;			/**< set to true by the commander app if there is a valid airspeed measurement available */
	bool condition_landed;					/**< true if vehicle is landed, always true if disarmed */

	bool rc_signal_found_once;
	bool rc_signal_lost;				/**< true if RC reception lost */
	bool rc_input_blocked;				/**< set if RC input should be ignored */

	bool offboard_control_signal_found_once;
	bool offboard_control_signal_lost;
	bool offboard_control_signal_weak;
	uint64_t offboard_control_signal_lost_interval;	/**< interval in microseconds without an offboard control message */

	/* see SYS_STATUS mavlink message for the following */
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;

	float load;					/**< processor load from 0 to 1 */
	float battery_voltage;
	float battery_current;
	float battery_remaining;

	enum VEHICLE_BATTERY_WARNING battery_warning;    /**< current battery warning mode, as defined by VEHICLE_BATTERY_WARNING enum */
	uint16_t drop_rate_comm;
	uint16_t errors_comm;
	uint16_t errors_count1;
	uint16_t errors_count2;
	uint16_t errors_count3;
	uint16_t errors_count4;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_status);

#endif
