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
	VEHICLE_MODE_FLAG_STABILIZED_ENABLED = 16,
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8,
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4,
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2,
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
}; /**< Same as MAV_MODE_FLAG of MAVLink 1.0 protocol */

enum VEHICLE_FLIGHT_MODE {
	VEHICLE_FLIGHT_MODE_MANUAL = 0,		/**< direct manual control, exact mode determined by VEHICLE_MANUAL_CONTROL_MODE */
	VEHICLE_FLIGHT_MODE_STAB,		/**< attitude or rate stabilization plus velocity or position stabilization */
	VEHICLE_FLIGHT_MODE_HOLD,		/**< hold current position (hover or loiter around position when switched) */
	VEHICLE_FLIGHT_MODE_AUTO		/**< attitude or rate stabilization plus absolute position control and waypoints */
};

enum VEHICLE_MANUAL_CONTROL_MODE {
	VEHICLE_MANUAL_CONTROL_MODE_DIRECT = 0,	/**< no attitude control, direct stick input mixing (only fixed wing) */
	VEHICLE_MANUAL_CONTROL_MODE_RATES,	/**< body rates control mode */
	VEHICLE_MANUAL_CONTROL_MODE_SAS		/**< stability augmented system (SAS) mode */
};

enum VEHICLE_MANUAL_SAS_MODE {
	VEHICLE_MANUAL_SAS_MODE_ROLL_PITCH_ABS_YAW_ABS = 0,	/**< roll, pitch and yaw absolute */
	VEHICLE_MANUAL_SAS_MODE_ROLL_PITCH_ABS_YAW_RATE,	/**< roll and pitch absolute, yaw rate */
	VEHICLE_MANUAL_SAS_MODE_SIMPLE,				/**< simple mode (includes altitude hold) */
	VEHICLE_MANUAL_SAS_MODE_ALTITUDE			/**< altitude hold */
};

/**
 * Should match 1:1 MAVLink's MAV_TYPE ENUM
 */
enum VEHICLE_TYPE {
	VEHICLE_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
	VEHICLE_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
	VEHICLE_TYPE_QUADROTOR=2, /* Quadrotor | */
	VEHICLE_TYPE_COAXIAL=3, /* Coaxial helicopter | */
	VEHICLE_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
	VEHICLE_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
	VEHICLE_TYPE_GCS=6, /* Operator control unit / ground control station | */
	VEHICLE_TYPE_AIRSHIP=7, /* Airship, controlled | */
	VEHICLE_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
	VEHICLE_TYPE_ROCKET=9, /* Rocket | */
	VEHICLE_TYPE_GROUND_ROVER=10, /* Ground rover | */
	VEHICLE_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
	VEHICLE_TYPE_SUBMARINE=12, /* Submarine | */
	VEHICLE_TYPE_HEXAROTOR=13, /* Hexarotor | */
	VEHICLE_TYPE_OCTOROTOR=14, /* Octorotor | */
	VEHICLE_TYPE_TRICOPTER=15, /* Octorotor | */
	VEHICLE_TYPE_FLAPPING_WING=16, /* Flapping wing | */
	VEHICLE_TYPE_KITE=17, /* Kite | */
	VEHICLE_TYPE_ENUM_END=18, /*  | */
};

enum VEHICLE_BATTERY_WARNING {
    VEHICLE_BATTERY_WARNING_NONE = 0,    /**< no battery low voltage warning active */
    VEHICLE_BATTERY_WARNING_WARNING,        /**< warning of low voltage 1. stage */
    VEHICLE_BATTERY_WARNING_ALERT            /**< aleting of low voltage 2. stage */
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
	uint64_t failsave_lowlevel_start_time;		/**< time when the lowlevel failsafe flag was set */
	//uint64_t failsave_highlevel_begin; TO BE COMPLETED

	commander_state_machine_t state_machine;	/**< current flight state, main state machine */
	enum VEHICLE_FLIGHT_MODE flight_mode;		/**< current flight mode, as defined by mode switch */
	enum VEHICLE_MANUAL_CONTROL_MODE manual_control_mode;	/**< current attitude control mode, as defined by VEHICLE_ATTITUDE_MODE enum */
	enum VEHICLE_MANUAL_SAS_MODE	manual_sas_mode;	/**< current stabilization mode */
	int32_t system_type;				/**< system type, inspired by MAVLink's VEHICLE_TYPE enum */
	int32_t	system_id;				/**< system id, inspired by MAVLink's system ID field */
	int32_t component_id;				/**< subsystem / component id, inspired by MAVLink's component ID field */

	/* system flags - these represent the state predicates */

	bool flag_system_armed;				/**< true is motors / actuators are armed */
	bool flag_control_manual_enabled;		/**< true if manual input is mixed in */
	bool flag_control_offboard_enabled;		/**< true if offboard control input is on */
	bool flag_hil_enabled;				/**< true if hardware in the loop simulation is enabled */

	bool flag_control_rates_enabled;		/**< true if rates are stabilized */
	bool flag_control_attitude_enabled;		/**< true if attitude stabilization is mixed in */
	bool flag_control_velocity_enabled;		/**< true if speed (implies direction) is controlled */
	bool flag_control_position_enabled;		/**< true if position is controlled */

	bool flag_preflight_gyro_calibration;		/**< true if gyro calibration is requested */
	bool flag_preflight_mag_calibration;		/**< true if mag calibration is requested */
	bool flag_preflight_accel_calibration;
	bool flag_preflight_airspeed_calibration;

	bool rc_signal_found_once;
	bool rc_signal_lost;				/**< true if RC reception is terminally lost */
	bool rc_signal_cutting_off;			/**< true if RC reception is weak / cutting off */
	uint64_t rc_signal_lost_interval;		/**< interval in microseconds since when no RC signal is available */

	bool offboard_control_signal_found_once;
	bool offboard_control_signal_lost;
	bool offboard_control_signal_weak;
	uint64_t offboard_control_signal_lost_interval;	/**< interval in microseconds without an offboard control message */

	bool failsave_lowlevel;				/**< Set to true if low-level failsafe mode is enabled */
	//bool failsave_highlevel;

	/* see SYS_STATUS mavlink message for the following */
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;
	float load;
	float voltage_battery;
	float current_battery;
	float battery_remaining;
	enum VEHICLE_BATTERY_WARNING battery_warning;    /**< current battery warning mode, as defined by VEHICLE_BATTERY_WARNING enum */
	uint16_t drop_rate_comm;
	uint16_t errors_comm;
	uint16_t errors_count1;
	uint16_t errors_count2;
	uint16_t errors_count3;
	uint16_t errors_count4;

	bool flag_global_position_valid;		/**< set to true by the commander app if the quality of the gps signal is good enough to use it in the position estimator */
	bool flag_local_position_valid;
	bool flag_vector_flight_mode_ok;		/**< position estimation, battery voltage and other critical subsystems are good for autonomous flight */
	bool flag_auto_flight_mode_ok;			/**< conditions of vector flight mode apply plus a valid takeoff position lock has been aquired */
	bool flag_external_manual_override_ok;		/**< external override non-fatal for system. Only true for fixed wing */
	bool flag_valid_launch_position;		/**< indicates a valid launch position */
	bool flag_valid_home_position;			/**< indicates a valid home position (a valid home position is not always a valid launch) */
	bool flag_airspeed_valid;			/**< set to true by the commander app if there is a valid airspeed measurement available */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_status);

#endif
