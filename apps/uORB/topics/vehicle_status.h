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
	VEHICLE_FLIGHT_MODE_MANUAL = 0,	/**< direct manual control, same as VEHICLE_FLIGHT_MODE_ATTITUDE for multirotors */
	VEHICLE_FLIGHT_MODE_ATTITUDE,	/**< attitude or rate stabilization, as defined by VEHICLE_ATTITUDE_MODE */
	VEHICLE_FLIGHT_MODE_STABILIZED,	/**< attitude or rate stabilization plus velocity or position stabilization */
	VEHICLE_FLIGHT_MODE_AUTO	/**< attitude or rate stabilization plus absolute position control and waypoints */
};

enum VEHICLE_ATTITUDE_MODE {
	VEHICLE_ATTITUDE_MODE_DIRECT,	/**< no attitude control, direct stick input mixing (only fixed wing) */
	VEHICLE_ATTITUDE_MODE_RATES,	/**< body rates control mode */
	VEHICLE_ATTITUDE_MODE_ATTITUDE	/**< tait-bryan attitude control mode */
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
	enum VEHICLE_ATTITUDE_MODE attitute_mode;	/**< current attitude control mode, as defined by VEHICLE_ATTITUDE_MODE enum */

	// uint8_t mode;


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
	bool flag_preflight_mag_calibration;			/**< true if mag calibration is requested */
	bool flag_preflight_accel_calibration;

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
