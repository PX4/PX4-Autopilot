/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vehicle_control_mode.h
 * Definition of the vehicle_control_mode uORB topic.
 *
 * All control apps should depend their actions based on the flags set here.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#ifndef VEHICLE_CONTROL_MODE
#define VEHICLE_CONTROL_MODE

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"
#include "vehicle_status.h"

/**
 * @addtogroup topics @{
 */


/**
 * state machine / state of vehicle.
 *
 * Encodes the complete system state and is set by the commander app.
 */

struct vehicle_control_mode_s {
	uint64_t timestamp; /**< in microseconds since system start, is set whenever the writing thread stores new data */

	bool flag_armed;

	bool flag_external_manual_override_ok;	/**< external override non-fatal for system. Only true for fixed wing */

	// XXX needs yet to be set by state machine helper
	bool flag_system_hil_enabled;

	bool flag_control_manual_enabled;		/**< true if manual input is mixed in */
	bool flag_control_auto_enabled;			/**< true if onboard autopilot should act */
	bool flag_control_offboard_enabled;		/**< true if offboard control should be used */
	bool flag_control_rates_enabled;		/**< true if rates are stabilized */
	bool flag_control_attitude_enabled;		/**< true if attitude stabilization is mixed in */
	bool flag_control_force_enabled;		/**< true if force control is mixed in */
	bool flag_control_velocity_enabled;		/**< true if horizontal velocity (implies direction) is controlled */
	bool flag_control_position_enabled;		/**< true if position is controlled */
	bool flag_control_altitude_enabled;		/**< true if altitude is controlled */
	bool flag_control_climb_rate_enabled;	/**< true if climb rate is controlled */
	bool flag_control_termination_enabled;	/**< true if flighttermination is enabled */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_control_mode);

#endif
