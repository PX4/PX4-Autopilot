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

#include <float.h>
#include <math.h>

#include <uORB/topics/position_setpoint.h>

/**
 * @brief State of the altitude first order hold (FOH), persisted across calls.
 *
 * A ramp is (re)started whenever the target (current waypoint) altitude changes. It always starts from the
 * current (measured) altitude at the vehicle's current horizontal distance to the target, hence the ramp
 * anchor fields have to survive between calls.
 */
struct FirstOrderHoldAltitudeState {
	float target_altitude{NAN};        ///< altitude setpoint currently ramped toward [m AMSL]
	float ramp_start_altitude{NAN};    ///< altitude the current ramp started from [m AMSL]
	float ramp_start_distance{NAN};    ///< horizontal distance to the target when the ramp started [m]
	float min_distance{FLT_MAX};       ///< closest horizontal approach to the target during this ramp [m]
};

/**
 * @brief Calculate the altitude setpoint using an altitude first order hold (FOH).
 *
 * Whenever the target (current waypoint) altitude changes, a new ramp is started from the current (measured)
 * vehicle altitude at the vehicle's present horizontal distance to the target. The setpoint is then linearly
 * interpolated along the remaining distance such that the new target altitude is reached at the acceptance
 * radius around the target. While the target altitude stays the same the ramp keeps progressing and is never
 * restarted, even if the position setpoint is otherwise updated.
 *
 * @param target_lat target (current waypoint) latitude [deg]
 * @param target_lon target (current waypoint) longitude [deg]
 * @param target_altitude target (current waypoint) altitude [m AMSL]
 * @param current_lat current vehicle latitude [deg]
 * @param current_lon current vehicle longitude [deg]
 * @param current_altitude current vehicle altitude [m AMSL]
 * @param acc_rad acceptance radius around the current waypoint [m]
 * @param state in/out FOH state, persisted across calls
 * @return altitude setpoint [m AMSL]
 */
float calculateFirstOrderHoldAltitude(const double target_lat, const double target_lon, const float target_altitude,
				      const double current_lat, const double current_lon, const float current_altitude,
				      const float acc_rad, FirstOrderHoldAltitudeState &state);
