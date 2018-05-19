/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightManualAltitude.cpp
 */

#include "FlightTaskManualAltitude.hpp"
#include <float.h>

using namespace matrix;

void FlightTaskManualAltitude::_scaleSticks()
{
	/* Reuse same scaling as for stabilized */
	FlightTaskManualStabilized::_scaleSticks();

	/* Scale horizontal velocity with expo curve stick input*/
	const float vel_max_z = (_sticks(2) > 0.0f) ? _vel_max_down.get() : _vel_max_up.get();
	_velocity_setpoint(2) = vel_max_z * _sticks_expo(2);
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	/* Depending on stick inputs and velocity, position is locked.
	 * If not locked, altitude setpoint is set to NAN.
	 */

	/* handle position and altitude hold */
	const bool apply_brake_z = fabsf(_velocity_setpoint(2)) <= FLT_EPSILON;
	const bool stopped_z = (_vel_hold_thr_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _vel_hold_thr_z.get());

	if (apply_brake_z && stopped_z && !PX4_ISFINITE(_position_setpoint(2))) {
		_position_setpoint(2) = _position(2);

	} else if (!apply_brake_z) {
		_position_setpoint(2) = NAN;
	}
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	FlightTaskManualStabilized::_updateSetpoints(); // get yaw and thrust setpoints

	_thrust_setpoint *= NAN; // Don't need thrust setpoint from Stabilized mode.

	/* Thrust in xy are extracted directly from stick inputs. A magnitude of
	 * 1 means that maximum thrust along xy is required. A magnitude of 0 means no
	 * thrust along xy is required. The maximum thrust along xy depends on the thrust
	 * setpoint along z-direction, which is computed in PositionControl.cpp.
	 */
	Vector2f sp{_sticks(0), _sticks(1)};
	_rotateIntoHeadingFrame(sp);

	if (sp.length() > 1.0f) {
		sp.normalize();
	}

	_thrust_setpoint(0) = sp(0);
	_thrust_setpoint(1) = sp(1);

	_updateAltitudeLock();
}
