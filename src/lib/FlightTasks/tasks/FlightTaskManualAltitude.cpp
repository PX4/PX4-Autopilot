/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

bool FlightTaskManualAltitude::updateInitialize()
{
	bool ret = FlightTaskManualStabilized::updateInitialize();
	// in addition to stabilized require valid position and velocity in D-direction
	return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2));
}

bool FlightTaskManualAltitude::activate()
{
	bool ret = FlightTaskManualStabilized::activate();
	_thrust_setpoint(2) = NAN; // altitude is controlled from position/velocity
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.0f;
	_setDefaultConstraints();
	_constraints.min_distance_to_ground = SENS_FLOW_MINRNG.get();
	return ret;
}

void FlightTaskManualAltitude::_scaleSticks()
{
	// reuse same scaling as for stabilized
	FlightTaskManualStabilized::_scaleSticks();

	// scale horizontal velocity with expo curve stick input
	const float vel_max_z = (_sticks(2) > 0.0f) ? _constraints.speed_down : _constraints.speed_up;
	_velocity_setpoint(2) = vel_max_z * _sticks_expo(2);
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	// Depending on stick inputs and velocity, position is locked.
	// If not locked, altitude setpoint is set to NAN.

	// check if user wants to break
	const bool apply_brake = fabsf(_velocity_setpoint(2)) <= FLT_EPSILON;

	// check if vehicle has stopped
	const bool stopped = (MPC_HOLD_MAX_Z.get() < FLT_EPSILON || fabsf(_velocity(2)) < MPC_HOLD_MAX_Z.get());

	if (MPC_ALT_MODE.get() && PX4_ISFINITE(_dist_to_bottom)) {
		// terrain following
		_terrain_following(apply_brake, stopped);

	} else {
		// altitude based on locale coordinate system
		_dist_to_ground_lock = NAN; // reset since not used

		if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2))) {
			// lock position
			_position_setpoint(2) = _position(2);
			// ensure that minimum altitude is respected
			_respectMinAltitude();

		} else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake) {
			// Position is locked but check if a reset event has happened.
			// We will shift the setpoints.
			if (_sub_vehicle_local_position->get().z_reset_counter != _reset_counter) {
				_position_setpoint(2) = _position(2);
				_reset_counter = _sub_vehicle_local_position->get().z_reset_counter;
			}

		} else  {
			// user demands velocity change
			_position_setpoint(2) = NAN;
		}
	}
}

void FlightTaskManualAltitude::_respectMinAltitude()
{

	const bool respectAlt = _sub_vehicle_local_position->get().limit_hagl
				&& PX4_ISFINITE(_dist_to_bottom)
				&& _dist_to_bottom < SENS_FLOW_MINRNG.get();

	// Height above ground needs to be limited (flow / range-finder)
	if (respectAlt) {
		// increase altitude to minimum flow distance
		_position_setpoint(2) = _position(2)
					- (SENS_FLOW_MINRNG.get() - _dist_to_bottom);
	}
}

void FlightTaskManualAltitude::_terrain_following(bool apply_brake, bool stopped)
{

	if (apply_brake && stopped && !PX4_ISFINITE(_dist_to_ground_lock)) {
		// User wants to break and vehicle reached zero velocity. Lock height to ground.

		// lock position
		_position_setpoint(2) = _position(2);
		// ensure that minimum altitude is respected
		_respectMinAltitude();
		// lock distance to ground but adjust first for minimum altitude
		_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));

	} else if (apply_brake && PX4_ISFINITE(_dist_to_ground_lock)) {
		// vehicle needs to follow terrain

		// difference between the current distance to ground and the desired distance to ground
		const float delta_distance_to_ground = _dist_to_ground_lock - _dist_to_bottom;
		// adjust position setpoint for the delta (note: NED frame)
		_position_setpoint(2) = _position(2) - delta_distance_to_ground;

	} else {
		// user demands velocity change in D-direction
		_dist_to_ground_lock = _position_setpoint(2) = NAN;
	}
}
void FlightTaskManualAltitude::_updateSetpoints()
{
	FlightTaskManualStabilized::_updateSetpoints(); // get yaw and thrust setpoints

	_thrust_setpoint *= NAN; // Don't need thrust setpoint from Stabilized mode.

	// Thrust in xy are extracted directly from stick inputs. A magnitude of
	// 1 means that maximum thrust along xy is demanded. A magnitude of 0 means no
	// thrust along xy is demanded. The maximum thrust along xy depends on the thrust
	// setpoint along z-direction, which is computed in PositionControl.cpp.

	Vector2f sp{_sticks(0), _sticks(1)};
	_rotateIntoHeadingFrame(sp);

	if (sp.length() > 1.0f) {
		sp.normalize();
	}

	_thrust_setpoint(0) = sp(0);
	_thrust_setpoint(1) = sp(1);

	_updateAltitudeLock();
}
