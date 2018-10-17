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
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPosition::updateInitialize()
{
	bool ret = FlightTaskManualAltitude::updateInitialize();
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskManualPosition::activate()
{

	// all requirements from altitude-mode still have to hold
	bool ret = FlightTaskManualAltitude::activate();

	// set task specific constraint
	if (_constraints.speed_xy >= MPC_VEL_MANUAL.get()) {
		_constraints.speed_xy = MPC_VEL_MANUAL.get();
	}

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
	_velocity_scale = _constraints.speed_xy;

	// for position-controlled mode, we need a valid position and velocity state
	// in NE-direction
	return ret;
}

void FlightTaskManualPosition::_scaleSticks()
{
	/* Use same scaling as for FlightTaskManualAltitude */
	FlightTaskManualAltitude::_scaleSticks();

	/* Constrain length of stick inputs to 1 for xy*/
	Vector2f stick_xy(&_sticks_expo(0));

	float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);

	if (mag > FLT_EPSILON) {
		stick_xy = stick_xy.normalized() * mag;
	}

	// scale the stick inputs
	if (PX4_ISFINITE(_sub_vehicle_local_position->get().vxy_max)) {
		// estimator provides vehicle specific max

		// use the minimum of the estimator and user specified limit
		_velocity_scale = fminf(_constraints.speed_xy, _sub_vehicle_local_position->get().vxy_max);
		// Allow for a minimum of 0.3 m/s for repositioning
		_velocity_scale = fmaxf(_velocity_scale, 0.3f);

	} else if (stick_xy.length() > 0.5f) {
		// raise the limit at a constant rate up to the user specified value

		if (_velocity_scale < _constraints.speed_xy) {
			_velocity_scale += _deltatime * MPC_ACC_HOR_ESTM.get();

		} else {
			_velocity_scale = _constraints.speed_xy;

		}
	}

	// scale velocity to its maximum limits
	Vector2f vel_sp_xy = stick_xy * _velocity_scale;

	/* Rotate setpoint into local frame. */
	_rotateIntoHeadingFrame(vel_sp_xy);

	/*constrain setpoint to not collide with obstacles */
	if(MPC_USE_OBS_SENS.get()){

		// calculate the maximum velocity along x,y axis when moving in the demanded direction
		float vel_mag = sqrt(vel_sp_xy(0) * vel_sp_xy(0) + vel_sp_xy(1) * vel_sp_xy(1));
		float v_max_x, v_max_y;
		if(vel_mag > 0){
			v_max_x = abs(_constraints.speed_xy/vel_mag * vel_sp_xy(0));
			v_max_y = abs(_constraints.speed_xy/vel_mag * vel_sp_xy(1));
		}else{
			v_max_x = 0.f;
			v_max_y = 0.f;
		}

		//scale the velocity reductions with the maximum possible velocity along the respective axis
		_constraints.velocity_limits[0] *= v_max_x;
		_constraints.velocity_limits[1] *= v_max_y;
		_constraints.velocity_limits[2] *= v_max_x;
		_constraints.velocity_limits[3] *= v_max_y;

		//apply the velocity reductions to form velocity limits
		_constraints.velocity_limits[0] = v_max_x - _constraints.velocity_limits[0];
		_constraints.velocity_limits[1] = v_max_y - _constraints.velocity_limits[1];
	    _constraints.velocity_limits[2] = v_max_x - _constraints.velocity_limits[2];
	    _constraints.velocity_limits[3] = v_max_y - _constraints.velocity_limits[3];

	    //constrain the velocity setpoint to respect the velocity limits
		vel_sp_xy(0) = math::constrain(vel_sp_xy(0), -_constraints.velocity_limits[2], _constraints.velocity_limits[0]);
		vel_sp_xy(1) = math::constrain(vel_sp_xy(1), -_constraints.velocity_limits[3], _constraints.velocity_limits[1]);
	}

	_velocity_setpoint(0) = vel_sp_xy(0);
	_velocity_setpoint(1) = vel_sp_xy(1);
}

void FlightTaskManualPosition::_updateXYlock()
{
	/* If position lock is not active, position setpoint is set to NAN.*/
	const float vel_xy_norm = Vector2f(_velocity).length();
	const bool apply_brake = Vector2f(_velocity_setpoint).length() < FLT_EPSILON;
	const bool stopped = (MPC_HOLD_MAX_XY.get() < FLT_EPSILON || vel_xy_norm < MPC_HOLD_MAX_XY.get());

	if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(0))) {
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);

	} else if (PX4_ISFINITE(_position_setpoint(0)) && apply_brake) {
		// Position is locked but check if a reset event has happened.
		// We will shift the setpoints.
		if (_sub_vehicle_local_position->get().xy_reset_counter != _reset_counter) {
			_position_setpoint(0) = _position(0);
			_position_setpoint(1) = _position(1);
			_reset_counter = _sub_vehicle_local_position->get().xy_reset_counter;
		}

	} else {
		/* don't lock*/
		_position_setpoint(0) = NAN;
		_position_setpoint(1) = NAN;
	}
}

void FlightTaskManualPosition::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_thrust_setpoint.setAll(NAN); // don't require any thrust setpoints
	_updateXYlock(); // check for position lock
}
