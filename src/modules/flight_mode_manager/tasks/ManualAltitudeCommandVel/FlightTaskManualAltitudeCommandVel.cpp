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

#include "FlightTaskManualAltitudeCommandVel.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <geo/geo.h>

using namespace matrix;

FlightTaskManualAltitudeCommandVel::FlightTaskManualAltitudeCommandVel() :
	_sticks(this)
{}

bool FlightTaskManualAltitudeCommandVel::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sticks.checkAndUpdateStickInputs();

	if (_sticks_data_required) {
		ret = ret && _sticks.isAvailable();
	}

	// in addition to manual require valid position and velocity in D-direction and valid yaw
	return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
}

bool FlightTaskManualAltitudeCommandVel::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_yaw_setpoint = NAN;				// always use stick commands (or 0)
	_yawspeed_setpoint = 0.f;
	_acceleration_setpoint = Vector3f(0.f, 0.f, NAN); // altitude is controlled from velocity
	_last_position = _position;			// initialize loop to assume we're stable
	_position_setpoint(2) = NAN;
	_velocity_setpoint(2) = 0.f;
	_setDefaultConstraints();

	return ret;
}

void FlightTaskManualAltitudeCommandVel::_scaleSticks()
{
	// Use stick input with deadzone, exponential curve and first order lpf for yawspeed
	const float yawspeed_target = _sticks.getPositionExpo()(3) * math::radians(_param_mpc_man_y_max.get());
	_yawspeed_setpoint = _applyYawspeedFilter(yawspeed_target);

	// Use sticks input with deadzone and exponential curve for vertical velocity
	const float vel_max_z = (_sticks.getPosition()(2) > 0.0f) ? _constraints.speed_down : _constraints.speed_up;
	_velocity_setpoint(2) = vel_max_z * _sticks.getPositionExpo()(2);
}

float FlightTaskManualAltitudeCommandVel::_applyYawspeedFilter(float yawspeed_target)
{
	const float den = math::max(_param_mpc_man_y_tau.get() + _deltatime, 0.001f);
	const float alpha = _deltatime / den;
	_yawspeed_filter_state = (1.f - alpha) * _yawspeed_filter_state + alpha * yawspeed_target;
	return _yawspeed_filter_state;
}

void FlightTaskManualAltitudeCommandVel::_rotateIntoHeadingFrame(Vector2f &v)
{
	float yaw_rotate = PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : _yaw;
	Vector3f v_r = Vector3f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * Vector3f(v(0), v(1), 0.0f));
	v(0) = v_r(0);
	v(1) = v_r(1);
}

void FlightTaskManualAltitudeCommandVel::_updateSetpoints()
{
	// Thrust in xy are extracted directly from stick inputs. A magnitude of
	// 1 means that maximum thrust along xy is demanded. A magnitude of 0 means no
	// thrust along xy is demanded. The maximum thrust along xy depends on the thrust
	// setpoint along z-direction, which is computed in PositionControl.cpp.

	Vector2f sp(_sticks.getPosition().slice<2, 1>(0, 0));

	_man_input_filter.setParameters(_deltatime, _param_mc_man_tilt_tau.get());
	_man_input_filter.update(sp);
	sp = _man_input_filter.getState();
	_rotateIntoHeadingFrame(sp);

	if (sp.length() > 1.0f) {
		sp.normalize();
	}

	// constrain acceleration
	_acceleration_setpoint.xy() = sp * tanf(math::radians(_param_mpc_man_tilt_max.get())) * CONSTANTS_ONE_G;
	// position is never set
	_position_setpoint(2) = NAN;
}

bool FlightTaskManualAltitudeCommandVel::_checkTakeoff()
{
	// stick is deflected above 65% throttle (throttle stick is in the range [-1,1])
	return _sticks.getPosition()(2) < -0.3f;
}

bool FlightTaskManualAltitudeCommandVel::update()
{
	bool ret = FlightTask::update();
	_scaleSticks();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();

	// update position estimate
	_last_position = _position;

	return ret;
}
