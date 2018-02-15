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
 * @file FlightManualStabilized.cpp
 */

#include "FlightTaskManualStabilized.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

FlightTaskManualStabilized::FlightTaskManualStabilized(control::SuperBlock *parent, const char *name) :
	FlightTaskManual(parent, name),
	_yaw_rate_scaling(parent, "MPC_MAN_Y_MAX", false),
	_tilt_max_man(parent, "MPC_MAN_TILT_MAX", false),
	_throttle_min(parent, "MPC_THR_MIN", false),
	_throttle_max(parent, "MPC_THR_MAX", false),
	_throttle_hover(parent, "MPC_THR_HOVER", false)
{}

bool FlightTaskManualStabilized::activate()
{
	_resetToNAN();
	_yaw_rate_sp = 0.0f;
	_thr_sp = matrix::Vector3f(0.0f, 0.0f, -_throttle_hover.get());
	return FlightTaskManual::activate();
}

void FlightTaskManualStabilized::_scaleSticks()
{
	/* Scale sticks to yaw and thrust using
	 * linear scale for yaw and piecewise linear map for thrust. */
	_yaw_rate_sp = _sticks(3) * math::radians(_yaw_rate_scaling.get());
	_throttle = _throttleCurve();
}

void FlightTaskManualStabilized::_updateHeadingSetpoints()
{
	/* Yaw-lock depends on stick input. If not locked,
	 * yaw_sp is set to NAN.
	 * TODO: add yawspeed to get threshold.*/
	const bool stick_yaw_zero = fabsf(_sticks(3)) <= _stick_dz.get();

	if (stick_yaw_zero && !PX4_ISFINITE(_yaw_sp)) {
		_yaw_sp = _yaw;

	} else if (!stick_yaw_zero) {
		_yaw_sp = NAN;
	}
}

void FlightTaskManualStabilized::_updateThrustSetpoints()
{
	/* Rotate setpoint into local frame. */
	matrix::Vector3f sp{_sticks(0), _sticks(1), 0.0f};
	sp = (matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, _yaw)) * sp);

	/* Ensure that maximum tilt is in [0.001, Pi] */
	float tilt_max = math::constrain(math::radians(_tilt_max_man.get()), 0.001f, M_PI_F);

	const float x = sp(0) * tilt_max;
	const float y = sp(1) * tilt_max;

	/* The norm of the xy stick input provides the pointing
	 * direction of the horizontal desired thrust setpoint. The magnitude of the
	 * xy stick inputs represents the desired tilt. Both tilt and magnitude can
	 * be captured through Axis-Angle.
	 */
	/* The Axis-Angle is the perpendicular vector to xy-stick input */
	matrix::Vector2f v = matrix::Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > tilt_max) { // limit to the configured maximum tilt angle
		v *= tilt_max / v_norm;
	}

	/* The final thrust setpoint is found by rotating the scaled unit vector pointing
	 * upward by the Axis-Angle.
	 */
	matrix::Quatf q_sp = matrix::AxisAnglef(v(0), v(1), 0.0f);
	_thr_sp = q_sp.conjugate(matrix::Vector3f(0.0f, 0.0f, -1.0f)) * _throttle;
}

void FlightTaskManualStabilized::_updateSetpoints()
{
	_updateHeadingSetpoints();
	_updateThrustSetpoints();
}

float FlightTaskManualStabilized::_throttleCurve()
{
	/* Scale stick z from [-1,1] to [min thrust, max thrust]
	 * with hover throttle at 0.5 stick */
	float throttle = -((_sticks(2) - 1.0f) * 0.5f);

	if (throttle < 0.5f) {
		return (_throttle_hover.get() - _throttle_min.get()) / 0.5f * throttle + _throttle_min.get();

	} else {
		return (_throttle_max.get() - _throttle_hover.get()) / 0.5f * (throttle - 1.0f) + _throttle_max.get();
	}
}

bool FlightTaskManualStabilized::update()
{
	_scaleSticks();
	_updateSetpoints();

	_setPositionSetpoint(_pos_sp);
	_setVelocitySetpoint(_vel_sp);
	_setThrustSetpoint(_thr_sp);
	_setYawSetpoint(_yaw_sp);
	_setYawspeedSetpoint(_yaw_rate_sp);

	return true;
}
