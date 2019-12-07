/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionLock.hpp
 *
 * Library for position and yaw lock logic when flying by stick.
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include "SlewRate.hpp"

class PositionLock
{
public:
	PositionLock() = default;
	~PositionLock() = default;

	/**
	 * Lock yaw when not currently turning
	 * When applying a yawspeed the vehicle is turning, when the speed is
	 * set to zero the vehicle needs to slow down and then lock at the yaw
	 * it stops at to not drift over time.
	 * @param yaw_stick_input current yaw rotational rate state
	 * @return yaw setpoint to execute to have a yaw lock at the correct moment in time
	 */
	void updateYawFromStick(float &yawspeed_setpoint, float &yaw_setpoint, const float desired_yawspeed, const float yaw,
				const float deltatime)
	{
		_yawspeed_slew_rate.setSlewRate(2.f * M_PI_F);
		yawspeed_setpoint = _yawspeed_slew_rate.update(desired_yawspeed, deltatime);
		yaw_setpoint = updateYawLock(yaw, yawspeed_setpoint, yaw_setpoint);
	}

	/**
	 * Lock yaw when not currently turning
	 * When applying a yawspeed the vehicle is turning, when the speed is
	 * set to zero the vehicle needs to slow down and then lock at the yaw
	 * it stops at to not drift over time.
	 * @param yawspeed current yaw rotational rate state
	 * @param yaw current yaw rotational rate state
	 * @param yawspeed_setpoint rotation rate at which to turn around yaw axis
	 * @param yaw current yaw setpoint which then will be overwritten by the return value
	 * @return yaw setpoint to execute to have a yaw lock at the correct moment in time
	 */
	static float updateYawLock(const float yaw, const float yawspeed_setpoint, const float yaw_setpoint)
	{
		// Yaw-lock depends on desired yawspeed input. If not locked, yaw_sp is set to NAN.
		if (fabsf(yawspeed_setpoint) > FLT_EPSILON) {
			// no fixed heading when rotating around yaw by stick
			return NAN;

		} else {
			// break down and hold the current heading when no more rotation commanded
			if (!PX4_ISFINITE(yaw_setpoint)) {
				return yaw;

			} else {
				return yaw_setpoint;
			}
		}
	}

	/**
	 * Limit the the horizontal input from a square shaped joystick gimbal to a unit circle
	 * @param v Vector containing x, y, z axis of the joystick gimbal. x, y get adjusted
	 */
	void limitStickUnitLengthXY(matrix::Vector2f &v)
	{
		const float vl = v.length();

		if (vl > 1.0f) {
			v(1) /= vl;
			v(2) /= vl;
		}
	}

	/**
	 * Rotate horizontal component of joystick input into the vehicle body orientation
	 * @param v Vector containing x, y, z axis of the joystick input.
	 * @param yaw Current vehicle yaw heading
	 * @param yaw_setpoint Current yaw setpoint if it's locked else NAN
	 */
	void rotateIntoHeadingFrameXY(matrix::Vector2f &v, const float yaw, const float yaw_setpoint)
	{
		using namespace matrix;
		// Rotate horizontal acceleration input to body heading
		Vector3f v3(v(0), v(1), 0.f);
		const float yaw_rotate = PX4_ISFINITE(yaw_setpoint) ? yaw_setpoint : yaw;
		v = Vector2f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * v3);
	}

private:
	SlewRate<float> _yawspeed_slew_rate;

};
