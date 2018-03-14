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
 * @file FlightTaskSport.hpp
 *
 * Manual controlled position with maximum speed and no smoothing.
 */

#pragma once

#include "FlightTaskManualPosition.hpp"
#include "mathlib/mathlib.h"
#include <float.h>

class FlightTaskSport : public FlightTaskManualPosition
{
public:
	FlightTaskSport(control::SuperBlock *parent, const char *name) :
		FlightTaskManualPosition(parent, name),
		_vel_xy_max(parent, "MPC_XY_VEL_MAX", false)
	{ }

	virtual ~FlightTaskSport() = default;

protected:
	void _scaleSticks() override
	{

		/* Get all stick scaling from FlightTaskManualAltitude */
		FlightTaskManualAltitude::_scaleSticks();

		/* Constrain length of stick inputs to 1 for xy*/
		matrix::Vector2f stick_xy(_sticks_expo(0), _sticks_expo(1));

		float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);

		if (mag > FLT_EPSILON) {
			stick_xy = stick_xy.normalized() * mag;
		}

		/* Scale to velocity using max velocity */
		_vel_sp_xy = stick_xy * _vel_xy_max.get();

		/* Rotate setpoint into local frame. */
		matrix::Vector3f vel_sp { _vel_sp_xy(0), _vel_sp_xy(1), 0.0f };
		vel_sp = (matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, _yaw)) * vel_sp);
		_vel_sp_xy = matrix::Vector2f(vel_sp(0), vel_sp(1));
	}

private:
	control::BlockParamFloat _vel_xy_max; /**< maximal allowed horizontal speed, in sport mode full stick input*/

};
