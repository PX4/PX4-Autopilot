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

using namespace matrix;

class FlightTaskSport : public FlightTaskManualPosition
{
public:
	FlightTaskSport() = default;

	virtual ~FlightTaskSport() = default;

protected:
	void _updateSetpoints() override
	{
		FlightTaskManualPosition::_updateSetpoints(); // get all setpoints from position task

		/* Scale horizontal velocity setpoint by maximum allowed velocity. */
		if (PX4_ISFINITE(_velocity_setpoint(0)) && Vector2f(&_velocity_setpoint(0)).length() > 0.0f) {
			Vector2f vel_sp_xy = Vector2f(&_velocity_setpoint(0));
			vel_sp_xy = vel_sp_xy.normalized() * _vel_xy_max.get() / _vel_xy_manual_max.get() * vel_sp_xy.length();
			_velocity_setpoint(0) = vel_sp_xy(0);
			_velocity_setpoint(1) = vel_sp_xy(1);
		}
	}

private:
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualPosition,
					(ParamFloat<px4::params::MPC_XY_VEL_MAX>)
					_vel_xy_max /**< maximal allowed horizontal speed, in sport mode full stick input*/
				       )

};
