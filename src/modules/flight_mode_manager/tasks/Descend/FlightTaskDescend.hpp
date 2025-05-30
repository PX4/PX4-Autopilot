/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskDescend.hpp
 */

#pragma once

#include <lib/stick_yaw/StickYaw.hpp>
#include "FlightTask.hpp"
#include "Sticks.hpp"
#include "StickTiltXY.hpp"

class FlightTaskDescend : public FlightTask
{
public:
	FlightTaskDescend() = default;
	virtual ~FlightTaskDescend() = default;

	bool update() override;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;

private:
	Sticks _sticks{this};
	StickTiltXY _stick_tilt_xy{this};
	StickYaw _stick_yaw{this};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamInt<px4::params::MPC_LAND_RC_HELP>) _param_mpc_land_rc_help,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed, ///< velocity for controlled descend
					(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover ///< thrust at hover equilibrium
				       )
};
