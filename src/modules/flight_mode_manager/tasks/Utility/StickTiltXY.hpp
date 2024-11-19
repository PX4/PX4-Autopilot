/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file StickTiltXY.hpp
 * @brief Generate only horizontal acceleration setpoint from stick input
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/module_params.h>

class StickTiltXY : public ModuleParams
{
public:
	StickTiltXY(ModuleParams *parent);
	~StickTiltXY() = default;

	/**
	 * Produce acceleration setpoint to tilt a multicopter based on stick input
	 *
	 * Forward pitch stick input pitches the vehicle's pitch e.g. accelerates the vehicle in its nose direction.
	 *
	 * @param stick_xy the raw pitch and roll stick positions as input
	 * @param dt time in seconds since the last execution
	 * @param yaw the current yaw estimate for frame rotation
	 * @param yaw_setpoint the current heading setpoint used instead of the estimate if absolute yaw is locked
	 * @return NED frame horizontal x, y axis acceleration setpoint
	 */
	matrix::Vector2f generateAccelerationSetpoints(matrix::Vector2f stick_xy, const float dt, const float yaw,
			const float yaw_setpoint);
private:
	void updateParams() override;

	float _maximum_acceleration{0.f};
	AlphaFilter<matrix::Vector2f> _man_input_filter;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max, ///< maximum tilt allowed for manual flight
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>) _param_mc_man_tilt_tau ///< time constant for stick filter
	)
};
