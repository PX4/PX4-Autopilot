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
 * @file FlightManualAltitude.hpp
 *
 * Flight task for manual controlled altitude via velocity setpoints.
 */

#pragma once

#include "FlightTask.hpp"
#include "Sticks.hpp"
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/Subscription.hpp>

class FlightTaskManualAltitudeCommandVel : public FlightTask
{
public:
	FlightTaskManualAltitudeCommandVel();
	virtual ~FlightTaskManualAltitudeCommandVel() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

protected:
	virtual void _updateSetpoints(); /**< updates all setpoints */
	virtual void _scaleSticks(); /**< scales sticks to velocity in z */
	bool _checkTakeoff() override;

	/**
	 * rotates vector into local frame
	 */
	void _rotateIntoHeadingFrame(matrix::Vector2f &vec);

	Sticks _sticks;
	bool _sticks_data_required = true; ///< let inherited task-class define if it depends on stick data

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max, /**< scaling factor from stick to yaw rate */
					(ParamFloat<px4::params::MPC_MAN_Y_TAU>) _param_mpc_man_y_tau,
					(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max, /**< maximum tilt allowed for manual flight */
					(ParamFloat<px4::params::MC_MAN_TILT_TAU>) _param_mc_man_tilt_tau
				       )
private:

	/**
	 * Filter between stick input and yaw setpoint
	 *
	 * @param yawspeed_target yaw setpoint desired by the stick
	 * @return filtered value from independent filter state
	 */
	float _applyYawspeedFilter(float yawspeed_target);

	float _yawspeed_filter_state{}; /**< state of low-pass filter in rad/s */

	matrix::Vector3f _last_position; /**< last loop's vehicle position */

	AlphaFilter<matrix::Vector2f> _man_input_filter;
};
