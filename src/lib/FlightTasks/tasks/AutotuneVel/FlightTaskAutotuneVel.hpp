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
 * @file FlightTaskAutotuneVel.hpp
 *
 * Flight task velocity PID autotuning
 */

#pragma once

#include "FlightTaskManual.hpp"

class FlightTaskAutotuneVel : public FlightTaskManual
{
public:
	FlightTaskAutotuneVel() = default;
	virtual ~FlightTaskAutotuneVel() = default;
	bool initializeSubscriptions(SubscriptionArray &subscription_array) override;
	bool activate() override;
	bool updateInitialize() override;
	bool update() override;

protected:
	virtual void _updateSetpoints(); /**< updates all setpoints */

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManual,
					(ParamBool<px4::params::MPC_XY_VEL_ATUNE>) _param_mpc_xy_vel_atune,
					(ParamFloat<px4::params::MPC_XY_VEL_P>) _param_mpc_xy_vel_p,
					(ParamFloat<px4::params::MPC_XY_VEL_I>) _param_mpc_xy_vel_i,
					(ParamFloat<px4::params::MPC_XY_VEL_D>) _param_mpc_xy_vel_d
				       )
private:
	bool _checkSticks(); /**< check if the user wants to abort the autotuning */
	void _updateUltimateGain();
	void _measureUltimatePeriod();
	void _computeControlGains();
	void _exit();

	float _thrust{};
	float _thrust_sat{};
	float _thrust_max{0.1f}; /**< maximum allowed thrust amplitude **/
	float _ku{}; /**< ultimate gain of the controller */
	float _period_u{}; /**< ultimate period is seconds **/
	float _epsilon{0.0001};
	float _alpha{0.05f};
	bool _done{false};

	matrix::Vector2f _current_position_xy{};

	int _convergence_counter{}; /**< counts how many times the exit criteria has been true **/

	/* Period counter */
	int _peak_counter{};
	enum class SignalState {unknown, low, high};
	SignalState _peak_counter_state{SignalState::unknown};
	hrt_abstime _start_time{};
};
