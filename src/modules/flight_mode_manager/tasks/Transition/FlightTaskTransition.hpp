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
 * @file FlightTaskTransition.hpp
 *
 * Flight task for automatic VTOL transitions between hover and forward flight and vice versa.
 */

#pragma once

#include "FlightTask.hpp"
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;


class FlightTaskTransition : public FlightTask
{
public:
	FlightTaskTransition();

	virtual ~FlightTaskTransition() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

private:
	static constexpr float VERTICAL_VELOCITY_TIME_CONSTANT = 2.0f;
	static constexpr float DECELERATION_INTEGRATOR_LIMIT = .3f;

	uORB::SubscriptionData<vehicle_status_s> _sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<position_setpoint_triplet_s> _sub_position_sp_triplet{ORB_ID(position_setpoint_triplet)};

	float _param_fw_psp_off{0.f};
	float _param_vt_b_dec_i{0.f};
	float _param_vt_b_dec_mss{0.f};

	AlphaFilter<float> _vel_z_filter{VERTICAL_VELOCITY_TIME_CONSTANT};
	float _decel_error_bt_int{0.f}; ///< Backtransition deceleration error integrator value

	float computeBackTranstionPitchSetpoint();
};
