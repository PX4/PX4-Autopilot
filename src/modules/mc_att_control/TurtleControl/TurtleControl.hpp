/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include "TurtleSession.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/motor_direction_request.h>
#include <uORB/topics/motor_direction_status.h>
#include <uORB/topics/turtle_control.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>

class TurtleControl : public ModuleParams
{
public:
	explicit TurtleControl(ModuleParams *parent);
	~TurtleControl() override;

	/** Update recovery and return whether Turtle is selected. */
	bool update(const vehicle_status_s &status, const manual_control_setpoint_s &manual,
		    const vehicle_attitude_s &attitude);

private:
	void updateParams() override { _parameters_pending = true; }
	void stop();
	void requestReverse(uint64_t authorizing_status_timestamp);
	static bool fresh(uint64_t timestamp, uint64_t now, uint64_t timeout);

	static constexpr uint16_t MOTOR_MASK = 0xf; // Four logical motors.
	static constexpr float STICK_DEADBAND = 0.2f;
	static constexpr float THROTTLE_LOW = -0.9f;

	uORB::SubscriptionData<control_allocator_status_s> _allocation_sub{ORB_ID(control_allocator_status)};
	uORB::SubscriptionData<motor_direction_status_s> _direction_sub{ORB_ID(motor_direction_status)};
	uORB::Publication<turtle_control_s> _control_pub{ORB_ID(turtle_control)};
	uORB::Publication<motor_direction_request_s> _direction_pub{ORB_ID(motor_direction_request)};

	enum class ArmState : uint8_t { Disarmed, Active, Aborted };

	TurtleSession _session;
	uint32_t _request_id{0};
	ArmState _arm_state{ArmState::Disarmed};
	bool _selected{false};
	bool _parameters_pending{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::TRTL_EN>) _param_enabled,
		(ParamFloat<px4::params::TRTL_MAX>) _param_maximum,
		(ParamFloat<px4::params::TRTL_TIME>) _param_time
	)
};
