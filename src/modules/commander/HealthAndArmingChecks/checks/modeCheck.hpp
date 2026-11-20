/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#if defined(CONFIG_MODE_TURTLE)
#include <uORB/topics/turtle_control.h>
#include <uORB/topics/motor_direction_status.h>
#include <uORB/topics/control_allocator_status.h>
#endif

class ModeChecks : public HealthAndArmingCheckBase
{
public:
	ModeChecks() = default;
	~ModeChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

#if defined(CONFIG_MODE_TURTLE)
	struct TurtleArmingStatus {
		hrt_abstime timestamp_disarmed{0};
		bool ready_to_arm{false};
		bool direction_normal{false};
		bool direction_active{false};
	};

	TurtleArmingStatus turtleArmingStatus(const vehicle_status_s &status);
	bool copyTurtleControl(turtle_control_s &turtle, hrt_abstime nav_state_timestamp);
#endif

private:
#if defined(CONFIG_MODE_TURTLE)
	const param_t _turtle_enable_handle {param_find("TRTL_EN")};
	const param_t _throw_enable_handle {param_find("COM_THROW_EN")};
	uORB::Subscription _turtle_control_sub {ORB_ID(turtle_control)};
	uORB::Subscription _motor_direction_status_sub{ORB_ID(motor_direction_status)};
	uORB::Subscription _turtle_allocator_status_sub{ORB_ID(control_allocator_status)};
#endif
	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamBool<px4::params::COM_ARM_MIS_REQ>) _param_com_arm_mis_req,
					(ParamInt<px4::params::COM_RC_IN_MODE>) _param_com_rc_in_mode
				       );
};
