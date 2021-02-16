/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ManualControl.hpp
 *
 * @brief Logic for handling RC or Joystick input
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

class ManualControl : ModuleParams
{
public:
	ManualControl(ModuleParams *parent) : ModuleParams(parent) {};
	~ManualControl() override = default;

	void setRCAllowed(const bool rc_allowed) { _rc_allowed = rc_allowed; }
	void update();
	bool isRCAvailable() { return _rc_available; }
	bool wantsOverride(const vehicle_control_mode_s &vehicle_control_mode);

	manual_control_setpoint_s _manual_control_setpoint{};

private:
	void process(manual_control_setpoint_s &manual_control_setpoint);

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _last_manual_control_setpoint{};

	bool _rc_allowed{false};
	bool _rc_available{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::COM_RC_LOSS_T>) _param_com_rc_loss_t,
		(ParamInt<px4::params::COM_RC_OVERRIDE>) _param_rc_override,
		(ParamFloat<px4::params::COM_RC_STICK_OV>) _param_com_rc_stick_ov
	)
};
