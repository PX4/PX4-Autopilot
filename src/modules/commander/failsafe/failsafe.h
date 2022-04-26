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

#include "framework.h"


class Failsafe : public FailsafeBase
{
public:
	Failsafe(ModuleParams *parent) : FailsafeBase(parent) {}

protected:

	void checkStateAndMode(const hrt_abstime &time_us, const State &state,
			       const vehicle_status_flags_s &status_flags) override;
	Action checkModeFallback(const vehicle_status_flags_s &status_flags, uint8_t user_intended_mode) const override;

	uint8_t modifyUserIntendedMode(Action previous_action, Action current_action,
				       uint8_t user_intended_mode) const override;

private:
	enum class RCLossExceptionBits : int32_t {
		Mission = (1 << 0),
		Hold = (1 << 1),
		Offboard = (1 << 2)
	};

	ActionOptions fromNavDllOrRclActParam(int param_value) const;

	ActionOptions fromGfActParam(int param_value) const;

	const int _caller_id_mode_fallback{genCallerId()};
	bool _last_state_mode_fallback{false};
	const int _caller_id_mission_control_lost{genCallerId()};
	bool _last_state_mission_control_lost{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FailsafeBase,
					(ParamInt<px4::params::NAV_DLL_ACT>) 	_param_nav_dll_act,
					(ParamInt<px4::params::NAV_RCL_ACT>) 	_param_nav_rcl_act,
					(ParamInt<px4::params::COM_RCL_EXCEPT>) _param_com_rcl_except,
					(ParamInt<px4::params::COM_RC_IN_MODE>) _param_com_rc_in_mode,
					(ParamInt<px4::params::COM_POSCTL_NAVL>) _param_com_posctl_navl,
					(ParamInt<px4::params::GF_ACTION>)  	_param_gf_action
				       );

};

