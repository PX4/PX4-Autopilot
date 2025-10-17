/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
#include <uORB/topics/input_rc.h>

class RcCalibrationChecks : public HealthAndArmingCheckBase
{
public:
	RcCalibrationChecks();
	~RcCalibrationChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	void updateParams() override;

	struct ParamHandles {
		param_t min;
		param_t trim;
		param_t max;
	};
	struct ParamValues {
		float min;
		float trim;
		float max;
	};

	ParamHandles _param_handles[input_rc_s::RC_INPUT_MAX_CHANNELS];
	ParamValues _param_values[input_rc_s::RC_INPUT_MAX_CHANNELS];

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::COM_RC_IN_MODE>) _param_com_rc_in_mode
				       )
};
