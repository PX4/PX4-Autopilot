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

#include "FunctionProviderBase.hpp"

#include <uORB/topics/wing_deploy_command.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <parameters/param.h>

/**
 * Functions: Wing_Deploy
 */
class FunctionWingDeploy : public FunctionProviderBase
{
public:
	FunctionWingDeploy() = default;
	static FunctionProviderBase *allocate(const Context &context) { return new FunctionWingDeploy(); }

	void update() override
	{
		wing_deploy_command_s wing_deploy_cmd;
		manual_control_setpoint_s manual_control;

		// Check for automatic wing deploy command from rocket mode manager (highest priority)
		if (_topic.update(&wing_deploy_cmd)) {
			if (wing_deploy_cmd.deploy) {
				_data = 1.f;  // Deploy wings (max position)
				_auto_deployed = true; // Remember that wings were deployed automatically
			} else {
				// Only allow automatic retraction if not manually overridden
				if (!_auto_deployed) {
					_data = -1.f; // Retract wings (min position)
				}
			}
		}

		// Check for manual RC control (lower priority than automatic)
		if (_manual_control_topic.update(&manual_control)) {
			// Get the configured RC AUX channel for wing deploy control
			int32_t rc_aux_channel = 0;
			param_get(_param_rc_aux_channel, &rc_aux_channel);

			if (rc_aux_channel > 0 && rc_aux_channel <= 6) {
				float aux_value = 0.f;

				// Get the appropriate AUX channel value
				switch (rc_aux_channel) {
					case 1: aux_value = manual_control.aux1; break;
					case 2: aux_value = manual_control.aux2; break;
					case 3: aux_value = manual_control.aux3; break;
					case 4: aux_value = manual_control.aux4; break;
					case 5: aux_value = manual_control.aux5; break;
					case 6: aux_value = manual_control.aux6; break;
				}

				// Manual RC logic: Can only deploy, cannot retract if auto-deployed
				if (aux_value > 0.5f) {
					_data = 1.f;  // Always allow manual deploy
				} else if (aux_value < -0.5f && !_auto_deployed) {
					// Only allow manual retraction if wings were NOT auto-deployed
					_data = -1.f; // Retract wings
				}
				// If AUX is in middle position (-0.5 to 0.5), maintain current position
				// If auto-deployed and trying to retract, ignore the command (maintain deployed state)
			}
		}
	}

	float value(OutputFunction func) override { return _data; }

private:
	uORB::Subscription _topic{ORB_ID(wing_deploy_command)};
	uORB::Subscription _manual_control_topic{ORB_ID(manual_control_setpoint)};
	float _data{-1.f}; // Start with wings retracted
	bool _auto_deployed{false}; // Track if wings were deployed automatically

	// Parameter for configurable RC AUX channel
	param_t _param_rc_aux_channel{param_find("WD_RC_AUX_CH")};
};
