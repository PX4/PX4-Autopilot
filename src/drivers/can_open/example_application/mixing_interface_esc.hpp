/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <lib/mixer_module/mixer_module.hpp>

#include <uORB/Subscription.hpp>

#include "CANopen.h"
#include "OD.h"
#include "ODRecord.hpp"

class COMixingInterfaceESC : public OutputModuleInterface, public ODRecord
{
public:
	COMixingInterfaceESC() :
		OutputModuleInterface(MODULE_NAME "-actuators-esc", px4::wq_configurations::can), ODRecord(UORB_TO_OD_RECORD) { };
	~COMixingInterfaceESC() {};

	void update()
	{
		_mixing_output.update();
		_mixing_output.updateSubscriptions();
	}

protected:
	void Run() override
	{
		PX4_ERR("Run called");
	};

private:
	enum motor_controller_cmd_subidx {
		SUBIDX_DESIRED_THROTTLE = 1,
		SUBIDX_DESIRED_GEAR
	};
	enum motor_controller_cmd_gear {
		GEAR_NEUTRAL = 0,
		GEAR_FORWARD,
		GEAR_REVERSE
	};

	MixingOutput _mixing_output{"CO_EC", 1, *this, MixingOutput::SchedulingPolicy::Disabled, false, false};

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override
	{
		int16_t des_throttle;
		uint8_t des_gear;
		const int16_t midpt = (_mixing_output.maxValue(0) - _mixing_output.minValue(0) + 1)/ 2;
		const int16_t output = outputs[0];

		if(num_outputs != 1) {
			PX4_ERR("Invalid number of outputs: %u", num_outputs);
			return false;
		}

		if(stop_motors || (output == _mixing_output.disarmedValue(0)) ||
				  (output == _mixing_output.failsafeValue(0))){
			des_throttle = 0;
			des_gear = GEAR_NEUTRAL;
		} else {
			des_throttle = output - midpt;
			if(des_throttle == 0) {
				des_gear = GEAR_NEUTRAL;
			} else if(des_throttle < 0) {
				des_gear = GEAR_REVERSE;
				des_throttle *= -1;
			} else {
				des_gear = GEAR_FORWARD;
			}
		}
		OD_set_i16(OD_ENTRY_H5021_motorControllerCommand, SUBIDX_DESIRED_THROTTLE, des_throttle, false);
		OD_set_u8(OD_ENTRY_H5021_motorControllerCommand, SUBIDX_DESIRED_GEAR, des_gear, false);
		return true;
	};
};
