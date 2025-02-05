/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

/**
 * Functions: Thruster1 ... ThrusterMax
 */
class FunctionThrusters : public FunctionProviderBase
{
public:
	static_assert(actuator_motors_s::NUM_CONTROLS == (int)OutputFunction::ThrusterMax - (int)OutputFunction::Thruster1 + 1,
		      "Unexpected num thrusters");

	static_assert(actuator_motors_s::ACTUATOR_FUNCTION_THRUSTER1 == (int)OutputFunction::Thruster1,
		      "Unexpected thruster idx");

	FunctionThrusters(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_motors)),
		_thrust_factor(context.thrust_factor)
	{
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionThrusters(context); }

	void update() override
	{
		if (_topic.update(&_data)) {
			updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
		}
	}

	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Thruster1]; }

	bool allowPrearmControl() const override { return false; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	static inline void updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values)
	{

	}

	bool reversible(OutputFunction func) const override { return false; }

private:
	uORB::SubscriptionCallbackWorkItem _topic;
	actuator_motors_s _data{};
	const float &_thrust_factor;
};
