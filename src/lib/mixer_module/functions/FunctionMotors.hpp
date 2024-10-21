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

#include <lib/slew_rate/SlewRate.hpp>
#include <uORB/topics/actuator_motors.h>

/**
 * Functions: Motor1 ... MotorMax
 */
class FunctionMotors : public FunctionProviderBase
{
public:
	static_assert(actuator_motors_s::NUM_CONTROLS == (int)OutputFunction::MotorMax - (int)OutputFunction::Motor1 + 1,
		      "Unexpected num motors");

	static_assert(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 == (int)OutputFunction::Motor1, "Unexpected motor idx");

	FunctionMotors(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_motors)),
		_thrust_factor(context.thrust_factor),
		_motor_rise_time(context.motor_rise_time)
	{
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
			_output_slewrates[i].setForcedValue(NAN);
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionMotors(context); }

	void update() override
	{
		if (_topic.update(&_data)) {
			updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
		}

		if (_motor_rise_time > FLT_EPSILON) { // also makes sure to not divide by zero
			hrt_abstime now = hrt_absolute_time();
			const float dt = (now - _timestamp_last_update) / 1e6f;

			for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
				// (rise time [-1,1] = 2 / slew) -> (slew = 2 / rise time [-1,1])
				_output_slewrates[i].setSlewRate(2.f / _motor_rise_time);

				if (!PX4_ISFINITE(_output_slewrates[i].getState())) {
					_output_slewrates[i].setForcedValue(_data.control[i]);

				} else {
					_data.control[i] = _output_slewrates[i].update(_data.control[i], dt);
				}
			}

			_timestamp_last_update = now;
		}
	}

	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Motor1]; }

	bool allowPrearmControl() const override { return false; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	static inline void updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values)
	{
		if (thrust_factor > 0.f && thrust_factor <= 1.f) {
			// thrust factor
			//  rel_thrust = factor * x^2 + (1-factor) * x,
			const float a = thrust_factor;
			const float b = (1.f - thrust_factor);

			// don't recompute for all values (ax^2+bx+c=0)
			const float tmp1 = b / (2.f * a);
			const float tmp2 = b * b / (4.f * a * a);

			for (int i = 0; i < num_values; ++i) {
				float control = values[i];

				if (control > 0.f) {
					values[i] = -tmp1 + sqrtf(tmp2 + (control / a));

				} else if (control < -0.f) {
					values[i] =  tmp1 - sqrtf(tmp2 - (control / a));

				} else {
					values[i] = 0.f;
				}
			}
		}

		for (int i = 0; i < num_values; ++i) {
			if ((reversible & (1u << i)) == 0) {
				if (values[i] < -FLT_EPSILON) {
					values[i] = NAN;

				} else {
					// remap from [0, 1] to [-1, 1]
					values[i] = values[i] * 2.f - 1.f;
				}
			}
		}
	}

	bool reversible(OutputFunction func) const override { return _data.reversible_flags & (1u << ((int)func - (int)OutputFunction::Motor1)); }

private:
	uORB::SubscriptionCallbackWorkItem _topic;
	actuator_motors_s _data{};
	const float &_thrust_factor;
	const float &_motor_rise_time; // Parameter to configure slew rate
	SlewRate<float> _output_slewrates[actuator_motors_s::NUM_CONTROLS];
	hrt_abstime _timestamp_last_update{0};
};
