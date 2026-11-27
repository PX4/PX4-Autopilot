/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "GZMixingInterfaceWheel.hpp"
#include <lib/mathlib/mathlib.h>

int GZMixingInterfaceWheel::wheelSlot(int channel) const
{
	if (!_mixing_output.isMotor(channel)) {
		return -1;
	}

	// WheelEncoders entry i is the wheel driven by Motor(i+1), whichever channel it is assigned to
	const int motor = static_cast<int>(_mixing_output.outputFunction(channel)) - static_cast<int>(OutputFunction::Motor1);

	return (motor < MAX_WHEELS) ? motor : -1;
}

bool GZMixingInterfaceWheel::init(const std::string &model_name)
{

	std::string wheel_speed_topic = "/model/" + model_name + "/command/motor_speed";

	if (!_node.Subscribe(wheel_speed_topic, &GZMixingInterfaceWheel::wheelSpeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", wheel_speed_topic.c_str());
		return false;
	}

	std::string wheel_topic = "/model/" + model_name + "/command/motor_speed";
	_actuators_pub = _node.Advertise<gz::msgs::Actuators>(wheel_topic);

	if (!_actuators_pub.Valid()) {
		PX4_ERR("failed to advertise %s", wheel_topic.c_str());
		return false;
	}

	pthread_mutex_init(&_node_mutex, nullptr);

	_wheel_encoders_pub.advertise();

	ScheduleNow();

	return true;
}

bool GZMixingInterfaceWheel::updateOutputs(float outputs[MAX_ACTUATORS],
		unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	unsigned active_output_count = 0;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			active_output_count++;

		} else {
			break;
		}
	}

	if (active_output_count > 0) {
		gz::msgs::Actuators wheel_velocity_message;

		auto *vel = wheel_velocity_message.mutable_velocity();
		vel->Clear();
		vel->Reserve(active_output_count);

		// Offsetting the output allows for negative values despite unsigned integer to reverse the wheels
		static constexpr double output_offset = 100.0;

		for (unsigned i = 0; i < active_output_count; i++) {
			double scaled_output = static_cast<double>(outputs[i]) - output_offset;
			vel->Add(scaled_output);
		}

		if (_actuators_pub.Valid()) {
			return _actuators_pub.Publish(wheel_velocity_message);
		}
	}

	return false;
}

void GZMixingInterfaceWheel::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

void GZMixingInterfaceWheel::wheelSpeedCallback(const gz::msgs::Actuators &actuators)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	hrt_abstime current_t = hrt_absolute_time();

	pthread_mutex_lock(&_node_mutex);

	wheel_encoders_s wheel_encoders{};

	// Velocity i belongs to output channel i, see updateOutputs()
	const int num_channels = math::min(actuators.velocity_size(), MAX_ACTUATORS);

	// No integration on the first callback, there is no previous speed yet
	const float dt = (_prev_t == 0) ? 0.f : (current_t - _prev_t) * 1e-6f;

	for (int i = 0; i < num_channels; i++) {
		const int slot = wheelSlot(i);

		if (slot < 0) {
			continue;
		}

		const float wheel_speed = (float)actuators.velocity(i);

		if (_prev_t == 0) {
			_prev_wheel_speed[slot] = wheel_speed;
		}

		// Trapezoidal integration of the wheel speed to the unwrapped wheel angle
		_wheel_angle[slot] += 0.5f * (wheel_speed + _prev_wheel_speed[slot]) * dt;
		_prev_wheel_speed[slot] = wheel_speed;

		// Quantize like a real encoder
		const int64_t encoder_counts = static_cast<int64_t>(floorf(_wheel_angle[slot] * SIM_COUNTS_PER_REV / M_TWOPI_F));

		wheel_encoders.wheel_speed[slot] = wheel_speed;
		wheel_encoders.counts_per_rev[slot] = SIM_COUNTS_PER_REV;
		wheel_encoders.encoder_counts[slot] = encoder_counts;
		wheel_encoders.wheel_angle[slot] = encoder_counts * M_TWOPI_F / SIM_COUNTS_PER_REV;
	}

	_prev_t = current_t;

	if (actuators.velocity_size() > 0) {
		wheel_encoders.timestamp = hrt_absolute_time();
		_wheel_encoders_pub.publish(wheel_encoders);
	}

	pthread_mutex_unlock(&_node_mutex);
}
