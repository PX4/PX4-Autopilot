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

#include "GZMixingInterfaceThruster.hpp"

bool GZMixingInterfaceThruster::init(const std::string &model_name)
{
	pthread_mutex_init(&_node_mutex, nullptr);

	// output eg /X500/command/motor_speed
	std::string actuator_topic = "/" + model_name + "/command/duty_cycle";
	_actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);

	if (!_actuators_pub.Valid()) {
		PX4_ERR("failed to advertise %s", actuator_topic.c_str());
		return false;
	}


	ScheduleNow();

	return true;
}

bool GZMixingInterfaceThruster::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
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
		gz::msgs::Actuators thruster_pwm_message;
		thruster_pwm_message.mutable_normalized()->Resize(active_output_count, 0);

		for (unsigned i = 0; i < active_output_count; i++) {
			const double a = (1.0 - 0.0) / (_mixing_output.maxValue(i) - _mixing_output.minValue(i));
			const double b = 0.0 - (a * static_cast<double>(_mixing_output.minValue(i)));
			const double c = (a * static_cast<double>(outputs[i])) + b;
			thruster_pwm_message.set_normalized(i, c);
		}

		if (_actuators_pub.Valid()) {
			return _actuators_pub.Publish(thruster_pwm_message);
		}
	}

	return false;
}

void GZMixingInterfaceThruster::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}
