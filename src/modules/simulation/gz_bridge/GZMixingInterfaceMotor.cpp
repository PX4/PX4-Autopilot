/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "GZMixingInterfaceMotor.hpp"

bool GZMixingInterfaceMotor::init(const std::string &model_name)
{
	pthread_mutex_init(&_node_mutex, nullptr);

	// Update mixing output count such that the interface can be configured as ESC mixer or PWM mixer
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);

	if (isESCInterface()) {
		// ESC feedback: /x500/command/motor_speed
		std::string motor_speed_topic = "/" + model_name + "/command/motor_speed";

		if (!_node.Subscribe(motor_speed_topic, &GZMixingInterfaceMotor::motorSpeedCallback, this)) {
			PX4_ERR("failed to subscribe to %s", motor_speed_topic.c_str());
			return false;
		}

		PX4_INFO("GZ bridge Motor interface configured as ESC interface, reporting ESC telemetry");

		_esc_status_pub.advertise();

	} else {
		PX4_INFO("GZ bridge Motor interface configured as PWM interface, not reporting ESC telemetry");
	}

	// output eg /X500/command/motor_speed
	std::string actuator_topic = "/" + model_name + "/command/motor_speed";
	_actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);

	if (!_actuators_pub.Valid()) {
		PX4_ERR("failed to advertise %s", actuator_topic.c_str());
		return false;
	}


	ScheduleNow();

	return true;
}

bool GZMixingInterfaceMotor::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
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
		gz::msgs::Actuators rotor_velocity_message;
		rotor_velocity_message.mutable_velocity()->Resize(active_output_count, 0);

		for (unsigned i = 0; i < active_output_count; i++) {
			rotor_velocity_message.set_velocity(i, outputs[i]);
		}

		if (_actuators_pub.Valid()) {
			return _actuators_pub.Publish(rotor_velocity_message);
		}
	}

	return false;
}

bool GZMixingInterfaceMotor::isESCInterface()
{
	unsigned active_output_count = 0;

	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			active_output_count++;

		} else {
			break;
		}
	}

	return (active_output_count <= MAX_DSHOT_ESCS);
}

void GZMixingInterfaceMotor::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

void GZMixingInterfaceMotor::motorSpeedCallback(const gz::msgs::Actuators &actuators)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	esc_status_s esc_status{};
	esc_status.esc_count = actuators.velocity_size();

	for (int i = 0; i < actuators.velocity_size(); i++) {
		esc_status.esc[i].timestamp = hrt_absolute_time();
		esc_status.esc[i].esc_rpm = actuators.velocity(i);
		esc_status.esc_online_flags |= 1 << i;

		if (actuators.velocity(i) > 0) {
			esc_status.esc_armed_flags |= 1 << i;
		}
	}

	if (esc_status.esc_count > 0) {
		esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(esc_status);
	}

	pthread_mutex_unlock(&_node_mutex);
}
