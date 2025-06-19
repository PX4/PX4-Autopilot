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

#pragma once

#include <lib/mixer_module/mixer_module.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/esc_status.h>

/**
 * GZBridge mixing class for Motors.
 * It is separate from GZBridge to have separate WorkItems and therefore allowing independent scheduling
 * All work items are expected to run on the same work queue.
 *
 * Simulates behavior of (Dshot/UAVCAN) escs or PWM escs based on how many actuators are configured with SIM_GZ_EC function.
 * If less than 8 actuators are configured, esc telemetry can be reported via esc_status uORB topic. But if its more than 8
 * it cannot be reported, and the interface will behave as a PWM interface.
 */
class GZMixingInterfaceMotor : public OutputModuleInterface
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;
	static constexpr int MAX_DSHOT_ESCS = 8; // Maximum number of DShot ESCs supported

	GZMixingInterfaceMotor(gz::transport::Node &node) :
		OutputModuleInterface(MODULE_NAME "-actuators-motor", px4::wq_configurations::rate_ctrl),
		_node(node)
	{}

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;


	MixingOutput &mixingOutput() { return _mixing_output; }

	bool init(const std::string &model_name);

	void stop()
	{
		_mixing_output.unregister();
		ScheduleClear();
	}

private:
	friend class GZBridge;

	void Run() override;

	void motorSpeedCallback(const gz::msgs::Actuators &actuators);

	bool isESCInterface();

	gz::transport::Node &_node;
	pthread_mutex_t _node_mutex;

	MixingOutput _mixing_output{"SIM_GZ_EC", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	gz::transport::Node::Publisher _actuators_pub;

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

};
