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


// GZBridge mixing class for ESCs.
// It is separate from GZBridge to have separate WorkItems and therefore allowing independent scheduling
// All work items are expected to run on the same work queue.
class GZMixingInterfaceESC : public OutputModuleInterface
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;

	GZMixingInterfaceESC(gz::transport::Node &node, pthread_mutex_t &node_mutex) :
		OutputModuleInterface(MODULE_NAME "-actuators-esc", px4::wq_configurations::rate_ctrl),
		_node(node),
		_node_mutex(node_mutex)
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

	gz::transport::Node &_node;
	pthread_mutex_t &_node_mutex;

	MixingOutput _mixing_output{"SIM_GZ_EC", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	gz::transport::Node::Publisher _actuators_pub;

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

};
