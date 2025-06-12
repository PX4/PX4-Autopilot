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

#include <gz/transport.hh>
#include <px4_platform_common/module_params.h>

// GZBridge mixing class for Servos.
// It is separate from GZBridge to have separate WorkItems and therefore allowing independent scheduling
// All work items are expected to run on the same work queue.
class GZMixingInterfaceServo : public OutputModuleInterface
{
public:
	GZMixingInterfaceServo(gz::transport::Node &node) :
		OutputModuleInterface(MODULE_NAME "-actuators-servo", px4::wq_configurations::rate_ctrl),
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

	/**
	 * @brief Get maximum configured angle of servo.
	 * @param index: servo index
	 * @return angle_max [rad]
	 */
	float get_servo_angle_max(const size_t index);

	/**
	 * @brief Get minimum configured angle of servo.
	 * @param index: servo index
	 * @return angle_min [rad]
	 */
	float get_servo_angle_min(const size_t index);

	gz::transport::Node &_node;
	pthread_mutex_t _node_mutex;

	MixingOutput _mixing_output{"SIM_GZ_SV", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	std::vector<gz::transport::Node::Publisher> _servos_pub;
	std::vector<double> _angle_min_rad;
	std::vector<double> _angular_range_rad;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA1>) _servo_max_angle_1,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA1>) _servo_min_angle_1,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA2>) _servo_max_angle_2,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA2>) _servo_min_angle_2,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA3>) _servo_max_angle_3,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA3>) _servo_min_angle_3,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA4>) _servo_max_angle_4,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA4>) _servo_min_angle_4,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA5>) _servo_max_angle_5,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA5>) _servo_min_angle_5,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA6>) _servo_max_angle_6,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA6>) _servo_min_angle_6,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA7>) _servo_max_angle_7,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA7>) _servo_min_angle_7,
		(ParamFloat<px4::params::SIM_GZ_SV_MAXA8>) _servo_max_angle_8,
		(ParamFloat<px4::params::SIM_GZ_SV_MINA8>) _servo_min_angle_8
	)
};
