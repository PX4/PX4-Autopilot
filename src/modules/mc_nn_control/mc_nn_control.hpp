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

/**
 * @file mc_nn_control.h
 * Multicopter Neural Network Control module, from position setpoints to control allocator.
 *
 * @author Sindre Meyer Hegre <sindre.hegre@gmail.com>
 * @author Welf Rehberg <welf.rehberg@ntnu.no>
 */
#pragma once

#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <matrix/matrix/math.hpp>

#include <tflite_micro/tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tflite_micro/tensorflow/lite/micro/micro_interpreter.h>
#include <tflite_micro/tensorflow/lite/schema/schema_generated.h>

// Include model
#include "control_net.hpp"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

// Subscriptions
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>

// Publications
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/neural_control.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/arming_check_reply.h>

using namespace time_literals; // For the 1_s in the subscription interval
class MulticopterNeuralNetworkControl : public ModuleBase<MulticopterNeuralNetworkControl>, public ModuleParams,
	public px4::WorkItem
{
public:

	MulticopterNeuralNetworkControl();
	~MulticopterNeuralNetworkControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	void Run() override;

	// Functions
	void PopulateInputTensor();
	void PublishOutput(float *command_actions);
	void RescaleActions();
	int InitializeNetwork();
	int32_t GetTime();
	void RegisterNeuralFlightMode();
	void UnregisterNeuralFlightMode(int8 arming_check_id, int8 mode_id);
	void ConfigureNeuralFlightMode(int8 mode_id);
	void ReplyToArmingCheck(int8 request_id);
	void CheckModeRegistration();
	void generate_trajectory_setpoint(float dt);
	void reset_trajectory_setpoint(vehicle_local_position_s &_position);
	void check_setpoint_validity(vehicle_local_position_s &_position);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Publications
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<neural_control_s> _neural_control_pub{ORB_ID(neural_control)};
	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};

	// Variables
	bool _use_neural{false};
	bool _sent_mode_registration{false};
	perf_counter_t _loop_perf; /**< loop duration performance counter */
	hrt_abstime _last_run{0};
	uint8 _mode_request_id{231}; //Random value
	int8 _arming_check_id{-1};
	int8 _mode_id{-1};
	tflite::MicroInterpreter *_interpreter;
	TfLiteTensor *_input_tensor;
	TfLiteTensor *_output_tensor;
	float _input_data[15];
	trajectory_setpoint_s _trajectory_setpoint;
	vehicle_angular_velocity_s _angular_velocity;
	vehicle_local_position_s _position;
	vehicle_attitude_s _attitude;
	manual_control_setpoint_s _manual_control_setpoint{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_NN_MAX_RPM>) _param_max_rpm,
		(ParamInt<px4::params::MC_NN_MIN_RPM>) _param_min_rpm,
		(ParamFloat<px4::params::MC_NN_THRST_COEF>) _param_thrust_coeff,
		(ParamBool<px4::params::MC_NN_MANL_CTRL>) _param_manual_control
	)
};
