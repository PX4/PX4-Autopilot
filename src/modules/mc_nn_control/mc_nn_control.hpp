/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
#include <tflite_micro/tensorflow/lite/kernels/internal/reference/tanh.h>

// Include model
#include "allocation_net.hpp"
#include "control_net.hpp"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

// Subscriptions
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>

// Publications
#include <uORB/topics/actuator_motors.h>

using namespace time_literals; // For the 1_s in the subscription callback

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

	bool init();

private:
	void Run() override;

	// Functions
	void PopulateInputTensor();
	void PublishOutput(float* command_actions);
	void RescaleActions();
	int InitializeNetwork();

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Publications
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	// Variables
	bool _use_neural{false};
	perf_counter_t _loop_perf; /**< loop duration performance counter */
	hrt_abstime _last_run{0};
	tflite::MicroInterpreter* _control_interpreter;
	tflite::MicroInterpreter* _allocation_interpreter;
	TfLiteTensor* _input_tensor;
	TfLiteTensor* _output_tensor;
	vehicle_angular_velocity_s _angular_velocity;
	vehicle_local_position_s _position;
	vehicle_local_position_setpoint_s _position_setpoint;
	vehicle_attitude_s _attitude;

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::NN_IN_DEBUG>) 		_param_debug_input_tensor,
		(ParamBool<px4::params::NN_OUT_DEBUG>)	 	_param_debug_output_tensor,
		(ParamBool<px4::params::NN_TIME_DEBUG>)		_param_debug_inference_time
	)
};
