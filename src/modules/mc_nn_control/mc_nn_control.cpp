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
 * @file mc_nn_control.cpp
 * Multicopter Neural Network Control module, from position setpoints to control allocator.
 *
 * @author Sindre Meyer Hegre <sindre.hegre@gmail.com>
 */

#include "mc_nn_control.hpp"
#ifdef __PX4_NUTTX
#include <drivers/drv_hrt.h>
#else
#include <chrono>
#endif

namespace {
using NNControlOpResolver = tflite::MicroMutableOpResolver<4>; // This number should be the number of operations in the model, like tanh and fully connected

TfLiteStatus RegisterOps(NNControlOpResolver& op_resolver) {
  // Add the operations to you need to the op_resolver
  TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
  TF_LITE_ENSURE_STATUS(op_resolver.AddTanh());
  TF_LITE_ENSURE_STATUS(op_resolver.AddRelu());
  TF_LITE_ENSURE_STATUS(op_resolver.AddAdd());
  return kTfLiteOk;
}
}  // namespace

MulticopterNeuralNetworkControl::MulticopterNeuralNetworkControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{

}

MulticopterNeuralNetworkControl::~MulticopterNeuralNetworkControl()
{
	perf_free(_loop_perf);
}


bool MulticopterNeuralNetworkControl::init()
{
	if (!_angular_velocity_sub.registerCallback())
	{
		PX4_ERR("callback registration failed");
		return false;
	}

  	return true;
}

int MulticopterNeuralNetworkControl::InitializeNetwork() {
	// Initialize the neural network
	// Load the model
	const tflite::Model* control_model = ::tflite::GetModel(control_net_tflite);  // TODO: Replace with your model data variable
	const tflite::Model* allocation_model = ::tflite::GetModel(allocation_net_tflite);  // TODO: Replace with your model data variable

	// Set up the interpreter
	static NNControlOpResolver resolver;
	if (RegisterOps(resolver) != kTfLiteOk) {
		PX4_ERR("Failed to register ops");
		return -1;
	}
	constexpr int kTensorArenaSize = 10 * 1024; //TODO: Check this size
	static uint8_t tensor_arena[kTensorArenaSize];
	_control_interpreter = new tflite::MicroInterpreter(control_model, resolver, tensor_arena, kTensorArenaSize);
	_allocation_interpreter = new tflite::MicroInterpreter(allocation_model, resolver, tensor_arena, kTensorArenaSize);

	// Allocate memory for the model's tensors
	TfLiteStatus allocate_status_control = _control_interpreter->AllocateTensors();
	if (allocate_status_control != kTfLiteOk) {
		PX4_ERR("AllocateTensors() failed");
		return -1;
	}
	TfLiteStatus allocate_status = _allocation_interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		PX4_ERR("AllocateTensors() failed");
		return -1;
	}

	_input_tensor = _control_interpreter->input(0);
	if (_input_tensor == nullptr) {
		PX4_ERR("Input tensor is null");
		return -1;
	}
	if (_allocation_interpreter->input(0) == nullptr) {
		PX4_ERR("Input tensor is null");
		return -1;
	}

	return PX4_OK;
}


void MulticopterNeuralNetworkControl::PopulateInputTensor() {
	// Creates a 15 element input tensor for the neural network [pos_err(3), lin_vel(3), att(6), ang_vel(3)]

	// transform observations in correct frame
	matrix::Dcmf frame_transf;
	frame_transf(0, 0) = 1.0f;
	frame_transf(0, 1) = 0.0f;
	frame_transf(0, 2) = 0.0f;
	frame_transf(1, 0) = 0.0f;
	frame_transf(1, 1) = -1.0f;
	frame_transf(1, 2) = 0.0f;
	frame_transf(2, 0) = 0.0f;
	frame_transf(2, 1) = 0.0f;
	frame_transf(2, 2) = -1.0f;

	matrix::Dcmf frame_transf_2;
	frame_transf_2(0, 0) = 0.0f;
	frame_transf_2(0, 1) = 1.0f;
	frame_transf_2(0, 2) = 0.0f;
	frame_transf_2(1, 0) = -1.0f;
	frame_transf_2(1, 1) = 0.0f;
	frame_transf_2(1, 2) = 0.0f;
	frame_transf_2(2, 0) = 0.0f;
	frame_transf_2(2, 1) = 0.0f;
	frame_transf_2(2, 2) = 1.0f;

	matrix::Vector3f position_local = matrix::Vector3f(_position.x, _position.y, _position.z);
	position_local = frame_transf * frame_transf_2 * position_local;

	matrix::Vector3f position_setpoint_local = matrix::Vector3f(_position_setpoint.x, _position_setpoint.y, _position_setpoint.z);
	position_setpoint_local = frame_transf * frame_transf_2 * position_setpoint_local;

	matrix::Vector3f linear_velocity_local = matrix::Vector3f(_position.vx, _position.vy, _position.vz);
	linear_velocity_local = frame_transf * frame_transf_2 * linear_velocity_local;

	matrix::Quatf attitude = matrix::Quatf(_attitude.q);
	matrix::Dcmf _attitude_local_mat = frame_transf * (frame_transf_2 * matrix::Dcmf(attitude)) * frame_transf.transpose();

	matrix::Vector3f angular_vel_local = matrix::Vector3f( _angular_velocity.xyz[0], _angular_velocity.xyz[1], _angular_velocity.xyz[2]);
	angular_vel_local = frame_transf * angular_vel_local;

	_input_tensor->data.f[0] = position_setpoint_local(0) - position_local(0);
	_input_tensor->data.f[1] = position_setpoint_local(1) - position_local(1);
	_input_tensor->data.f[2] = position_setpoint_local(2) - position_local(2);
	_input_tensor->data.f[3] = _attitude_local_mat(0, 0);
	_input_tensor->data.f[4] = _attitude_local_mat(0, 1);
	_input_tensor->data.f[5] = _attitude_local_mat(0, 2);
	_input_tensor->data.f[6] = _attitude_local_mat(1, 0);
	_input_tensor->data.f[7] = _attitude_local_mat(1, 1);
	_input_tensor->data.f[8] = _attitude_local_mat(1, 2);
	_input_tensor->data.f[9] = linear_velocity_local(0);
	_input_tensor->data.f[10] = linear_velocity_local(1);
	_input_tensor->data.f[11] = linear_velocity_local(2);
	_input_tensor->data.f[12] = angular_vel_local(0);
	_input_tensor->data.f[13] = angular_vel_local(1);
	_input_tensor->data.f[14] = angular_vel_local(2);

}

void MulticopterNeuralNetworkControl::PublishOutput(float* command_actions) {

	actuator_motors_s actuator_motors;
        actuator_motors.timestamp = hrt_absolute_time();

	actuator_motors.control[0] = PX4_ISFINITE(command_actions[0]) ? command_actions[0] : NAN;
	actuator_motors.control[1] = PX4_ISFINITE(command_actions[2]) ? command_actions[2] : NAN;
	actuator_motors.control[2] = PX4_ISFINITE(command_actions[3]) ? command_actions[3] : NAN;
	actuator_motors.control[3] = PX4_ISFINITE(command_actions[1]) ? command_actions[1] : NAN;
        actuator_motors.control[4] = -NAN;
        actuator_motors.control[5] = -NAN;
        actuator_motors.control[6] = -NAN;
        actuator_motors.control[7] = -NAN;
        actuator_motors.control[8] = -NAN;
        actuator_motors.control[9] = -NAN;
        actuator_motors.control[10] = -NAN;
        actuator_motors.control[11] = -NAN;
	actuator_motors.reversible_flags = 0;

	_actuator_motors_pub.publish(actuator_motors);
}



inline void MulticopterNeuralNetworkControl::RescaleActions() {
	//static const float _thrust_coefficient = 0.00001286412;
	static const float _thrust_coefficient = 0.00001006412;
	const float a = 0.8f;
  	const float b = (1.f - 0.8f);
	const float tmp1 = b / (2.f * a);
	const float tmp2 = b * b / (4.f * a * a);
	const int max_rpm = 22000.0f;
	const int min_rpm = 1000.0f;
	for (int i = 0; i < 4; i++) {
		if (_output_tensor->data.f[i] < 0.2f){
			_output_tensor->data.f[i] = 0.2f;
		}
		else if (_output_tensor->data.f[i] > 1.2f){
			_output_tensor->data.f[i] = 1.2f;
		}
		float rps = _output_tensor->data.f[i]/_thrust_coefficient;
		rps = sqrt(rps);
		float rpm = rps * 60.0f;
		_output_tensor->data.f[i] = (rpm*2.0f - max_rpm - min_rpm) / (max_rpm - min_rpm);
		_output_tensor->data.f[i] = a * (((_output_tensor->data.f[i] + 1.0f) / 2.0f + tmp1) * ((_output_tensor->data.f[i] + 1.0f) / 2.0f + tmp1) - tmp2);
	}
}


int MulticopterNeuralNetworkControl::task_spawn(int argc, char *argv[])
{
	// This function loads the model, sets up the interpreter, allocates memory for the model's tensors, and prepares the input data.
	MulticopterNeuralNetworkControl *instance = new MulticopterNeuralNetworkControl();

	if (instance){
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() and instance->InitializeNetwork() == PX4_OK) {
			return PX4_OK;
		}
		else {
			PX4_ERR("init failed");
		}
	} else {
		PX4_ERR("alloc failed");
	}
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void MulticopterNeuralNetworkControl::Run()
{
	if (should_exit())
	{
		_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	#ifdef __PX4_NUTTX
		hrt_abstime start_time1 = hrt_absolute_time();
	#else
		auto start_time1 = std::chrono::high_resolution_clock::now();
	#endif

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_control_mode_s vehicle_control_mode;
	if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
		_use_neural = vehicle_control_mode.flag_control_neural_enabled;
	}

	if(!_use_neural) {
		// If the neural network flight mode is not enabled, do nothing
		return;
	}

	// run controller on angular velocity updates
	if (_angular_velocity_sub.update(&_angular_velocity)) {
		_last_run = _angular_velocity.timestamp_sample;

		if(_attitude_sub.updated()) {
			_attitude_sub.copy(&_attitude);
		}
		if(_position_sub.updated()) {
			_position_sub.copy(&_position);
		}
		if(_position_setpoint_sub.updated()) {
			_position_setpoint_sub.copy(&_position_setpoint);
		}

		PopulateInputTensor();

		// Run inference
		#ifdef __PX4_NUTTX
        		hrt_abstime start_time2 = hrt_absolute_time();
		#else
			auto start_time2 = std::chrono::high_resolution_clock::now();
		#endif
		// Inference
		TfLiteStatus invoke_status_control = _control_interpreter->Invoke();
		#ifdef __PX4_NUTTX
			hrt_abstime inference_time_control = hrt_absolute_time() - start_time2;
		#else
			auto inference_time_control = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time2).count();
		#endif
		if (invoke_status_control != kTfLiteOk) {
			PX4_ERR("Invoke() failed");
			return;
		}
		// Print the output
		TfLiteTensor* control_output_tensor = _control_interpreter->output(0);
		if (control_output_tensor == nullptr) {
			PX4_ERR("Output tensor is null");
			return;
		}

		TfLiteTensor* allocation_input_tensor = _allocation_interpreter->input(0);
		// rescale actions
		allocation_input_tensor->data.f[0] = control_output_tensor->data.f[0] * 0;
		allocation_input_tensor->data.f[1] = control_output_tensor->data.f[1] * 0;
		allocation_input_tensor->data.f[2] = control_output_tensor->data.f[2] * 2.0f + 2.8f;
		allocation_input_tensor->data.f[3] = control_output_tensor->data.f[3] * 0.32f;
		allocation_input_tensor->data.f[4] = control_output_tensor->data.f[4] * 0.32f;
		allocation_input_tensor->data.f[5] = control_output_tensor->data.f[5] * 0.02f;

		#ifdef __PX4_NUTTX
			hrt_abstime start_time3 = hrt_absolute_time();
		#else
			auto start_time3 = std::chrono::high_resolution_clock::now();
		#endif
		TfLiteStatus invoke_status = _allocation_interpreter->Invoke();
		#ifdef __PX4_NUTTX
			hrt_abstime inference_time_allocation = hrt_absolute_time() - start_time3;
		#else
			auto inference_time_allocation = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time3).count();
		#endif
		if (invoke_status != kTfLiteOk) {
			PX4_ERR("Invoke() failed");
			return;
		}
		_output_tensor = _allocation_interpreter->output(0);
		if (_output_tensor == nullptr) {
			PX4_ERR("Output tensor is null");
			return;
		}

		// Convert the output tensor to actuator values
		RescaleActions();

		// Publish the actuator values
		PublishOutput(_output_tensor->data.f);


		#ifdef __PX4_NUTTX
			hrt_abstime full_controller_time = hrt_absolute_time() - start_time1;
		#else
			auto full_controller_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time1).count();
		#endif

		// Publish the neural control debug message
		neural_control_s neural_control;
		neural_control.timestamp = hrt_absolute_time();
		neural_control.control_inference_time = static_cast<int32_t>(inference_time_control);
		neural_control.controller_time = static_cast<int32_t>(full_controller_time);
		neural_control.allocation_inference_time = static_cast<int32_t>(inference_time_allocation);
		for (int i = 0; i < 15; i++) {
			neural_control.observation[i] = _input_tensor->data.f[i];
		}
		for (int i = 0; i < 6; i++) {
			neural_control.wrench[i] = allocation_input_tensor->data.f[i];
		}
		neural_control.motor_thrust[0] = _output_tensor->data.f[0];
		neural_control.motor_thrust[1] = _output_tensor->data.f[2];
		neural_control.motor_thrust[2] = _output_tensor->data.f[3];
		neural_control.motor_thrust[3] = _output_tensor->data.f[1];
		_neural_control_pub.publish(neural_control);
	}
	perf_end(_loop_perf);
}

int MulticopterNeuralNetworkControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterNeuralNetworkControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Multicopter Neural Network Control module.
This module is an end-to-end neural network control system for multicopters.
It takes in 15 input values and outputs 4 control actions.
Inputs: [pos_err(3), att(6), vel(3), ang_vel(3)]
Outputs: [Actuator motors(4)]
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_nn_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_nn_control_main(int argc, char *argv[])
{
	return MulticopterNeuralNetworkControl::main(argc, argv);
}
