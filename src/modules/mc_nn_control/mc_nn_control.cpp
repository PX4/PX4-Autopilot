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
* Multicopter Neural Network Control module, from position setpoints to actuator motors.
*
* @author Sindre Meyer Hegre <sindre.hegre@gmail.com>
* @author Welf Rehberg <welf.rehberg@ntnu.no>
*/

#include "mc_nn_control.hpp"
#ifdef __PX4_NUTTX
#include <drivers/drv_hrt.h>
#else
#include <chrono>
#endif

namespace
{
// This number should be the number of operations in the model, like tanh and fully connected
using NNControlOpResolver = tflite::MicroMutableOpResolver<3>;

TfLiteStatus RegisterOps(NNControlOpResolver &op_resolver)
{
	// Add the operations to you need to the op_resolver
	TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
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
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int MulticopterNeuralNetworkControl::InitializeNetwork()
{
	// Initialize the neural network
	const tflite::Model *control_model = ::tflite::GetModel(control_net_tflite);

	// Set up the interpreter
	static NNControlOpResolver resolver;

	if (RegisterOps(resolver) != kTfLiteOk) {
		PX4_ERR("Failed to register ops");
		return -1;
	}

	constexpr int kTensorArenaSize = 10 * 1024;
	static uint8_t tensor_arena[kTensorArenaSize];
	_interpreter = new tflite::MicroInterpreter(control_model, resolver, tensor_arena, kTensorArenaSize);

	// Allocate memory for the model's tensors
	TfLiteStatus allocate_status = _interpreter->AllocateTensors();

	if (allocate_status != kTfLiteOk) {
		PX4_ERR("AllocateTensors() failed");
		return -1;
	}

	_input_tensor = _interpreter->input(0);

	if (_input_tensor == nullptr) {
		PX4_ERR("Input tensor is null");
		return -1;
	}

	return PX4_OK;
}

int32_t MulticopterNeuralNetworkControl::GetTime()
{
#ifdef __PX4_NUTTX
	return static_cast<int32_t>(hrt_absolute_time());
#else
	return static_cast<int32_t>(std::chrono::duration_cast<std::chrono::microseconds>
				    (std::chrono::system_clock::now().time_since_epoch()).count());
#endif
}

void MulticopterNeuralNetworkControl::RegisterNeuralFlightMode()
{
	// Register the neural flight mode with the commander
	register_ext_component_request_s register_ext_component_request{};
	register_ext_component_request.timestamp = hrt_absolute_time();
	strncpy(register_ext_component_request.name, "Neural Control", sizeof(register_ext_component_request.name) - 1);
	register_ext_component_request.request_id = _mode_request_id;
	register_ext_component_request.px4_ros2_api_version = 1;
	register_ext_component_request.register_arming_check = true;
	register_ext_component_request.register_mode = true;
	_register_ext_component_request_pub.publish(register_ext_component_request);
}


void MulticopterNeuralNetworkControl::UnregisterNeuralFlightMode(int8 arming_check_id, int8 mode_id)
{
	// Unregister the neural flight mode with the commander
	unregister_ext_component_s unregister_ext_component{};
	unregister_ext_component.timestamp = hrt_absolute_time();
	strncpy(unregister_ext_component.name, "Neural Control", sizeof(unregister_ext_component.name) - 1);
	unregister_ext_component.arming_check_id = arming_check_id;
	unregister_ext_component.mode_id = mode_id;
	_unregister_ext_component_pub.publish(unregister_ext_component);
}


void MulticopterNeuralNetworkControl::ConfigureNeuralFlightMode(int8 mode_id)
{
	// Configure the neural flight mode with the commander
	vehicle_control_mode_s config_control_setpoints{};
	config_control_setpoints.timestamp = hrt_absolute_time();
	config_control_setpoints.source_id = mode_id;
	config_control_setpoints.flag_multicopter_position_control_enabled = false;
	config_control_setpoints.flag_control_manual_enabled = _param_manual_control.get();
	config_control_setpoints.flag_control_offboard_enabled = false;
	config_control_setpoints.flag_control_position_enabled = false;
	config_control_setpoints.flag_control_climb_rate_enabled = true;
	config_control_setpoints.flag_control_allocation_enabled = false;
	config_control_setpoints.flag_control_termination_enabled = true;
	_config_control_setpoints_pub.publish(config_control_setpoints);
}


void MulticopterNeuralNetworkControl::ReplyToArmingCheck(int8 request_id)
{
	// Reply to the arming check request
	arming_check_reply_s arming_check_reply;
	arming_check_reply.timestamp = hrt_absolute_time();
	arming_check_reply.request_id = request_id;
	arming_check_reply.registration_id = _arming_check_id;
	arming_check_reply.health_component_index = arming_check_reply.HEALTH_COMPONENT_INDEX_NONE;
	arming_check_reply.num_events = 0;
	arming_check_reply.can_arm_and_run = true;
	arming_check_reply.mode_req_angular_velocity = true;
	arming_check_reply.mode_req_local_position = true;
	arming_check_reply.mode_req_attitude = true;
	arming_check_reply.mode_req_local_alt = true;
	arming_check_reply.mode_req_home_position = false;
	arming_check_reply.mode_req_mission = false;
	arming_check_reply.mode_req_global_position = false;
	arming_check_reply.mode_req_prevent_arming = false;
	arming_check_reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(arming_check_reply);
}


void MulticopterNeuralNetworkControl::CheckModeRegistration()
{
	register_ext_component_reply_s register_ext_component_reply;
	int tries = register_ext_component_reply.ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&register_ext_component_reply) && --tries >= 0) {
		if (register_ext_component_reply.request_id == _mode_request_id && register_ext_component_reply.success) {
			_arming_check_id = register_ext_component_reply.arming_check_id;
			_mode_id = register_ext_component_reply.mode_id;
			PX4_INFO("NeuralControl mode registration successful, arming_check_id: %d, mode_id: %d", _arming_check_id, _mode_id);
			ConfigureNeuralFlightMode(_mode_id);
			break;
		}
	}
}


void MulticopterNeuralNetworkControl::check_setpoint_validity(vehicle_local_position_s &_position)
{
	const float _setpoint_age = (hrt_absolute_time() - _trajectory_setpoint.timestamp) * 1e-6f;

	if (_setpoint_age < 0.0f || _setpoint_age > 1.0f) {
		reset_trajectory_setpoint(_position);
		PX4_INFO("Age: %.2f s, resetting trajectory setpoint to current position", (double)_setpoint_age);
	}
}

void MulticopterNeuralNetworkControl::reset_trajectory_setpoint(vehicle_local_position_s &_position)
{
	// Reset trajectory setpoint to current position and attitude
	_trajectory_setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint.position[0] = _position.x;
	_trajectory_setpoint.position[1] = _position.y;
	_trajectory_setpoint.position[2] = _position.z;
}

void MulticopterNeuralNetworkControl::generate_trajectory_setpoint(float dt)
{
	// Update position setpoints based on manual control inputs
	float vx_sp = 0.0;

	if (_manual_control_setpoint.pitch > 0.1f
	    || _manual_control_setpoint.pitch < -0.1f) {
		// If pitch is not zero, we use it to set the roll setpoint
		vx_sp = _manual_control_setpoint.pitch * 0.5f;
	}

	float vy_sp = 0.0;

	if (_manual_control_setpoint.roll > 0.1f
	    || _manual_control_setpoint.roll < -0.1f) {
		// If roll is not zero, we use it to set the pitch setpoint
		vy_sp = _manual_control_setpoint.roll * 0.5f;
	}

	float vz_sp = 0.0;

	if (_manual_control_setpoint.throttle > 0.1f
	    || _manual_control_setpoint.throttle < -0.1f) {
		// If throttle is not zero, we use it to set the vertical velocity
		// Note: negative sign due to NED frame
		vz_sp = -_manual_control_setpoint.throttle * 0.5f;
	}

	// Orient setpoint to vehicle
	matrix::Vector3f velocity_setpoint(vx_sp, vy_sp, vz_sp);
	float yaw = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
	matrix::Eulerf euler(0.0, 0.0, yaw);
	matrix::Quatf q_yaw = euler;
	matrix::Vector3f rotated_velocity_setpoint = q_yaw.rotateVector(velocity_setpoint);

	// Build setpoint
	_trajectory_setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint.position[0] = _trajectory_setpoint.position[0] + rotated_velocity_setpoint(
			0) * dt; // X in world frame
	_trajectory_setpoint.position[1] = _trajectory_setpoint.position[1] + rotated_velocity_setpoint(
			1) * dt; // Y in world frame
	_trajectory_setpoint.position[2] = _trajectory_setpoint.position[2] + rotated_velocity_setpoint(
			2) * dt; // Z in world frame
}


void MulticopterNeuralNetworkControl::PopulateInputTensor()
{
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

	// Set default setpoint if NAN
	_trajectory_setpoint.position[0] = PX4_ISFINITE(_trajectory_setpoint.position[0]) ? _trajectory_setpoint.position[0] :
					   0.0f;
	_trajectory_setpoint.position[1] = PX4_ISFINITE(_trajectory_setpoint.position[1]) ? _trajectory_setpoint.position[1] :
					   0.0f;
	_trajectory_setpoint.position[2] = PX4_ISFINITE(_trajectory_setpoint.position[2]) ? _trajectory_setpoint.position[2] :
					   -1.0f;

	matrix::Vector3f position_local = matrix::Vector3f(_position.x, _position.y, _position.z);
	position_local = frame_transf * frame_transf_2 * position_local;

	matrix::Vector3f trajectory_setpoint_local = matrix::Vector3f(_trajectory_setpoint.position[0],
			_trajectory_setpoint.position[1], _trajectory_setpoint.position[2]);
	trajectory_setpoint_local = frame_transf * frame_transf_2 * trajectory_setpoint_local;

	matrix::Vector3f linear_velocity_local = matrix::Vector3f(_position.vx, _position.vy, _position.vz);
	linear_velocity_local = frame_transf * frame_transf_2 * linear_velocity_local;

	matrix::Quatf attitude = matrix::Quatf(_attitude.q);
	matrix::Dcmf _attitude_local_mat = frame_transf * (frame_transf_2 * matrix::Dcmf(attitude)) * frame_transf.transpose();

	matrix::Vector3f angular_vel_local = matrix::Vector3f(_angular_velocity.xyz[0], _angular_velocity.xyz[1],
					     _angular_velocity.xyz[2]);
	angular_vel_local = frame_transf * angular_vel_local;

	_input_tensor->data.f[0] = trajectory_setpoint_local(0) - position_local(0);
	_input_tensor->data.f[1] = trajectory_setpoint_local(1) - position_local(1);
	_input_tensor->data.f[2] = trajectory_setpoint_local(2) - position_local(2);
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

	for (int i = 0; i < 15; i++) {
		_input_data[i] = _input_tensor->data.f[i];
	}

}

void MulticopterNeuralNetworkControl::PublishOutput(float *command_actions)
{

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();

	actuator_motors.control[0] = PX4_ISFINITE(command_actions[0]) ? command_actions[0] : NAN;
	actuator_motors.control[1] = PX4_ISFINITE(command_actions[1]) ? command_actions[1] : NAN;
	actuator_motors.control[2] = PX4_ISFINITE(command_actions[2]) ? command_actions[2] : NAN;
	actuator_motors.control[3] = PX4_ISFINITE(command_actions[3]) ? command_actions[3] : NAN;
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



inline void MulticopterNeuralNetworkControl::RescaleActions()
{
	const float thrust_coeff = _param_thrust_coeff.get() / 100000.0f;
	const float min_rpm = _param_min_rpm.get();
	const float max_rpm = _param_max_rpm.get();
	const float a = 0.8f;
	const float b = (1.0f - 0.8f);
	const float tmp1 = b / (2.f * a);
	const float tmp2 = b * b / (4.f * a * a);

	for (int i = 0; i < 4; i++) {

		if (_output_tensor->data.f[i] < -1.0f) {
			_output_tensor->data.f[i] = -1.0f;

		} else if (_output_tensor->data.f[i] > 1.0f) {
			_output_tensor->data.f[i] = 1.0f;
		}

		_output_tensor->data.f[i] = _output_tensor->data.f[i] + 1.0f;
		float rps = _output_tensor->data.f[i] / thrust_coeff;
		rps = sqrt(rps);
		float rpm = rps * 60.0f;
		_output_tensor->data.f[i] = (rpm * 2.0f - max_rpm - min_rpm) / (max_rpm - min_rpm);
		_output_tensor->data.f[i] = a * (((_output_tensor->data.f[i] + 1.0f) / 2.0f + tmp1) * ((
				_output_tensor->data.f[i] + 1.0f) / 2.0f + tmp1) - tmp2);
	}
}


int MulticopterNeuralNetworkControl::task_spawn(int argc, char *argv[])
{
	// This function loads the model, sets up the interpreter, allocates memory for the model's tensors, and prepares the input data.
	MulticopterNeuralNetworkControl *instance = new MulticopterNeuralNetworkControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() and instance->InitializeNetwork() == PX4_OK) {
			return PX4_OK;

		} else {
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
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();

		if (_sent_mode_registration) {
			UnregisterNeuralFlightMode(_arming_check_id, _mode_id);
		}

		exit_and_cleanup();
		return;
	}

	// Register the flight mode with the commander
	if (!_sent_mode_registration) {
		RegisterNeuralFlightMode();
		_sent_mode_registration = true;
		return;
	}

	// Check if registration was successful
	if (_mode_id == -1 || _arming_check_id == -1) {
		CheckModeRegistration();
		return;
	}

	perf_begin(_loop_perf);

	// Check if an arming check request is received
	if (_arming_check_request_sub.updated()) {
		arming_check_request_s arming_check_request;
		_arming_check_request_sub.copy(&arming_check_request);
		ReplyToArmingCheck(arming_check_request.request_id);
	}

	// Check if navigation mode is set to Neural Control
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&vehicle_status);
		_use_neural = vehicle_status.nav_state == _mode_id;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	if (!_use_neural) {
		// If the neural network flight mode is not enabled, do nothing
		perf_end(_loop_perf);
		return;
	}

	int32_t start_time1 = GetTime();

	// run controller on angular velocity updates
	if (_angular_velocity_sub.update(&_angular_velocity)) {
		const float dt = math::constrain(((_angular_velocity.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = _angular_velocity.timestamp_sample;

		if (_attitude_sub.updated()) {
			_attitude_sub.copy(&_attitude);
		}

		if (_position_sub.updated()) {
			_position_sub.copy(&_position);

			// If there is no position setpoint, use the position when switching mode as the setpoint
			if (!PX4_ISFINITE(_trajectory_setpoint.position[0])
			    && !PX4_ISFINITE(_trajectory_setpoint.position[1])
			    && !PX4_ISFINITE(_trajectory_setpoint.position[2])) {
				reset_trajectory_setpoint(_position);
			}
		}

		if (_param_manual_control.get()) {
			// Run manual control mode
			_manual_control_setpoint_sub.update(&_manual_control_setpoint);

			// Ensure no nan and sufficiently recent setpoint
			check_setpoint_validity(_position);

			// Generate _trajectory_setpoint -> creates _trajectory_setpoint
			generate_trajectory_setpoint(dt);

		} else {
			// Parse offboard trajectory setpoint
			if (_trajectory_setpoint_sub.updated()) {
				trajectory_setpoint_s _trajectory_setpoint_temp;
				_trajectory_setpoint_sub.copy(&_trajectory_setpoint_temp);

				// Make sure the trajectory setpoint is defined before using it
				if (PX4_ISFINITE(_trajectory_setpoint_temp.position[0]) && PX4_ISFINITE(_trajectory_setpoint_temp.position[1]) &&
				    PX4_ISFINITE(_trajectory_setpoint_temp.position[2])) {
					_trajectory_setpoint = _trajectory_setpoint_temp;
				}
			}
		}

		PopulateInputTensor();

		int32_t start_time2 = GetTime();
		TfLiteStatus invoke_status = _interpreter->Invoke();
		int32_t inference_time = GetTime() - start_time2;

		if (invoke_status != kTfLiteOk) {
			PX4_ERR("Invoke() failed");
			return;
		}

		_output_tensor = _interpreter->output(0);

		if (_output_tensor == nullptr) {
			PX4_ERR("Output tensor is null");
			return;
		}

		// Convert the output tensor to actuator values
		RescaleActions();

		PublishOutput(_output_tensor->data.f);

		int32_t full_controller_time = GetTime() - start_time1;

		// Publish the neural control debug message
		neural_control_s neural_control;
		neural_control.timestamp = hrt_absolute_time();
		neural_control.inference_time = inference_time;
		neural_control.controller_time = full_controller_time;

		for (int i = 0; i < 15; i++) {
			neural_control.observation[i] = _input_data[i];
		}

		neural_control.network_output[0] = _output_tensor->data.f[0];
		neural_control.network_output[1] = _output_tensor->data.f[1];
		neural_control.network_output[2] = _output_tensor->data.f[2];
		neural_control.network_output[3] = _output_tensor->data.f[3];
		_neural_control_pub.publish(neural_control);
	}

	perf_end(_loop_perf);
}

int MulticopterNeuralNetworkControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterNeuralNetworkControl::print_status()
{
	if (_mode_id == -1) {
		PX4_INFO("Neural control flight mode: Mode registration failed");
		PX4_INFO("Neural control flight mode: Request sent: %d", _sent_mode_registration);

	} else {
		PX4_INFO("Neural control flight mode: Registered, mode id: %d, arming check id: %d", _mode_id, _arming_check_id);
	}

	return 0;
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
