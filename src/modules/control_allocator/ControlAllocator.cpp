/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	free(_control_allocation);
	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("vehicle_torque_setpoint callback registration failed!");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("vehicle_thrust_setpoint callback registration failed!");
		return false;
	}

	return true;
}

void
ControlAllocator::parameters_updated()
{
	// Allocation method
	// Do this first: in case a new method is loaded, it will be configured below
	update_allocation_method();

	// Minimum actuator values
	matrix::Vector<float, NUM_ACTUATORS> actuator_min;
	actuator_min(0) = _param_ca_act0_min.get();
	actuator_min(1) = _param_ca_act1_min.get();
	actuator_min(2) = _param_ca_act2_min.get();
	actuator_min(3) = _param_ca_act3_min.get();
	actuator_min(4) = _param_ca_act4_min.get();
	actuator_min(5) = _param_ca_act5_min.get();
	actuator_min(6) = _param_ca_act6_min.get();
	actuator_min(7) = _param_ca_act7_min.get();
	actuator_min(8) = _param_ca_act8_min.get();
	actuator_min(9) = _param_ca_act9_min.get();
	actuator_min(10) = _param_ca_act10_min.get();
	actuator_min(11) = _param_ca_act11_min.get();
	actuator_min(12) = _param_ca_act12_min.get();
	actuator_min(13) = _param_ca_act13_min.get();
	actuator_min(14) = _param_ca_act14_min.get();
	actuator_min(15) = _param_ca_act15_min.get();
	_control_allocation->setActuatorMin(actuator_min);

	// Maximum actuator values
	matrix::Vector<float, NUM_ACTUATORS> actuator_max;
	actuator_max(0) = _param_ca_act0_max.get();
	actuator_max(1) = _param_ca_act1_max.get();
	actuator_max(2) = _param_ca_act2_max.get();
	actuator_max(3) = _param_ca_act3_max.get();
	actuator_max(4) = _param_ca_act4_max.get();
	actuator_max(5) = _param_ca_act5_max.get();
	actuator_max(6) = _param_ca_act6_max.get();
	actuator_max(7) = _param_ca_act7_max.get();
	actuator_max(8) = _param_ca_act8_max.get();
	actuator_max(9) = _param_ca_act9_max.get();
	actuator_max(10) = _param_ca_act10_max.get();
	actuator_max(11) = _param_ca_act11_max.get();
	actuator_max(12) = _param_ca_act12_max.get();
	actuator_max(13) = _param_ca_act13_max.get();
	actuator_max(14) = _param_ca_act14_max.get();
	actuator_max(15) = _param_ca_act15_max.get();
	_control_allocation->setActuatorMax(actuator_max);

	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> B = getEffectinvenessMatrix();

	// Assign control effectiveness matrix
	_control_allocation->setEffectivenessMatrix(B);
}

const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS>
ControlAllocator::getEffectinvenessMatrix()
{
	// Control effectiveness
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> B;

	switch (_param_ca_airframe.get()) {

	case 0: {
			// quad_w
			const float B_quad_w[NUM_AXES][NUM_ACTUATORS] = {
				// quad_w
				{-0.5717536f,  0.43756646f,  0.5717536f, -0.43756646f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.35355328f, -0.35355328f,  0.35355328f, -0.35355328f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.28323701f,  0.28323701f, -0.28323701f, -0.28323701f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{-0.25f, -0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			B = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_quad_w);
			break;
		}

	case 1: {
			// hexa_x
			const float B_hexa_x[NUM_AXES][NUM_ACTUATORS] = {
				{-0.333333f,  0.333333f,  0.166667f, -0.166667f, -0.166667f,  0.166667f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.288675f, -0.288675f,  0.288675f, -0.288675f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{-0.166667f,  0.166667f, -0.166667f,  0.166667f,  0.166667f, -0.166667f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f     },
				{ 0.f,  0.f,  0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f    },
				{-0.166667f, -0.166667f, -0.166667f, -0.166667f, -0.166667f, -0.166667f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			};
			B = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_hexa_x);
			break;
		}

	case 2: {
			if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				// standard_vtol_hover
				const float B_standard_vtol_hover[NUM_AXES][NUM_ACTUATORS] = {
					{-0.5,  0.5,  0.5, -0.5, 0.f, 0.0f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.5, -0.5,  0.5, -0.5, 0.f, 0.f, 0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.28323701f,  0.28323701f, -0.28323701f, -0.28323701f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.f,  0.f,  0.f,  0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{-0.25f, -0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
				};
				B = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_standard_vtol_hover);

			} else {
				// standard_vtol_fixed_wing
				const float B_standard_vtol_fixed_wing[NUM_AXES][NUM_ACTUATORS] = {
					{ 0.0, 0.0, 0.0, 0.0, 0.f, -0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.0, 0.0, 0.0, 0.0, 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.0, 0.0, 0.0, 0.0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.f,  0.f,  0.f,  0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
					{ 0.0f, 0.0f, 0.0f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
				};
				B = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_standard_vtol_fixed_wing);
			}

			break;
		}

	default:
		// none
		break;
	}


	matrix::Vector<float, NUM_ACTUATORS> actuator_max = _control_allocation->getActuatorMax();
	matrix::Vector<float, NUM_ACTUATORS> actuator_min = _control_allocation->getActuatorMin();

	// Set 0 effectiveness for actuators that are disabled (act_min >= act_max)
	for (size_t j = 0; j < NUM_ACTUATORS; j++) {
		if (actuator_min(j) >= actuator_max(j)) {
			for (size_t i = 0; i < NUM_AXES; i++) {
				B(i, j) = 0.0f;
			}

		}
	}

	return B;
}

void
ControlAllocator::update_allocation_method()
{
	int method = _param_ca_method.get();

	if (_allocation_method_id != method) {

		// Save current state
		matrix::Vector<float, NUM_ACTUATORS> actuator_sp;

		if (_control_allocation != nullptr) {
			actuator_sp = _control_allocation->getAllocatedActuator();
		}

		// try to instanciate new allocation method
		ControlAllocation *tmp = nullptr;

		switch (method) {
		case 0:
			tmp = new ControlAllocationPseudoInverse();
			break;

		case 1:
			tmp = new ControlAllocationSequentialDesaturation();
			break;

		default:
			PX4_ERR("Unknown allocation method");
			tmp = nullptr;
			break;
		}

		// Replace previous method with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Control allocation init failed");
			_param_ca_method.set(_allocation_method_id);

		} else if (_control_allocation == tmp) {
			// Nothing has changed, this should not happen
			PX4_ERR("Control allocation init error");
			_allocation_method_id = method;

		} else {
			// Successful update
			PX4_INFO("Control allocation init success");

			// Swap allocation methods
			if (_control_allocation != nullptr) {
				free(_control_allocation);
			}

			_control_allocation = tmp;

			// Save method id
			_allocation_method_id = method;

			// Configure new allocation method
			_control_allocation->setCurrentActuatorSetpoint(actuator_sp);
		}
	}

	// Guard against bad initialization
	if (_control_allocation == nullptr) {
		PX4_ERR("Falling back to ControlAllocationPseudoInverse");
		_control_allocation = new ControlAllocationPseudoInverse();
		_allocation_method_id = 0;
	}
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status = {};
		_vehicle_status_sub.update(&vehicle_status);

		if (_vehicle_type != vehicle_status.vehicle_type) {
			_vehicle_type = vehicle_status.vehicle_type;
			matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> B = getEffectinvenessMatrix();

			// Assign control effectiveness matrix
			_control_allocation->setEffectivenessMatrix(B);
		}
	}


	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
	_last_run = now;

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 5_ms) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}

	if (do_update) {

		// Set control setpoint vector
		matrix::Vector<float, NUM_AXES> c;
		c(0) = _torque_sp(0);
		c(1) = _torque_sp(1);
		c(2) = _torque_sp(2);
		c(3) = _thrust_sp(0);
		c(4) = _thrust_sp(1);
		c(5) = _thrust_sp(2);
		_control_allocation->setControlSetpoint(c);

		// Do allocation
		_control_allocation->allocate();

		// Publish actuator setpoint and allocator status
		publish_actuator_setpoint();
		publish_control_allocator_status();

		// Publish on legacy topics for compatibility with
		// the current mixer system and multicopter controller
		// TODO: remove
		publish_legacy_actuator_controls();
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::publish_actuator_setpoint()
{
	matrix::Vector<float, NUM_ACTUATORS> actuator_sp = _control_allocation->getAllocatedActuator();

	vehicle_actuator_setpoint_s vehicle_actuator_setpoint{};
	vehicle_actuator_setpoint.timestamp = hrt_absolute_time();
	vehicle_actuator_setpoint.timestamp_sample = _timestamp_sample;
	actuator_sp.copyTo(vehicle_actuator_setpoint.actuator);

	_vehicle_actuator_setpoint_pub.publish(vehicle_actuator_setpoint);
}

void
ControlAllocator::publish_control_allocator_status()
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// Allocated control
	matrix::Vector<float, NUM_AXES> allocated_control = _control_allocation->getAllocatedControl();
	control_allocator_status.allocated_torque[0] = allocated_control(0);
	control_allocator_status.allocated_torque[1] = allocated_control(1);
	control_allocator_status.allocated_torque[2] = allocated_control(2);
	control_allocator_status.allocated_thrust[0] = allocated_control(3);
	control_allocator_status.allocated_thrust[1] = allocated_control(4);
	control_allocator_status.allocated_thrust[2] = allocated_control(5);

	// Unallocated control
	matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation->getControlSetpoint() - allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// Actuator saturation
	matrix::Vector<float, NUM_ACTUATORS> actuator_sp = _control_allocation->getAllocatedActuator();
	matrix::Vector<float, NUM_ACTUATORS> actuator_min = _control_allocation->getActuatorMin();
	matrix::Vector<float, NUM_ACTUATORS> actuator_max = _control_allocation->getActuatorMax();

	for (size_t i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	_control_allocator_status_pub.publish(control_allocator_status);
}

void
ControlAllocator::publish_legacy_actuator_controls()
{
	// For compatibility with the current mixer system,
	// publish normalized version on actuator_controls_4/5
	actuator_controls_s actuator_controls_4{};
	actuator_controls_s actuator_controls_5{};
	actuator_controls_4.timestamp = hrt_absolute_time();
	actuator_controls_5.timestamp = hrt_absolute_time();
	actuator_controls_4.timestamp_sample = _timestamp_sample;
	actuator_controls_5.timestamp_sample = _timestamp_sample;

	matrix::Vector<float, NUM_ACTUATORS> actuator_sp_normalized = _control_allocation->normalizeActuatorSetpoint();

	for (size_t i = 0; i < 8; i++) {
		actuator_controls_4.control[i] = (PX4_ISFINITE(actuator_sp_normalized(i))) ? actuator_sp_normalized(i) : 0.0f;
		actuator_controls_5.control[i] = (PX4_ISFINITE(actuator_sp_normalized(i + 8))) ? actuator_sp_normalized(i + 8) : 0.0f;
	}

	_actuator_controls_4_pub.publish(actuator_controls_4);
	_actuator_controls_5_pub.publish(actuator_controls_5);
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Allocation method: %d", _allocation_method_id);

	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(MODULE_NAME, "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
