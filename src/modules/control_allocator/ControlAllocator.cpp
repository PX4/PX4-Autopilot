/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
	WorkItem(MODULE_NAME, px4::wq_configurations::ctrl_alloc)
{
	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	delete _control_allocation_0;
	delete _control_allocation_1;
	delete _actuator_effectiveness;

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

	if (!_actuator_controls_0_sub.registerCallback()) {
		PX4_ERR("actuator_controls_0 callback registration failed!");
		return false;
	}

	if (!_actuator_controls_1_sub.registerCallback()) {
		PX4_ERR("actuator_controls_1 callback registration failed!");
		return false;
	}

	return true;
}

void
ControlAllocator::parameters_updated()
{
	//TODO: remove custom, as we anyway only have one option
	_actuator_effectiveness = new ActuatorEffectivenessCustom(this);
	update_allocation_method();

	if (_control_allocation_0 == nullptr) {
		return;
	}

	_control_allocation_0->updateParameters();
	_control_allocation_1->updateParameters();
	update_effectiveness_matrices_if_needed(true);
}

void
ControlAllocator::update_allocation_method()
{
	AllocationMethod method = (AllocationMethod)_param_ca_method.get();

	if (_allocation_method_id != method) {

		// Save current state
		matrix::Vector<float, NUM_ACTUATORS> actuator_sp_0;
		matrix::Vector<float, NUM_ACTUATORS> actuator_sp_1;

		if (_control_allocation_0 != nullptr) {
			actuator_sp_0 = _control_allocation_0->getActuatorSetpoint();
		}

		if (_control_allocation_1 != nullptr) {
			actuator_sp_1 = _control_allocation_1->getActuatorSetpoint();
		}

		// try to instanciate new allocation method
		ControlAllocation *tmp_0 = nullptr;
		ControlAllocation *tmp_1 = nullptr;

		switch (method) {
		case AllocationMethod::PSEUDO_INVERSE:
			tmp_0 = new ControlAllocationPseudoInverse();
			tmp_1 = new ControlAllocationPseudoInverse();
			break;

		case AllocationMethod::SEQUENTIAL_DESATURATION:
			tmp_0 = new ControlAllocationSequentialDesaturation();
			tmp_1 = new ControlAllocationSequentialDesaturation();
			break;

		default:
			PX4_ERR("Unknown allocation method");
			break;
		}

		// Replace previous method with new one
		bool failed = false;

		// for allocaton 0:

		if (tmp_0 == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Control allocation init for allocator 0 failed");
			failed = true;

		} else {
			// Swap allocation methods
			delete _control_allocation_0;
			_control_allocation_0 = tmp_0;

			// Configure new allocation method
			_control_allocation_0->setActuatorSetpoint(actuator_sp_0);
		}

		// for allocation 1:

		if (tmp_1 == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Control allocation init for allocator 1 failed");
			failed = true;

		} else {
			// Swap allocation methods
			delete _control_allocation_1;
			_control_allocation_1 = tmp_1;

			// Configure new allocation method
			_control_allocation_1->setActuatorSetpoint(actuator_sp_1);
		}


		if (!failed) {
			// Save method id
			_allocation_method_id = method;

		} else {
			_param_ca_method.set((int)_allocation_method_id);
		}
	}
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		_actuator_controls_0_sub.unregisterCallback();
		_actuator_controls_1_sub.unregisterCallback();
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

	if (_control_allocation_0 == nullptr || _actuator_effectiveness == nullptr) {
		return;
	}

	// TODO: check if flight phase is still needed
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {

		_is_fixed_wing_vehicle = !vehicle_status.is_vtol
					 && vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		_is_vtol = vehicle_status.is_vtol;

		ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

		// Check if the current flight phase is HOVER or FIXED_WING
		if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

		} else {
			flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
		}

		// Special cases for VTOL in transition
		if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
			if (vehicle_status.in_transition_to_fw) {
				flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
			}
		}

		// Forward to effectiveness source
		_actuator_effectiveness->setFlightPhase(flight_phase);
	}

	bool do_update = false;

	if (_actuator_controls_0_sub.update(&_actuator_controls_0)) {
		do_update = true;
		_timestamp_sample = _actuator_controls_0.timestamp_sample;
	}

	if (_actuator_controls_1_sub.update(&_actuator_controls_1)) {
		do_update = true;
		_timestamp_sample = _actuator_controls_1.timestamp_sample;
	}


	if (do_update) {
		_last_run = hrt_absolute_time();;

		update_effectiveness_matrices_if_needed();

		// Set control setpoint vector
		matrix::Vector<float, NUM_AXES> c_0;
		matrix::Vector<float, NUM_AXES> c_1;

		c_0(0) = _actuator_controls_0.control[actuator_controls_s::INDEX_ROLL];
		c_0(1) = _actuator_controls_0.control[actuator_controls_s::INDEX_PITCH];
		c_0(2) = _actuator_controls_0.control[actuator_controls_s::INDEX_YAW];

		c_0(4) = 0.f; // thrust in Y

		if (_is_fixed_wing_vehicle) {
			c_0(3) = _actuator_controls_0.control[actuator_controls_s::INDEX_THROTTLE]; // for pure FW
			c_0(5) = 0.f;

		} else {
			c_0(3) = 0.f;
			c_0(5) = -_actuator_controls_0.control[actuator_controls_s::INDEX_THROTTLE]; // for MC and VTOL
		}

		// VTOL controls
		c_1(0) = _actuator_controls_1.control[actuator_controls_s::INDEX_ROLL];
		c_1(1) = _actuator_controls_1.control[actuator_controls_s::INDEX_PITCH];
		c_1(2) = _actuator_controls_1.control[actuator_controls_s::INDEX_YAW];
		c_1(3) = _actuator_controls_1.control[actuator_controls_s::INDEX_THROTTLE];
		c_1(4) = 0.f;
		c_1(5) = 0.f;

		_control_allocation_0->setControlSetpoint(c_0);
		_control_allocation_1->setControlSetpoint(c_1);

		// Do allocation
		_control_allocation_0->allocate();
		_control_allocation_1->allocate();

		// Publish actuator setpoint and allocator status
		// publish_actuator_setpoint();
		// publish_control_allocator_status();

		// Publish on legacy topics for compatibility with
		// the current mixer system and multicopter controller
		// TODO: remove
		publish_legacy_actuator_controls();
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrices_if_needed(bool force)
{
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> effectiveness_0;
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> effectiveness_1;

	if (_actuator_effectiveness->getEffectivenessMatrix(effectiveness_0, effectiveness_1, force)) {

		const matrix::Vector<float, NUM_ACTUATORS> &trim = _actuator_effectiveness->getActuatorTrim();

		// Set 0 effectiveness for actuators that are disabled (act_min >= act_max)
		// matrix::Vector<float, NUM_ACTUATORS> actuator_max = _control_allocation->getActuatorMax();
		// matrix::Vector<float, NUM_ACTUATORS> actuator_min = _control_allocation->getActuatorMin();

		// for (size_t j = 0; j < NUM_ACTUATORS; j++) {
		// 	if (actuator_min(j) >= actuator_max(j)) {
		// 		for (size_t i = 0; i < NUM_AXES; i++) {
		// 			effectiveness_0(i, j) = 0.0f;
		// 		}
		// 	}
		// }

		// Assign control effectiveness matrix
		_control_allocation_0->setEffectivenessMatrix(effectiveness_0, trim, _actuator_effectiveness->numActuatorsInMatrix_0());
		_control_allocation_1->setEffectivenessMatrix(effectiveness_1, trim, _actuator_effectiveness->numActuatorsInMatrix_1());

		// set min/max of allocation 0
		// set min to -1 if servo and not tilt, set max to 1 for all
		// go through all actuators of output 0

		matrix::Vector<float, NUM_ACTUATORS> actuator_0_min;
		matrix::Vector<float, NUM_ACTUATORS> actuator_0_max;

		//printf("_actuator_effectiveness->numActuators0():%i\n", _actuator_effectiveness->numActuators0());

		for (int i = 0; i < _actuator_effectiveness->numActuators0(); i++) {
			switch (_actuator_effectiveness->get_actuator_type_0(i)) {
			case 1:
				actuator_0_min(i) = 0.f;
				actuator_0_max(i) = 1.f;
				break;

			case 2: // aileron, elevator, rudder, elevons
				actuator_0_min(i) = -1.f;
				actuator_0_max(i) = 1.f;
				break;

			case 3: // tilt - doesn't go through matrix
				// actuator_0_min(i) = 0.f;
				// actuator_0_max(i) = 1.f;

				break;

			case 4: // tilt with yaw
				actuator_0_min(i) = -1.f;
				actuator_0_max(i) = 1.f;

				break;


			case 5: // flaps - doesn't go through matrix
				// actuator_0_min(i) = -1.f;
				// actuator_0_max(i) = 1.f;
				break;

			case 0: //fall through
			default:
				actuator_0_min(i) = 0.f;
				actuator_0_max(i) = 0.f;
				printf("no valid actuator type in allocation 0, i: %i\n", i);
				break;
			}

			// printf("actuator_0_min(%i):%f\n", i, (double)actuator_0_min(i));
		}

		// set min/max of allocation 1
		// set min to -1 if servo and not tilt, set max to 1 for all
		// go through all actuators of output 1
		matrix::Vector<float, NUM_ACTUATORS> actuator_1_min;
		matrix::Vector<float, NUM_ACTUATORS> actuator_1_max;

		for (int i = 0; i < _actuator_effectiveness->numActuators1(); i++) {
			switch (_actuator_effectiveness->get_actuator_type_1(i)) {
			case 1:
				actuator_1_min(i) = 0.f;
				actuator_1_max(i) = 1.f;
				break;

			case 2: // aileron, elevator, rudder, elevons
				actuator_1_min(i) = -1.f;
				actuator_1_max(i) = 1.f;
				break;

			case 3: // tilt - doesn't go through matrix
				// actuator_1_min(i) = 0.f;
				// actuator_1_max(i) = 1.f;

				break;

			case 4: // tilt with yaw
				// actuator_1_min(i) = -1.f;
				// actuator_1_max(i) = 1.f;

				break;

			case 5: // flaps - doesn't go through matrix
				// actuator_1_min(i) = -1.f;
				// actuator_1_max(i) = 1.f;
				break;

			case 0: //fall through
			default:
				actuator_1_min(i) = 0.f;
				actuator_1_max(i) = 0.f;
				printf("no valid actuator type in allocation 0, i: %i\n", i);
				break;
			}

			printf("actuator_1_min(%i):%f\n", i, (double)actuator_1_min(i));
		}

		_control_allocation_0->setActuatorMin(actuator_0_min);
		_control_allocation_1->setActuatorMin(actuator_1_min);

		_control_allocation_0->setActuatorMax(actuator_0_max);
		_control_allocation_1->setActuatorMax(actuator_1_max);

		// printf("num of actuators in matrix 0: %i\n", _actuator_effectiveness->numActuatorsInMatrix_0());
		// printf("num of actuators in matrix 1: %i\n", _actuator_effectiveness->numActuatorsInMatrix_1());
	}
}

// void
// ControlAllocator::publish_actuator_setpoint()
// {
// 	matrix::Vector<float, NUM_ACTUATORS> actuator_sp = _control_allocation->getActuatorSetpoint();

// 	vehicle_actuator_setpoint_s vehicle_actuator_setpoint{};
// 	vehicle_actuator_setpoint.timestamp = hrt_absolute_time();
// 	vehicle_actuator_setpoint.timestamp_sample = _timestamp_sample;
// 	actuator_sp.copyTo(vehicle_actuator_setpoint.actuator);

// 	_vehicle_actuator_setpoint_pub.publish(vehicle_actuator_setpoint);
// }

// void
// ControlAllocator::publish_control_allocator_status()
// {
// 	control_allocator_status_s control_allocator_status{};
// 	control_allocator_status.timestamp = hrt_absolute_time();

// 	// Allocated control
// 	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation->getAllocatedControl();
// 	control_allocator_status.allocated_torque[0] = allocated_control(0);
// 	control_allocator_status.allocated_torque[1] = allocated_control(1);
// 	control_allocator_status.allocated_torque[2] = allocated_control(2);
// 	control_allocator_status.allocated_thrust[0] = allocated_control(3);
// 	control_allocator_status.allocated_thrust[1] = allocated_control(4);
// 	control_allocator_status.allocated_thrust[2] = allocated_control(5);

// 	// Unallocated control
// 	matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation->getControlSetpoint() - allocated_control;
// 	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
// 	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
// 	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
// 	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
// 	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
// 	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

// 	// Allocation success flags
// 	control_allocator_status.torque_setpoint_achieved = (Vector3f(unallocated_control(0), unallocated_control(1),
// 			unallocated_control(2)).norm_squared() < 1e-6f);
// 	control_allocator_status.thrust_setpoint_achieved = (Vector3f(unallocated_control(3), unallocated_control(4),
// 			unallocated_control(5)).norm_squared() < 1e-6f);

// 	// Actuator saturation
// 	const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp = _control_allocation->getActuatorSetpoint();
// 	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min = _control_allocation->getActuatorMin();
// 	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max = _control_allocation->getActuatorMax();

// 	for (size_t i = 0; i < NUM_ACTUATORS; i++) {
// 		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
// 			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

// 		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
// 			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
// 		}
// 	}

// 	_control_allocator_status_pub.publish(control_allocator_status);
// }

void
ControlAllocator::publish_legacy_actuator_controls()
{
	const hrt_abstime now = hrt_absolute_time();

	actuator_motors_s actuator_motors {};
	actuator_motors.timestamp = now;
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos {};
	actuator_servos.timestamp = now;
	actuator_servos.timestamp_sample = _timestamp_sample;

	matrix::Vector<float, NUM_ACTUATORS> actuator_sp_0 = _control_allocation_0->getActuatorSetpoint();
	matrix::Vector<float, NUM_ACTUATORS> actuator_sp_1 = _control_allocation_1->getActuatorSetpoint();

	matrix::Vector<float, NUM_ACTUATORS> actuator_sp_normalized_0 =
		_control_allocation_0->normalizeActuatorSetpoint(actuator_sp_0);
	matrix::Vector<float, NUM_ACTUATORS> actuator_sp_normalized_1 =
		_control_allocation_1->normalizeActuatorSetpoint(actuator_sp_1);

	int index_motor = 0;
	int index_servo = 0;

	// go through all actuators of output 0
	for (int i = 0; i < _actuator_effectiveness->numActuators0(); i++) {
		switch (_actuator_effectiveness->get_actuator_type_0(i)) {
		case 1:
			actuator_motors.control[index_motor] = actuator_sp_normalized_0(i);
			index_motor++;
			break;

		case 2:
			actuator_servos.control[index_servo] = actuator_sp_normalized_0(i);
			index_servo++;
			break;

		case 3: // tilt
			// TODO: add custom offset, scaling and inversion
			actuator_servos.control[index_servo] = _actuator_controls_1.control[4] * 2.f - 1.f;
			index_servo++;
			break;

		case 4: // tilt with yaw control
			// TODO: add custom offset, scaling and inversion, fix indexing (tilts need come first currently)
			actuator_servos.control[index_servo] = _actuator_controls_1.control[4] * 2.f - 1.f + actuator_sp_normalized_0(i);
			index_servo++;
			break;

		case 5: // flaps
			actuator_servos.control[index_servo] = _actuator_controls_0.control[actuator_controls_s::INDEX_FLAPS];
			index_servo++;
			break;

		case 0: //fall through
		default:
			printf("no valid actuator type in allocation 0, i: %i\n", i);
			break;
		}
	}

	// printf("actuator_sp_normalized_1: %f\n", (double)actuator_sp_normalized_1(0));

	// go through all actuators of output 1
	for (int i = 0; i < _actuator_effectiveness->numActuators1(); i++) {
		switch (_actuator_effectiveness->get_actuator_type_1(i)) {
		case 1:
			actuator_motors.control[index_motor] = actuator_sp_normalized_1(i);
			index_motor++;
			break;

		case 2:
			actuator_servos.control[index_servo] = actuator_sp_normalized_1(i);
			index_servo++;
			break;

		// case 3: // tilt
		// 	// TODO: add custom offset, scaling and inversion
		// 	actuator_servos.control[index_servo] = _actuator_controls_1.control[4] * 2.f - 1.f;
		// 	index_servo++;
		// 	break;

		// case 4: // tilt with yaw control
		// 	// TODO: add custom offset, scaling and inversion, fix indexing (tilts need come first currently)
		// 	actuator_servos.control[index_servo] = _actuator_controls_1.control[4] * 2.f - 1.f + actuator_sp_normalized_0(i);
		// 	index_servo++;
		// 	break;

		case 5: // flaps
			// actuator_servos.control[index_servo] = _actuator_controls_0.control[actuator_controls_s::INDEX_FLAPS];
			// index_servo++;
			break;

		case 0: //fall through
		default:
			printf("no valid actuator type in allocation 1\n");
			break;
		}
	}

	_actuator_motors_pub.publish(actuator_motors);
	_actuator_servos_pub.publish(actuator_servos);
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

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;
	}

	// Print current effectiveness matrix
	if (_control_allocation_0 != nullptr) {
		const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness = _control_allocation_0->getEffectivenessMatrix();
		PX4_INFO("Effectiveness.T =");
		effectiveness.T().print();
		PX4_INFO("Configured actuators: %i", _control_allocation_0->numConfiguredActuators());
	}

	// Print perf
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
