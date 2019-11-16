/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationMultirotor.hpp
 *
 * Control Allocation Algorithm for multirotors
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocationMultirotor.hpp"

void
ControlAllocationMultirotor::setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &B)
{
	_B = B;

	// Indicate that mixer needs update
	if (_mixer != nullptr) {
		free(_mixer);
		_mixer = nullptr;
	}
}


void
ControlAllocationMultirotor::allocate()
{
	// Update mixer if needed
	if (_mixer == nullptr) {
		// Compute pseudoinverse of effectiveness matrix
		matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> A = matrix::geninv(_B);

		// Convert A to MultirotorMixer::Rotor
		MultirotorMixer::Rotor rotors[NUM_ACTUATORS];

		for (size_t i = 0; i < NUM_ACTUATORS; i++) {
			rotors[i].roll_scale = A(i, 0);
			rotors[i].pitch_scale = A(i, 1);
			rotors[i].yaw_scale = A(i, 2);
			rotors[i].thrust_scale = -A(i, 5); // -Z thrust
		}

		_mixer = new MultirotorMixer(mixer_callback, (uintptr_t)this, rotors, NUM_ACTUATORS);
	}

	// Allocate
	if (_mixer != nullptr) {
		float outputs[NUM_ACTUATORS];
		_mixer->mix(outputs, NUM_ACTUATORS);
		_actuator_sp = matrix::Vector<float, NUM_ACTUATORS>(outputs);

	} else {
		_actuator_sp *= 0.0f;
	}

	// Clip
	_actuator_sp = clipActuatorSetpoint();

	// Compute achieved control
	_control_allocated = _B * _actuator_sp;
}

int
ControlAllocationMultirotor::mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index,
		float &control)
{
	matrix::Vector<float, NUM_AXES> control_sp = ((ControlAllocationMultirotor *)(handle))->getControlSetpoint();

	switch (control_index) {
	case 0:
	case 1:
	case 2:
		control = control_sp(control_index);
		break;

	case 3:
		control = -control_sp(5); // -Z thrust
		break;

	default:
		control = 0.0f;
		break;
	}

	return 0;
}
