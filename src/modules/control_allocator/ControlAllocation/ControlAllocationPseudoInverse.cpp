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
 * @file ControlAllocationPseudoInverse.hpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocationPseudoInverse.hpp"

void
ControlAllocationPseudoInverse::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators);
	_mix_update_needed = true;
}

void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		matrix::geninv(_effectiveness, _mix);
		normalizeControlAllocationMatrix();
		_mix_update_needed = false;
	}
}

void
ControlAllocationPseudoInverse::normalizeControlAllocationMatrix()
{
	if (_normalize_rpy) {
		// Same scale on roll and pitch
		const float roll_norm_sq = _mix.col(0).norm_squared();
		const float pitch_norm_sq = _mix.col(1).norm_squared();
		_control_allocation_scale(0) = sqrtf(fmaxf(roll_norm_sq, pitch_norm_sq) / (_num_actuators / 2.f));
		_control_allocation_scale(1) = _control_allocation_scale(0);

		if (_control_allocation_scale(0) > FLT_EPSILON) {
			_mix.col(0) /= _control_allocation_scale(0);
			_mix.col(1) /= _control_allocation_scale(1);
		}

		// Scale yaw separately
		_control_allocation_scale(2) = _mix.col(2).max();

		if (_control_allocation_scale(2) > FLT_EPSILON) {
			_mix.col(2) /= _control_allocation_scale(2);
		}

	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// Scale thrust by the sum of the norm of the thrust vectors (which is invariant to rotation)
	int num_non_zero_thrust = 0;
	float norm_sum = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		float norm = _mix.slice<1, 3>(i, 3).norm();
		norm_sum += norm;

		if (norm > FLT_EPSILON) {
			++num_non_zero_thrust;
		}
	}

	if (num_non_zero_thrust > 0) {
		norm_sum /= num_non_zero_thrust;
		_control_allocation_scale(3) = norm_sum;
		_control_allocation_scale(4) = norm_sum;
		_control_allocation_scale(5) = norm_sum;
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);

	} else {
		_control_allocation_scale(3) = 1.f;
		_control_allocation_scale(4) = 1.f;
		_control_allocation_scale(5) = 1.f;
	}


	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < 1e-3f) {
				_mix(i, j) = 0.f;
			}
		}
	}
}

void
ControlAllocationPseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
