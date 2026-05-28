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
 * @file ControlAllocationSequentialDesaturation.cpp
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "ControlAllocationSequentialDesaturation.hpp"


void
ControlAllocationSequentialDesaturation::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	mix(_param_mc_airmode_lim.get(), _param_mc_airmode_yaw_lim.get());
}

void ControlAllocationSequentialDesaturation::desaturateActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, float increase_limit)
{
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	// Negative gain raises setpoints (on the thrust axis, adds collective). increase_limit caps the
	// total upward excursion over both passes: 0 forbids it, (0,1) bounds it, >= 1 is unbounded.
	if (gain < 0.f) {
		if (increase_limit <= 0.f) {
			return;
		}

		if (increase_limit < 1.f && gain < -increase_limit) {
			gain = -increase_limit;
		}
	}

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}

	// The refinement pass must respect the same upward budget, otherwise the effective thrust
	// excursion exceeds increase_limit and the limit saturates well before 1.
	const float raised = (gain < 0.f) ? -gain : 0.f;

	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

	if (gain < 0.f && increase_limit > 0.f && increase_limit < 1.f) {
		const float budget = increase_limit - raised;

		if (budget <= 0.f) {
			gain = 0.f;

		} else if (gain < -budget) {
			gain = -budget;
		}
	}

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}
}

float ControlAllocationSequentialDesaturation::computeDesaturationGain(const ActuatorVector &desaturation_vector,
		const ActuatorVector &actuator_sp)
{
	float k_min = 0.f;
	float k_max = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		// Do not use try to desaturate using an actuator with weak effectiveness to avoid large desaturation gains
		if (fabsf(desaturation_vector(i)) < 0.2f) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}
	}

	// Reduce the saturation as much as possible
	return k_min + k_max;
}

void
ControlAllocationSequentialDesaturation::mix(float roll_pitch_limit, float yaw_limit)
{
	const bool yaw_in_sum = (yaw_limit > 0.f);

	ActuatorVector thrust_z;
	ActuatorVector roll;
	ActuatorVector pitch;
	ActuatorVector yaw;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  (yaw_in_sum
				   ? _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW))
				   : 0.f) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		roll(i) = _mix(i, ControlAxis::ROLL);
		pitch(i) = _mix(i, ControlAxis::PITCH);
		yaw(i) = _mix(i, ControlAxis::YAW);
	}

	desaturateActuators(_actuator_sp, thrust_z, roll_pitch_limit);

	if (roll_pitch_limit < 1.f) {
		// Reduce roll/pitch acceleration if any saturation remains; at roll_pitch_limit == 1
		// these passes are skipped so the full-airmode endpoint stays exact.
		desaturateActuators(_actuator_sp, roll);
		desaturateActuators(_actuator_sp, pitch);
	}

	if (yaw_in_sum) {
		// Yaw is already in the sum; deprioritize it relative to roll/pitch by desaturating
		// against the original bounds, capping any upward gain at yaw_limit.
		desaturateActuators(_actuator_sp, yaw, yaw_limit);

	} else {
		// Deferred-yaw path: add yaw after roll/pitch resolve, inflate the upper bound
		// by MINIMUM_YAW_MARGIN to grant a fixed 15% yaw authority near max thrust, then pull
		// thrust back down to clean up any overshoot.
		for (int i = 0; i < _num_actuators; i++) {
			_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		}

		const ActuatorVector max_prev = _actuator_max;
		_actuator_max += (_actuator_max - _actuator_min) * MINIMUM_YAW_MARGIN;
		desaturateActuators(_actuator_sp, yaw);
		_actuator_max = max_prev;

		desaturateActuators(_actuator_sp, thrust_z, 0.f);
	}
}

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}
