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

	switch (_param_mc_airmode.get()) {
	case 1:
		mix(/*airmode_rp=*/ true, /*airmode_yaw=*/ false);
		break;

	case 2:
		mix(/*airmode_rp=*/ true, /*airmode_yaw=*/ true);
		break;

	default:
		mix(/*airmode_rp=*/ false, /*airmode_yaw=*/ false);
		break;
	}
}

void ControlAllocationSequentialDesaturation::desaturateActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	if (increase_only && gain < 0.f) {
		return;
	}

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}

	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

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
ControlAllocationSequentialDesaturation::mix(bool airmode_rp, bool airmode_yaw)
{
	ActuatorVector thrust_z;
	ActuatorVector roll;
	ActuatorVector pitch;
	ActuatorVector yaw;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  (airmode_yaw
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

	// Slide thrust to unsaturate roll/pitch (and yaw when in the sum).
	// Without airmode_rp, thrust may only decrease.
	desaturateActuators(_actuator_sp, thrust_z, !airmode_rp);

	if (!airmode_rp) {
		// Reduce roll/pitch acceleration if needed to unsaturate.
		desaturateActuators(_actuator_sp, roll);
		desaturateActuators(_actuator_sp, pitch);
	}

	if (airmode_yaw) {
		// Yaw is already in the sum; deprioritize it relative to roll/pitch by
		// desaturating bidirectionally without the MINIMUM_YAW_MARGIN inflation.
		desaturateActuators(_actuator_sp, yaw);

	} else {
		// Add yaw to outputs.
		for (int i = 0; i < _num_actuators; i++) {
			_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		}

		// Inflate the upper bound by MINIMUM_YAW_MARGIN so yaw retains some authority
		// near maximum thrust, then desaturate yaw against the inflated bound.
		const ActuatorVector max_prev = _actuator_max;
		_actuator_max += (_actuator_max - _actuator_min) * MINIMUM_YAW_MARGIN;
		desaturateActuators(_actuator_sp, yaw);
		_actuator_max = max_prev;

		// Reduce thrust only to clean up any overshoot the inflation allowed.
		desaturateActuators(_actuator_sp, thrust_z, true);
	}
}

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}
