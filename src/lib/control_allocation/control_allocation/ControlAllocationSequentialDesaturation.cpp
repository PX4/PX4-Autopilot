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

	mix(_param_mc_airmode_lim.get(), _param_mc_airmode_yaw.get());
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

void ControlAllocationSequentialDesaturation::desaturateYaw(ActuatorVector &actuator_sp, const ActuatorVector &yaw)
{
	// A gain k along the yaw column turns the delivered yaw command into yaw_sp + k. Keep it
	// between yaw_sp and 0.
	const float yaw_sp = _control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW);
	const float gain_min = fminf(-yaw_sp, 0.f);
	const float gain_max = fmaxf(-yaw_sp, 0.f);

	const float gain = fmaxf(gain_min, fminf(gain_max, computeDesaturationGain(yaw, actuator_sp)));

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * yaw(i);
	}

	// Same half-step refinement as desaturateActuators, within what is left of the range.
	const float refinement = fmaxf(gain_min - gain, fminf(gain_max - gain, 0.5f * computeDesaturationGain(yaw,
				       actuator_sp)));

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += refinement * yaw(i);
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

float ControlAllocationSequentialDesaturation::computeWorstSaturation(const ActuatorVector &desaturation_vector,
		const ActuatorVector &actuator_sp)
{
	float worst = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		// Same weak-effectiveness cutoff as computeDesaturationGain
		if (fabsf(desaturation_vector(i)) < 0.2f) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			worst = fmaxf(worst, fabsf((_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i)));
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			worst = fmaxf(worst, fabsf((_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i)));
		}
	}

	return worst;
}

bool ControlAllocationSequentialDesaturation::yawReducesAirmodeThrust()
{
	ActuatorVector thrust_z;
	ActuatorVector mixed_no_yaw;
	ActuatorVector mixed_with_yaw;

	for (int i = 0; i < _num_actuators; i++) {
		const float base = _actuator_trim(i) +
				   _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				   _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				   _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				   _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				   _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		mixed_no_yaw(i) = base;
		mixed_with_yaw(i) = base + _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	// Compare the worst violation, not computeDesaturationGain(): when the most saturated outputs on
	// the two bounds spin in opposite directions, yaw moves both by the same amount and their gains
	// cancel, so that sum is an exact tie across whole regions of commands and float rounding would
	// pick the path.
	const float worst_no_yaw = computeWorstSaturation(thrust_z, mixed_no_yaw);
	const float worst_with_yaw = computeWorstSaturation(thrust_z, mixed_with_yaw);

	return worst_with_yaw < worst_no_yaw - YAW_FOLD_MIN_IMPROVEMENT;
}

void
ControlAllocationSequentialDesaturation::mix(float roll_pitch_limit, bool yaw_airmode)
{
	// Yaw airmode spends the roll_pitch_limit budget, so without a budget it has nothing to do and
	// roll_pitch_limit == 0 stays airmode disabled regardless of yaw_airmode.
	bool yaw_in_sum = yaw_airmode && (roll_pitch_limit > 0.f);

	// Deferred-yaw regime: fold yaw into the pre-thrust sum when that lets the thrust step relieve
	// the saturating actuator instead of over-adding collective (see yawReducesAirmodeThrust).
	if (!yaw_in_sum && roll_pitch_limit > 0.f) {
		yaw_in_sum = yawReducesAirmodeThrust();
	}

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

	if (yaw_in_sum) {
		// Yaw is already in the sum and is the least important axis: give it up first, before
		// roll/pitch, and without the MINIMUM_YAW_MARGIN inflation.
		desaturateYaw(_actuator_sp, yaw);
	}

	if (roll_pitch_limit < 1.f) {
		// Reduce roll/pitch acceleration if any saturation remains; at roll_pitch_limit == 1
		// these passes are skipped so the full-airmode endpoint stays exact.
		desaturateActuators(_actuator_sp, roll);
		desaturateActuators(_actuator_sp, pitch);
	}

	if (!yaw_in_sum) {
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
		desaturateActuators(_actuator_sp, thrust_z, 0.f);
	}
}

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}
