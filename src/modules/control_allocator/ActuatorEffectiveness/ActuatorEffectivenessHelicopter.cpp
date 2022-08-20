/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessHelicopter.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessHelicopter::ActuatorEffectivenessHelicopter(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
		_param_handles.swash_plate_servos[i].angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
		_param_handles.swash_plate_servos[i].arm_length = param_find(buffer);
	}

	_param_handles.num_swash_plate_servos = param_find("CA_SP0_COUNT");

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve[i] = param_find(buffer);
	}

	updateParams();
}

void ActuatorEffectivenessHelicopter::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_param_handles.num_swash_plate_servos, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_swash_plate_servos = math::constrain((int)count, 3, NUM_SWASH_PLATE_SERVOS_MAX);

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		float angle_deg{};
		param_get(_param_handles.swash_plate_servos[i].angle, &angle_deg);
		_geometry.swash_plate_servos[i].angle = math::radians(angle_deg);
		param_get(_param_handles.swash_plate_servos[i].arm_length, &_geometry.swash_plate_servos[i].arm_length);
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i], &_geometry.throttle_curve[i]);
		param_get(_param_handles.pitch_curve[i], &_geometry.pitch_curve[i]);
	}
}

bool
ActuatorEffectivenessHelicopter::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// Tail (yaw) servo
	configuration.addActuator(ActuatorType::SERVOS, Vector3f{0.f, 0.f, 1.f}, Vector3f{});

	// N swash plate servos
	_first_swash_plate_servo_index = configuration.num_actuators_matrix[0];

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessHelicopter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp)
{
	// Find index to use for curves
	float thrust_cmd = -control_sp(ControlAxis::THRUST_Z);
	float num_intervals = NUM_CURVE_POINTS - 1;
	// We access idx + 1 below, so max legal index is (size - 2)
	int idx = math::constrain((int)(thrust_cmd * num_intervals), 0, NUM_CURVE_POINTS - 2);

	// Local throttle curve gradient and offset
	float tg = (_geometry.throttle_curve[idx + 1] - _geometry.throttle_curve[idx]) * num_intervals;
	float to = (_geometry.throttle_curve[idx]) - (tg * idx / num_intervals);
	float throttle = math::constrain(tg * thrust_cmd + to, 0.0f, 1.0f);

	// Local pitch curve gradient and offset
	float pg = (_geometry.pitch_curve[idx + 1] - _geometry.pitch_curve[idx]) * num_intervals;
	float po = (_geometry.pitch_curve[idx]) - (pg * idx / num_intervals);
	float collective_pitch = math::constrain((pg * thrust_cmd + po), -0.5f, 0.5f);

	float roll_cmd = control_sp(ControlAxis::ROLL);
	float pitch_cmd = control_sp(ControlAxis::PITCH);

	actuator_sp(0) = throttle;

	for (int i = 0; i < _geometry.num_swash_plate_servos; i++) {
		actuator_sp(_first_swash_plate_servo_index + i) = collective_pitch
				+ cosf(_geometry.swash_plate_servos[i].angle) * pitch_cmd * _geometry.swash_plate_servos[i].arm_length
				- sinf(_geometry.swash_plate_servos[i].angle) * roll_cmd * _geometry.swash_plate_servos[i].arm_length;
	}

}
