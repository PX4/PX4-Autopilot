/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessThrustVectoring.hpp"

using namespace matrix;

ActuatorEffectivenessThrustVectoring::ActuatorEffectivenessThrustVectoring(ModuleParams *parent)
	: ModuleParams(parent),
	  _rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards),
	  _control_surfaces(this)
{
}

bool
ActuatorEffectivenessThrustVectoring::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	_rotors.enablePropellerTorque(false);
	const bool rotors_added_successfully = _rotors.addActuators(configuration);

	// Control Surfaces
	_first_control_surface_idx = configuration.num_actuators_matrix[0];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	return (rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessThrustVectoring::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	esc_status_s esc_status;
	hrt_abstime now = hrt_absolute_time();
	float rpm = 0.f;

	if (_esc_status_sub.copy(&esc_status) && (now < esc_status.timestamp + 1_s)) {

		for (size_t esc = 0; esc < math::min(esc_status.esc_count, (uint8_t)MAX_NUM_ESCS); esc++) {
			const esc_report_s &esc_report = esc_status.esc[esc];

			const bool esc_connected = (esc_status.esc_online_flags & (1 << esc)) || (esc_report.esc_rpm != 0);

			if (esc_connected && (now < esc_report.timestamp + 1_s)) {
				rpm = esc_report.esc_rpm;
				break;
			}
		}
	}

	constexpr float propeller_coefficient = 1.f / (20000.f * 20000.f); // scale down the thrust
	float thrust = rpm * rpm * propeller_coefficient;

	// Use thrust setpoint if there is no ESC telemetry available
	if (thrust < FLT_EPSILON) {
		vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

		if (_vehicle_thrust_setpoint_sub.copy(&vehicle_thrust_setpoint)) {
			thrust = vehicle_thrust_setpoint.xyz[2];
		}
	}

	const float scale = 1.f / fmaxf(thrust, 0.2f);
	_control_surfaces.applyScale(scale, _first_control_surface_idx, actuator_sp);
}
