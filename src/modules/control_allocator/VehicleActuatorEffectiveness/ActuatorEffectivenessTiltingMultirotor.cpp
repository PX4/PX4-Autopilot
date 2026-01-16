/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTiltingMultirotor.hpp
 *
 * Actuator effectiveness for tilting multirotor
 *
 * @author Andrea Berra <andrea.berra@outlook.com>
 */

#include "ActuatorEffectivenessTiltingMultirotor.hpp"

using namespace matrix;

ActuatorEffectivenessTiltingMultirotor::ActuatorEffectivenessTiltingMultirotor(ModuleParams *parent)
	: ModuleParams(parent)
{
	ModuleParams::updateParams();

	if (_mc_rotors) {
		delete _mc_rotors;
		_mc_rotors = nullptr;
	}

	if (_tilts) {
		delete _tilts;
		_tilts = nullptr;
	}

	_mc_rotors = new ActuatorEffectivenessRotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards,
			true, true);
	_tilts = new ActuatorEffectivenessTilts(this);

}

ActuatorEffectivenessTiltingMultirotor::~ActuatorEffectivenessTiltingMultirotor()
{
	if (_tilts) {
		delete _tilts;
		_tilts = nullptr;
	}

	if (_mc_rotors) {
		delete _mc_rotors;
		_mc_rotors = nullptr;
	}
}

bool
ActuatorEffectivenessTiltingMultirotor::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	bool rotors_added_successfully = false;
	bool tilts_added_successfully = false;

	//Vertical forces matrix
	configuration.selected_matrix = 0;
	rotors_added_successfully = _mc_rotors->addActuators(configuration);

	// Tilts
	configuration.selected_matrix = 1;
	tilts_added_successfully = _tilts->addActuators(configuration);

	return rotors_added_successfully && tilts_added_successfully;
}

void
ActuatorEffectivenessTiltingMultirotor::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{

	if (matrix_index == 0) {
		const ActuatorVector &vertlat_actuator_sp = actuator_sp;

		ActuatorVector motor_sp;
		ActuatorVector servo_sp;

		motor_sp.zero();
		servo_sp.zero();

		const int num_motors = _mc_rotors->geometry().num_rotors;
		const int num_servos = _tilts->count();

		for (int i = 0; i < num_motors; ++i) {
			const float vertical_force = vertlat_actuator_sp(2 * i);
			const float lateral_force = vertlat_actuator_sp(2 * i + 1);

			//  motor thrust magnitude
			motor_sp(i) = hypotf(vertical_force, lateral_force);

			int servo_idx = _mc_rotors->geometry().rotors[i].tilt_index;

			if (servo_idx == -1) {
				servo_idx = i;
			}

			if (servo_idx < num_servos) {
				// Prevent numerical instability and servo jitter when the commanded thrust is negligible.
				// The atan2f function is ill-conditioned when both inputs are close to zero
				float tilt_angle_rad = .0f;

				if (motor_sp(i) >= FLT_EPSILON) {
					tilt_angle_rad = atan2f(lateral_force, vertical_force);
				}

				// get config
				const auto &servo_param = _tilts->config(servo_idx);

				// polarity
				tilt_angle_rad *= static_cast<float>(static_cast<int32_t>(servo_param.polarity));

				// saturate
				tilt_angle_rad = math::constrain(tilt_angle_rad, servo_param.min_angle, servo_param.max_angle);

				// map to -+1 range
				servo_sp(servo_idx) = math::interpolate(tilt_angle_rad, servo_param.min_angle, servo_param.max_angle, -1.f, 1.f);
			}
		}

		_cached_servo_sp = servo_sp;
		actuator_sp = motor_sp;

	} else if (matrix_index == 1) {
		actuator_sp = _cached_servo_sp;
	}
}
