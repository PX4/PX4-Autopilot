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
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilts(this)
{
}

bool
ActuatorEffectivenessTiltingMultirotor::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	const auto &geometry = _mc_rotors.geometry();
	const int num_rotors = geometry.num_rotors;

	configuration.selected_matrix = 0;

	Vector3f torque_vert, force_vert, torque_lat, force_lat;
	const Vector3f axis_vert{0.0f, 0.0f, -1.0f};

	for (int i = 0; i < num_rotors; ++i) {

		const Vector3f &pos = geometry.rotors[i].position;
		const float ct = geometry.rotors[i].thrust_coef;
		const float km = geometry.rotors[i].moment_ratio;

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		const float rotor_angle = atan2f(pos(1), pos(0));

		const Vector3f axis_lat = ActuatorEffectivenessRotors::tiltedAxis(M_PI_2_F, rotor_angle + M_PI_2_F);

		computeRotorWrench(pos, axis_vert, ct, km, torque_vert, force_vert);
		computeRotorWrench(pos, axis_lat,  ct, km, torque_lat,  force_lat);

		configuration.addActuator(ActuatorType::MOTORS, torque_vert, force_vert);
		configuration.addActuator(ActuatorType::MOTORS, torque_lat, force_lat);
	}

	configuration.selected_matrix = 1;
	configuration.actuatorsAdded(ActuatorType::SERVOS, num_rotors);

	return true;
}

void
ActuatorEffectivenessTiltingMultirotor::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	if (matrix_index == 0) {
		const ActuatorVector force_sp = actuator_sp; // virtual force setpoint
		actuator_sp.zero();

		const int num_motors = _mc_rotors.geometry().num_rotors;

		for (int i = 0; i < num_motors; ++i) {
			const float f_vert = force_sp(2 * i);
			const float f_lat  = force_sp(2 * i + 1);

			actuator_sp(i) = hypotf(f_vert, f_lat);

			const int servo_idx = _mc_rotors.geometry().rotors[i].tilt_index;

			const float tilt_angle_rad = atan2f(f_lat, f_vert);

			const auto &servo_param = _tilts.config(servo_idx);

			_servo_sp(servo_idx) = math::interpolate(
						       math::constrain(tilt_angle_rad, servo_param.min_angle, servo_param.max_angle),
						       servo_param.min_angle, servo_param.max_angle, -1.f, 1.f);
		}

	} else if (matrix_index == 1) {
		actuator_sp = _servo_sp;
	}
}

void
ActuatorEffectivenessTiltingMultirotor::computeRotorWrench(const Vector3f &pos, const Vector3f &axis,
		float ct, float km,
		Vector3f &torque, Vector3f &force)
{
	torque = pos.cross(axis) * ct - ct * km * axis;
	force  = ct * axis;
}
