/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessRotorsINDI.cpp
 *
 * Actuator effectivness for model based INDI rate control
 * Modified from ActuatorEffectivenessRotors.cpp
 * TODO: Add tilt support, removed for now
 *
 * @author Rohan Inamdar <rninamdar@wpi.edu>
 */

#include "ActuatorEffectivenessRotorsINDI.hpp"

using namespace matrix;

ActuatorEffectivenessRotorsINDI::ActuatorEffectivenessRotorsINDI(ModuleParams *parent, AxisConfiguration axis_config)
	: ModuleParams(parent), _axis_config(axis_config)
{
	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "MET_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "MET_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "MET_ROTOR%u_IZZ", i);
		_param_handles[i].moment_inertia = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "INDI_G1_ADAPT_%u", i);
		_param_handles[i].g1_adaptive_constant = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "INDI_G2_ADAPT_%u", i);
		_param_handles[i].g2_adaptive_constant = param_find(buffer);
	}

	updateParams();
}

void ActuatorEffectivenessRotorsINDI::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_rotors = math::min(NUM_ACTUATORS, static_cast<int>(_param_rotor_count.get()));

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].axis;

		switch (_axis_config) {
		case AxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case AxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);
		param_get(_param_handles[i].moment_inertia, &_geometry.rotors[i].moment_inertia);

		float g1_value, g2_value;
		param_get(_param_handles[i].g1_adaptive_constant, &g1_value);
		param_get(_param_handles[i].g2_adaptive_constant, &g2_value);
		_G1_adaptive_constants(i, i) = g1_value;
		_G2_adaptive_constants(i, i) = g2_value;
	}

	_adaptive_constants_per_axis = diag(Vector3f(_param_indi_adapt_axis_r.get(), _param_indi_adapt_axis_p.get(),
					    _param_indi_adapt_axis_y.get()));

}

bool
ActuatorEffectivenessRotorsINDI::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[0],
			    configuration.effectiveness_matrices[1],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
	return true;
}

int
ActuatorEffectivenessRotorsINDI::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &G1, EffectivenessMatrix &G2, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			PX4_ERR("Bad axis definition, ignoring rotor %d", i);
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		if (geometry.propeller_torque_disabled) {
			PX4_ERR("Propeller torque disabled, setting moment ratio to 0, this is not supported for INDI rate control");
			km = 0.f;
		}

		if (geometry.propeller_torque_disabled_non_upwards) {
			PX4_ERR("Propeller torque disabled non upwards, setting moment ratio to 0, this is not supported for INDI rate control");
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;

			if (!upwards) {
				km = 0.f;
			}
		}

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrustTorque induced moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Compute gyroscopic moment generated by this rotor
		matrix::Vector3f gyroscopic_moment = geometry.rotors[i].moment_inertia * sign(km) * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			// note that thrust is not needed for indi so it is not included/calculated
			G1(j + 3, i + actuator_start_index) = moment(j);

			G2(j + 3, i + actuator_start_index) = gyroscopic_moment(j);
		}
	}

	PX4_INFO("computeEffectivenessMatrix: Completed with %d actuators", num_actuators);

	// Debug: Print the computed matrices
	PX4_INFO("G1 matrix (moment part):");

	for (int i = 0; i < 3; i++) {
		PX4_INFO("  Row %d: [%.10f, %.10f, %.10f, %.10f]", i,
			 (double)G1(i + 3, 0), (double)G1(i + 3, 1), (double)G1(i + 3, 2), (double)G1(i + 3, 3));
	}

	PX4_INFO("G2 matrix (gyroscopic part):");

	for (int i = 0; i < 3; i++) {
		PX4_INFO("  Row %d: [%.10f, %.10f, %.10f, %.10f]", i,
			 (double)G2(i + 3, 0), (double)G2(i + 3, 1), (double)G2(i + 3, 2), (double)G2(i + 3, 3));
	}

	return num_actuators;
}

uint32_t ActuatorEffectivenessRotorsINDI::getMotors() const
{
	uint32_t motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		motors |= 1u << i;
	}

	return motors;
}

bool
ActuatorEffectivenessRotorsINDI::initializeEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}


	return addActuators(configuration);
}

/**
 * @brief Adapt the effectiveness matrix to the current motor speeds and angular accelerations to account for changes in the system (weigth changes, battery voltage changes, etc.)
 * @details Implementation from "Adaptive Incremental nonlinear Dynamic Inversion for Attitude Contol of Micro Air Vehicles" by Smeur et al
 * @param configuration The configuration object
 * @param delta_motor_speeds The change in motor speeds
 * @param delta_dot_motor_speeds The change in motor speed derivatives
 * @param filtered_angular_accel The filtered angular acceleration
 */
void
ActuatorEffectivenessRotorsINDI::adaptEffectivenessMatrix(Configuration &configuration,
		Vector<float, NUM_ACTUATORS> &delta_motor_speeds,
		Vector<float, NUM_ACTUATORS> &delta_dot_motor_speeds,
		Vector3f &delta_filtered_angular_accel)
{
	// for readability
	// use auto to point to the slice object that acts a reference, and allows to not need to reassign the new values
	Matrix<float, 3, NUM_ACTUATORS> G1_moment = getG1(
				configuration); //G1_moment is the moment part of G1 (the bottom 3 rows)
	Matrix<float, 3, NUM_ACTUATORS> G2_moment = getG2(
				configuration); //G2_moment is the moment part of G2 (the bottom 3 rows)

	// split up the equation into g1 and g2, as recombining G = [G1, G2] is not computationally necessary as done in the paper with the current layout of the effectiveness matricies
	// note: mu2 = apative_constants_per_axis, mu1 is split between the two matricies: g1_adaptive_constants and g2_adaptive_constants
	G1_moment = G1_moment - _adaptive_constants_per_axis * (G1_moment * delta_motor_speeds - delta_filtered_angular_accel) *
		    delta_motor_speeds.transpose() * _G1_adaptive_constants;
	G2_moment = G2_moment - _adaptive_constants_per_axis * (G2_moment * delta_dot_motor_speeds -
			delta_filtered_angular_accel) * delta_dot_motor_speeds.transpose() * _G2_adaptive_constants;

	// Publish adaptation status
	publishAdaptationStatus(delta_motor_speeds, delta_dot_motor_speeds, delta_filtered_angular_accel, G1_moment, G2_moment);

}

void ActuatorEffectivenessRotorsINDI::publishAdaptationStatus(const Vector<float, NUM_ACTUATORS> &delta_motor_speeds,
		const Vector<float, NUM_ACTUATORS> &delta_dot_motor_speeds,
		const Vector3f &delta_filtered_angular_accel,
		const Matrix<float, 3, NUM_ACTUATORS> &G1,
		const Matrix<float, 3, NUM_ACTUATORS> &G2)
{
	_indi_adaptation_status.timestamp = hrt_absolute_time();
	_indi_adaptation_status.timestamp_sample = hrt_absolute_time();

	// Copy G1 matrix (3xN) to flattened array
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < NUM_ACTUATORS; col++) {
			_indi_adaptation_status.g1_matrix[row * _geometry.num_rotors + col] = G1(row, col);
		}
	}

	// Copy G2 matrix (3xN) to flattened array
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < NUM_ACTUATORS; col++) {
			_indi_adaptation_status.g2_matrix[row * _geometry.num_rotors + col] = G2(row, col);
		}
	}

	// Copy rotor speeds and derivatives
	for (int i = 0; i < NUM_ACTUATORS; i++) {
		_indi_adaptation_status.rotor_speeds[i] = delta_motor_speeds(i);
		_indi_adaptation_status.rotor_speed_dots[i] = delta_dot_motor_speeds(i);
	}

	// Copy angular acceleration delta
	_indi_adaptation_status.angular_accel_delta[0] = delta_filtered_angular_accel(0);	// TODO: check if this is correct
	_indi_adaptation_status.angular_accel_delta[1] = delta_filtered_angular_accel(1);	// TODO: check if this is correct
	_indi_adaptation_status.angular_accel_delta[2] = delta_filtered_angular_accel(2);	// TODO: check if this is correct

	// Copy adaptation constants
	_indi_adaptation_status.g1_adaptive_constants[0] = _G1_adaptive_constants(0, 0);
	_indi_adaptation_status.g1_adaptive_constants[1] = _G1_adaptive_constants(1, 1);
	_indi_adaptation_status.g1_adaptive_constants[2] = _G1_adaptive_constants(2, 2);
	_indi_adaptation_status.g2_adaptive_constants[0] = _G2_adaptive_constants(0, 0);
	_indi_adaptation_status.g2_adaptive_constants[1] = _G2_adaptive_constants(1, 1);
	_indi_adaptation_status.g2_adaptive_constants[2] = _G2_adaptive_constants(2, 2);

	_indi_adaptation_status.adaptation_enabled = true;
	_indi_adaptation_status.num_actuators = _geometry.num_rotors;

	_indi_adaptation_status_pub.publish(_indi_adaptation_status);
}
