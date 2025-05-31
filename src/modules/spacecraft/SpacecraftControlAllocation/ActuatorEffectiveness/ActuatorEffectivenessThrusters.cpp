/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessThrusters.hpp
 *
 * Actuator effectiveness computed from thrusters position and orientation
 *
 * @author Pedro Roque, <padr@kth.se>
 */

#include "ActuatorEffectivenessThrusters.hpp"

using namespace matrix;

ActuatorEffectivenessThrusters::ActuatorEffectivenessThrusters(ModuleParams *parent, AxisConfiguration axis_config,
		bool tilt_support)
	: ModuleParams(parent), _axis_config(axis_config), _tilt_support(tilt_support)
{

	for (int i = 0; i < NUM_THRUSTERS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_THRUSTER%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		if (_tilt_support) {
			PX4_ERR("Tilt support not implemented");
		}
	}

	_count_handle = param_find("CA_THRUSTER_CNT");

	updateParams();
}

void ActuatorEffectivenessThrusters::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_thrusters = math::min(NUM_THRUSTERS_MAX, (int)count);

	for (int i = 0; i < _geometry.num_thrusters; ++i) {
		Vector3f &position = _geometry.thrusters[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.thrusters[i].axis;

		switch (_axis_config) {
		case AxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.thrusters[i].thrust_coef);
	}
}

bool
ActuatorEffectivenessThrusters::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::THRUSTERS, num_actuators);
	return true;
}

int
ActuatorEffectivenessThrusters::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_thrusters; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.thrusters[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.thrusters[i].position;

		// Get coefficients
		float ct = geometry.thrusters[i].thrust_coef;

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis);

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
	}

	return num_actuators;
}


uint32_t ActuatorEffectivenessThrusters::getThrusters() const
{
	uint32_t thrusters = 0;

	for (int i = 0; i < _geometry.num_thrusters; ++i) {
		thrusters |= 1u << i;
	}

	return thrusters;
}

bool
ActuatorEffectivenessThrusters::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}
