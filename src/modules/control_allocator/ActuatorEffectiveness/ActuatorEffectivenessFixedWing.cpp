/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessFixedWing.hpp"
#include <ControlAllocation/ControlAllocation.hpp>

using namespace matrix;

ActuatorEffectivenessFixedWing::ActuatorEffectivenessFixedWing(ModuleParams *parent)
	: ModuleParams(parent), _rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedForward),
	  _control_surfaces(this)
{
}

bool
ActuatorEffectivenessFixedWing::getEffectivenessMatrix(Configuration &configuration,
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

void ActuatorEffectivenessFixedWing::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp)
{
	// apply flaps
	actuator_controls_s actuator_controls_0;

	if (_actuator_controls_0_sub.copy(&actuator_controls_0)) {
		float control_flaps = actuator_controls_0.control[actuator_controls_s::INDEX_FLAPS];
		float airbrakes_control = actuator_controls_0.control[actuator_controls_s::INDEX_AIRBRAKES];
		_control_surfaces.applyFlapsAndAirbrakes(control_flaps, airbrakes_control, _first_control_surface_idx, actuator_sp);
	}
}
