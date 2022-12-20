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

#include "ActuatorEffectivenessStandardVTOL.hpp"
#include <ControlAllocation/ControlAllocation.hpp>

using namespace matrix;

ActuatorEffectivenessStandardVTOL::ActuatorEffectivenessStandardVTOL(ModuleParams *parent)
	: ModuleParams(parent), _rotors(this), _control_surfaces(this)
{
}

bool
ActuatorEffectivenessStandardVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	configuration.selected_matrix = 0;
	_rotors.enablePropellerTorqueNonUpwards(false);
	const bool mc_rotors_added_successfully = _rotors.addActuators(configuration);
	_mc_motors_mask = _rotors.getUpwardsMotors();

	// Control Surfaces
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	return (mc_rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessStandardVTOL::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	if (matrix_index == 1) {
		// apply flaps
		normalized_unsigned_setpoint_s flaps_setpoint;

		if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
			_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}

		// apply spoilers
		normalized_unsigned_setpoint_s spoilers_setpoint;

		if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
			_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}
	}
}

void ActuatorEffectivenessStandardVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_stopped_motors = _mc_motors_mask;
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_stopped_motors = 0;
		break;
	}
}
