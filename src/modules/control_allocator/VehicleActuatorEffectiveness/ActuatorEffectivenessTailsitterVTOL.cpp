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

/**
 * @file ActuatorEffectivenessTailsitterVTOL.cpp
 *
 * Actuator effectiveness for tailsitter VTOL
 */

#include "ActuatorEffectivenessTailsitterVTOL.hpp"

using namespace matrix;

ActuatorEffectivenessTailsitterVTOL::ActuatorEffectivenessTailsitterVTOL(ModuleParams *parent)
	: ModuleParams(parent), _mc_rotors(this), _control_surfaces(this)
{
	_param_handles.vt_elev_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_param_handles.vt_ts_cs_hvr_dis = param_find("VT_TS_CS_HVR_DIS");
	updateParams();
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

void ActuatorEffectivenessTailsitterVTOL::updateParams()
{
	ModuleParams::updateParams();
	param_get(_param_handles.vt_elev_mc_lock, &_param_vt_elev_mc_lock);
	param_get(_param_handles.vt_ts_cs_hvr_dis, &_param_vt_ts_cs_hvr_dis);
}

bool
ActuatorEffectivenessTailsitterVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (!_control_surfaces_updated && external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors
	configuration.selected_matrix = 0;
	// enable MC yaw control if more than 3 rotors
	_mc_rotors.enableYawByDifferentialThrust(_mc_rotors.geometry().num_rotors > 3);
	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// Control Surfaces
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	// In HOVER_FLIGHT, selectively disable control surfaces based on VT_TS_CS_HVR_DIS bitmask.
	// This only takes effect when VT_ELEV_MC_LOCK=0 (control surfaces unlocked in hover).
	if (surfaces_added_successfully && _flight_phase == FlightPhase::HOVER_FLIGHT && _param_vt_elev_mc_lock == 0) {
		// VT_TS_CS_HVR_DIS: bit=1 means disabled in hover, bit=0 means enabled (default)
		for (int i = 0; i < _control_surfaces.count(); i++) {
			if ((_param_vt_ts_cs_hvr_dis & (1 << i)) != 0) {
				// Set the corresponding column in effectiveness matrix to zero
				for (int row = 0; row < NUM_AXES; row++) {
					configuration.effectiveness_matrices[1](row, _first_control_surface_idx + i) = 0.f;
				}
			}
		}
	}

	// Reset flag after update
	_control_surfaces_updated = false;

	return (mc_rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessTailsitterVTOL::allocateAuxilaryControls(const float dt, int matrix_index,
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

void ActuatorEffectivenessTailsitterVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index == 0) {
		stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);
	}
}

void ActuatorEffectivenessTailsitterVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// Trigger control surfaces matrix update when transitioning between HOVER_FLIGHT and other flight phases.
	_control_surfaces_updated = true;

	// update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_forwards_motors_mask = _mc_rotors.getUpwardsMotors(); // allocation frame they stay upwards
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_forwards_motors_mask = 0;
		_stopped_motors_mask = 0;
		break;
	}
}
