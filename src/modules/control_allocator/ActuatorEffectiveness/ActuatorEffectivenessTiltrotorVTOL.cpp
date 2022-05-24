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
 * @file ActuatorEffectivenessTiltrotorVTOL.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltrotorVTOL.hpp"

using namespace matrix;

ActuatorEffectivenessTiltrotorVTOL::ActuatorEffectivenessTiltrotorVTOL(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _control_surfaces(this), _tilts(this)
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}
bool
ActuatorEffectivenessTiltrotorVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (!_combined_tilt_updated && external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors
	configuration.selected_matrix = 0;
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	_mc_rotors.enableThreeDimensionalThrust(false);

	// Update matrix with tilts in vertical position when update is triggered by a manual
	// configuration (parameter) change. This is to make sure the normalization
	// scales are tilt-invariant. Note: configuration updates are only possible when disarmed.
	const float tilt_control_applied = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE) ? -1.f :
					   _last_tilt_control;
	_nontilted_motors = _mc_rotors.updateAxisFromTilts(_tilts, tilt_control_applied)
			    << configuration.num_actuators[(int)ActuatorType::MOTORS];

	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// Control Surfaces
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	// Tilts
	configuration.selected_matrix = 0;
	_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	_tilts.updateTorqueSign(_mc_rotors.geometry(), true /* disable pitch to avoid configuration errors */);
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// If it was an update coming from a config change, then make sure to update matrix in
	// the next iteration again with the correct tilt (but without updating the normalization scale).
	_combined_tilt_updated = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	return (mc_rotors_added_successfully && surfaces_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessTiltrotorVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp)
{
	// apply flaps
	if (matrix_index == 1) {
		actuator_controls_s actuator_controls_1;

		if (_actuator_controls_1_sub.copy(&actuator_controls_1)) {
			const float flaps_control = actuator_controls_1.control[actuator_controls_s::INDEX_FLAPS];
			const float airbrakes_control = actuator_controls_1.control[actuator_controls_s::INDEX_AIRBRAKES];
			_control_surfaces.applyFlapsAndAirbrakes(flaps_control, airbrakes_control, _first_control_surface_idx, actuator_sp);
		}
	}

	// apply tilt
	if (matrix_index == 0) {
		actuator_controls_s actuator_controls_1;

		if (_actuator_controls_1_sub.copy(&actuator_controls_1)) {
			float control_tilt = actuator_controls_1.control[actuator_controls_s::INDEX_COLLECTIVE_TILT] * 2.f - 1.f;

			// set control_tilt to exactly -1 or 1 if close to these end points
			control_tilt = control_tilt < -0.99f ? -1.f : control_tilt;
			control_tilt = control_tilt > 0.99f ? 1.f : control_tilt;

			// initialize _last_tilt_control
			if (!PX4_ISFINITE(_last_tilt_control)) {
				_last_tilt_control = control_tilt;

			} else if (fabsf(control_tilt - _last_tilt_control) > 0.01f) {
				_combined_tilt_updated = true;
				_last_tilt_control = control_tilt;
			}

			for (int i = 0; i < _tilts.count(); ++i) {
				if (_tilts.config(i).tilt_direction == ActuatorEffectivenessTilts::TiltDirection::TowardsFront) {
					actuator_sp(i + _first_tilt_idx) += control_tilt;
				}
			}
		}

		// in FW directly use throttle sp
		if (_flight_phase == FlightPhase::FORWARD_FLIGHT) {

			actuator_controls_s actuator_controls_0;

			if (_actuator_controls_0_sub.copy(&actuator_controls_0)) {
				for (int i = 0; i < _first_tilt_idx; ++i) {
					actuator_sp(i) = actuator_controls_0.control[actuator_controls_s::INDEX_THROTTLE];
				}
			}
		}
	}
}

void ActuatorEffectivenessTiltrotorVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_stopped_motors = _nontilted_motors;
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_stopped_motors = 0;
		break;
	}
}
