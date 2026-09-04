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
	_rotors.setMotorDirectionBitmasks(_motor_direction_bitmasks);

	// Control Surfaces
	_first_control_surface_idx = configuration.num_actuators_matrix[0];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	return (rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessFixedWing::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	const float surface_lock_delay = _param_ca_cs_lk_delay.get();
	const float motor_lock_delay = _param_ca_r_lk_delay.get();

	if (surface_lock_delay < FLT_EPSILON && motor_lock_delay < FLT_EPSILON) {
		return;
	}

	vehicle_status_s vehicle_status;

	if (!_vehicle_status_sub.copy(&vehicle_status)) {
		return;
	}

	// takeoff_time is 0 while disarmed and until a takeoff is detected
	const bool before_takeoff = vehicle_status.takeoff_time == 0;
	const hrt_abstime time_since_takeoff = hrt_elapsed_time(&vehicle_status.takeoff_time);

	// keep selected control surfaces at the disarmed value until the configured time after takeoff has passed
	if (surface_lock_delay > FLT_EPSILON
	    && (before_takeoff || time_since_takeoff < (hrt_abstime)(surface_lock_delay * 1_s))) {
		_control_surfaces.applyLaunchLock(_first_control_surface_idx, actuator_sp);
	}

	// keep the motors at the disarmed value until the configured time after takeoff has passed
	if (motor_lock_delay > FLT_EPSILON
	    && (before_takeoff || time_since_takeoff < (hrt_abstime)(motor_lock_delay * 1_s))) {
		// the motors take up the actuators of the first matrix ahead of the control surfaces
		for (int i = 0; i < _first_control_surface_idx; ++i) {
			actuator_sp(i) = NAN;
		}
	}
}

void ActuatorEffectivenessFixedWing::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
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
