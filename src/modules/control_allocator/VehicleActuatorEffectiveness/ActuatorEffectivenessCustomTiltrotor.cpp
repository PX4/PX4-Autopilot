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

// ActuatorEffectivenessCustomTiltrotor.cpp
#include "ActuatorEffectivenessCustomTiltrotor.hpp"

using namespace matrix;

ActuatorEffectivenessCustomTiltrotor::ActuatorEffectivenessCustomTiltrotor(ModuleParams *parent)
    : ModuleParams(parent),
      _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
      _control_surfaces(this),
      _tilts(this)
{
	_mc_rotors.enableYawByDifferentialThrust(false);
}

bool ActuatorEffectivenessCustomTiltrotor::getEffectivenessMatrix(
    Configuration &configuration, EffectivenessUpdateReason external_update)
{
    if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
        return false;
    }

    // Current tilt control needed here -- getEffectivenessMatrix() runs on a
    // slower/parameter-update cadence than updateSetpoint(), so this is the
    // best-known tilt value at matrix-rebuild time, not necessarily this
    // instant's setpoint. Track it as a member, updated each updateSetpoint() call.
    ActuatorBitmask non_tiltable = _mc_rotors.updateAxisFromTilts(_tilts, _current_tilt_control);
    (void)non_tiltable; // fine to ignore for now, but named/captured rather than silently dropped

    _mc_rotors.addActuators(configuration);

    _first_tilt_idx = configuration.totalNumActuators();
    _tilts.updateTorqueSign(_mc_rotors.geometry());
    _tilts.addActuators(configuration);

    _first_control_surface_idx = configuration.totalNumActuators();
    _control_surfaces.addActuators(configuration);

    return true;
}

float ActuatorEffectivenessCustomTiltrotor::propwashGain(float lambda) const
{
    // Below WASH_ONSET_LAMBDA: nacelle still near-vertical, wash exits downward,
    // tail sees ~nothing -> near-zero authority.
    // Between onset and full: wash sweeping aft, authority ramps up.
    // Above WASH_FULL_LAMBDA: tail fully in wash (hover-ish) or fully in
    // freestream (cruise) -> full authority either way.
    if (lambda <= WASH_ONSET_LAMBDA) {
        return 0.1f; // small floor, not exactly zero, for numerical safety
    }

    if (lambda >= WASH_FULL_LAMBDA) {
        return 1.0f;
    }

    return 0.1f + 0.9f * (lambda - WASH_ONSET_LAMBDA) / (WASH_FULL_LAMBDA - WASH_ONSET_LAMBDA);
}

void ActuatorEffectivenessCustomTiltrotor::updateSetpoint(
    const matrix::Vector<float, NUM_AXES> &control_sp,
    int matrix_index,
    ActuatorVector &actuator_sp,
    const ActuatorVector &actuator_min,
    const ActuatorVector &actuator_max)
{
    if (matrix_index != 0) {
        return;
    }

    // Derive lambda from the current tilt actuator setpoints themselves
    // rather than duplicating the collective-tilt subscription logic --
    // actuator_sp at the tilt indices already reflects commanded tilt
    // this cycle, before this function scales the surfaces below.
    float tilt_sum = 0.f;
    for (int i = 0; i < _tilts.count(); ++i) {
        tilt_sum += actuator_sp(i + _first_tilt_idx); // expect [-1 hover .. +1 forward] range, confirm against actual tilt sp convention
    }
    float avg_tilt = _tilts.count() > 0 ? tilt_sum / _tilts.count() : -1.f;
	_current_tilt_control = avg_tilt;
    _lambda = (avg_tilt + 1.f) * 0.5f; // map [-1,1] -> [0,1]


    float gain = propwashGain(_lambda);

    // Scale only the pitch/yaw control surfaces (elevator/rudder/ruddervators),
    // leaving motor and tilt setpoints untouched.
    for (int i = 0; i < _control_surfaces.count(); ++i) {
        actuator_sp(i + _first_control_surface_idx) *= gain;
    }
}