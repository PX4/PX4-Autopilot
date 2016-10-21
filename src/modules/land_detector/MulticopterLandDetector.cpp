/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#include <cmath>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include "MulticopterLandDetector.h"


namespace land_detector
{

MulticopterLandDetector::MulticopterLandDetector() : LandDetector(),
	_paramHandle(),
	_params(),
	_vehicleLocalPositionSub(-1),
	_actuatorsSub(-1),
	_armingSub(-1),
	_attitudeSub(-1),
	_manualSub(-1),
	_ctrl_state_sub(-1),
	_vehicle_control_mode_sub(-1),
	_vehicleLocalPosition{},
	_actuators{},
	_arming{},
	_vehicleAttitude{},
	_manual{},
	_ctrl_state{},
	_ctrl_mode{},
	_min_trust_start(0),
	_arming_time(0)
{
	_paramHandle.maxRotation = param_find("LNDMC_ROT_MAX");
	_paramHandle.maxVelocity = param_find("LNDMC_XY_VEL_MAX");
	_paramHandle.maxClimbRate = param_find("LNDMC_Z_VEL_MAX");
	_paramHandle.maxThrottle = param_find("MPC_THR_MIN");
	_paramHandle.minManThrottle = param_find("MPC_MANTHR_MIN");
	_paramHandle.acc_threshold_m_s2 = param_find("LNDMC_FFALL_THR");
	_paramHandle.ff_trigger_time = param_find("LNDMC_FFALL_TTRI");
}

void MulticopterLandDetector::_initialize_topics()
{
	// subscribe to position, attitude, arming and velocity changes
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_actuatorsSub = orb_subscribe(ORB_ID(actuator_controls_0));
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
	_manualSub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
}

void MulticopterLandDetector::_update_topics()
{
	_orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	_orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_orb_update(ORB_ID(actuator_controls_0), _actuatorsSub, &_actuators);
	_orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
	_orb_update(ORB_ID(manual_control_setpoint), _manualSub, &_manual);
	_orb_update(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
	_orb_update(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &_ctrl_mode);
}

void MulticopterLandDetector::_update_params()
{
	param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
	param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
	param_get(_paramHandle.maxRotation, &_params.maxRotation_rad_s);
	_params.maxRotation_rad_s = math::radians(_params.maxRotation_rad_s);
	param_get(_paramHandle.maxThrottle, &_params.maxThrottle);
	param_get(_paramHandle.minManThrottle, &_params.minManThrottle);
	param_get(_paramHandle.acc_threshold_m_s2, &_params.acc_threshold_m_s2);
	param_get(_paramHandle.ff_trigger_time, &_params.ff_trigger_time);
	_freefall_hysteresis.set_hysteresis_time_from(false, 1e6f * _params.ff_trigger_time);
}


bool MulticopterLandDetector::_get_freefall_state()
{
	if (_params.acc_threshold_m_s2 < 0.1f
	    || _params.acc_threshold_m_s2 > 10.0f) {	//if parameter is set to zero or invalid, disable free-fall detection.
		return false;
	}

	if (_ctrl_state.timestamp == 0) {
		// _ctrl_state is not valid yet, we have to assume we're not falling.
		return false;
	}

	float acc_norm = _ctrl_state.x_acc * _ctrl_state.x_acc
			 + _ctrl_state.y_acc * _ctrl_state.y_acc
			 + _ctrl_state.z_acc * _ctrl_state.z_acc;
	acc_norm = sqrtf(acc_norm);	//norm of specific force. Should be close to 9.8 m/s^2 when landed.

	return (acc_norm < _params.acc_threshold_m_s2);	//true if we are currently falling
}

bool MulticopterLandDetector::_get_landed_state()
{
	// Time base for this function
	const uint64_t now = hrt_absolute_time();

	float sys_min_throttle = (_params.maxThrottle + 0.01f);

	// Determine the system min throttle based on flight mode
	if (!_ctrl_mode.flag_control_altitude_enabled) {
		sys_min_throttle = (_params.minManThrottle + 0.01f);
	}

	// Check if thrust output is less than the minimum auto throttle param.
	bool minimalThrust = (_actuators.control[3] <= sys_min_throttle);

	if (minimalThrust && _min_trust_start == 0) {
		_min_trust_start = now;

	} else if (!minimalThrust) {
		_min_trust_start = 0;
	}

	// only trigger flight conditions if we are armed
	if (!_arming.armed) {
		_arming_time = 0;

		return true;

	} else if (_arming_time == 0) {
		_arming_time = now;
	}

	// If in manual flight mode never report landed if the user has more than idle throttle
	// Check if user commands throttle and if so, report not landed based on
	// the user intent to take off (even if the system might physically still have
	// ground contact at this point).
	if (_manual.timestamp > 0 && _manual.z > 0.15f && _ctrl_mode.flag_control_manual_enabled) {
		return false;
	}

	// Return status based on armed state and throttle if no position lock is available.
	if (_vehicleLocalPosition.timestamp == 0 ||
	    hrt_elapsed_time(&_vehicleLocalPosition.timestamp) > 500000 ||
	    !_vehicleLocalPosition.xy_valid ||
	    !_vehicleLocalPosition.z_valid) {

		// The system has minimum trust set (manual or in failsafe)
		// if this persists for 8 seconds AND the drone is not
		// falling consider it to be landed. This should even sustain
		// quite acrobatic flight.
		if ((_min_trust_start > 0) &&
		    (hrt_elapsed_time(&_min_trust_start) > 8 * 1000 * 1000)) {

			return true;

		} else {
			return false;
		}
	}

	float armThresholdFactor = 1.0f;

	// Widen acceptance thresholds for landed state right after arming
	// so that motor spool-up and other effects do not trigger false negatives.
	if (hrt_elapsed_time(&_arming_time) < LAND_DETECTOR_ARM_PHASE_TIME_US) {
		armThresholdFactor = 2.5f;
	}

	// Check if we are moving vertically - this might see a spike after arming due to
	// throttle-up vibration. If accelerating fast the throttle thresholds will still give
	// an accurate in-air indication.
	bool verticalMovement = fabsf(_vehicleLocalPosition.vz) > _params.maxClimbRate * armThresholdFactor;

	// Check if we are moving horizontally.
	bool horizontalMovement = sqrtf(_vehicleLocalPosition.vx * _vehicleLocalPosition.vx
					+ _vehicleLocalPosition.vy * _vehicleLocalPosition.vy) > _params.maxVelocity;

	// Next look if all rotation angles are not moving.
	float maxRotationScaled = _params.maxRotation_rad_s * armThresholdFactor;

	bool rotating = (fabsf(_vehicleAttitude.rollspeed)  > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.pitchspeed) > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.yawspeed) > maxRotationScaled);


	if (verticalMovement || rotating || !minimalThrust || horizontalMovement) {
		// Sensed movement or thottle high, so reset the land detector.
		return false;
	}

	return true;
}


}
