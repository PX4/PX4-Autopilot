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
 * Land detection algorithm for multicopters
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 */

#include "MulticopterLandDetector.h"

#include <cmath>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

MulticopterLandDetector::MulticopterLandDetector() : LandDetector(),
	_paramHandle(),
	_params(),
	_vehicleLocalPositionSub(-1),
	_actuatorsSub(-1),
	_armingSub(-1),
	_parameterSub(-1),
	_attitudeSub(-1),
	_manualSub(-1),
	_vehicleLocalPosition{},
	_actuators{},
	_arming{},
	_vehicleAttitude{},
	_landTimer(0)
{
	_paramHandle.maxRotation = param_find("LNDMC_ROT_MAX");
	_paramHandle.maxVelocity = param_find("LNDMC_XY_VEL_MAX");
	_paramHandle.maxClimbRate = param_find("LNDMC_Z_VEL_MAX");
	_paramHandle.maxThrottle = param_find("LNDMC_THR_MAX");
}

void MulticopterLandDetector::initialize()
{
	// subscribe to position, attitude, arming and velocity changes
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_actuatorsSub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
	_manualSub = orb_subscribe(ORB_ID(manual_control_setpoint));

	// download parameters
	updateParameterCache(true);
}

void MulticopterLandDetector::updateSubscriptions()
{
	orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	orb_update(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuatorsSub, &_actuators);
	orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
	orb_update(ORB_ID(manual_control_setpoint), _manualSub, &_manual);
}

bool MulticopterLandDetector::update()
{
	// first poll for new data from our subscriptions
	updateSubscriptions();

	updateParameterCache(false);

	return get_landed_state();
}

bool MulticopterLandDetector::get_landed_state()
{
	// only trigger flight conditions if we are armed
	if (!_arming.armed) {
		_arming_time = 0;
		return true;

	} else if (_arming_time == 0) {
		_arming_time = hrt_absolute_time();
	}

	// Check if user commands throttle and if so, report not landed
	if (_manual.z > 0.3f) {
		return false;
	}

	// Check if thrust output is less than max throttle param.
	bool minimalThrust = _actuators.control[3] <= _params.maxThrottle;

	// Return status based on armed state and throttle if no position lock is available.
	if (_vehicleLocalPosition.timestamp == 0 ||
	    hrt_elapsed_time(&_vehicleLocalPosition.timestamp) > 500000 ||
	    !_vehicleLocalPosition.xy_valid ||
	    !_vehicleLocalPosition.z_valid) {

		// Minimal thrust means landed.
		return minimalThrust;
	}

	const uint64_t now = hrt_absolute_time();

	float armThresholdFactor = 1.0f;

	// Widen acceptance thresholds for landed state right after arming
	// so that motor spool-up and other effects do not trigger false negatives.
	if (hrt_elapsed_time(&_arming_time) < LAND_DETECTOR_ARM_PHASE_TIME) {
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
		_landTimer = now;
		return false;
	}

	return (now - _landTimer > LAND_DETECTOR_TRIGGER_TIME);
}

void MulticopterLandDetector::updateParameterCache(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
		param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
		param_get(_paramHandle.maxRotation, &_params.maxRotation_rad_s);
		_params.maxRotation_rad_s = math::radians(_params.maxRotation_rad_s);
		param_get(_paramHandle.maxThrottle, &_params.maxThrottle);
	}
}
