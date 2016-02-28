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
 * @file FixedwingLandDetector.cpp
 * Land detection algorithm for fixedwings
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "FixedwingLandDetector.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

FixedwingLandDetector::FixedwingLandDetector() : LandDetector(),
	_paramHandle(),
	_params(),
	_controlStateSub(-1),
	_armingSub(-1),
	_airspeedSub(-1),
	_controlState{},
	_arming{},
	_airspeed{},
	_parameterSub(-1),
	_velocity_xy_filtered(0.0f),
	_velocity_z_filtered(0.0f),
	_airspeed_filtered(0.0f),
	_accel_horz_lp(0.0f),
	_landDetectTrigger(0)
{
	_paramHandle.maxVelocity = param_find("LNDFW_VEL_XY_MAX");
	_paramHandle.maxClimbRate = param_find("LNDFW_VEL_Z_MAX");
	_paramHandle.maxAirSpeed = param_find("LNDFW_AIRSPD_MAX");
	_paramHandle.maxIntVelocity = param_find("LNDFW_VELI_MAX");
}

void FixedwingLandDetector::initialize()
{
	_controlStateSub = orb_subscribe(ORB_ID(control_state));
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
	_airspeedSub = orb_subscribe(ORB_ID(airspeed));

	updateParameterCache(true);
}

void FixedwingLandDetector::updateSubscriptions()
{
	orb_update(ORB_ID(control_state), _controlStateSub, &_controlState);
	orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
	orb_update(ORB_ID(airspeed), _airspeedSub, &_airspeed);
}

bool FixedwingLandDetector::update()
{
	// First poll for new data from our subscriptions
	updateSubscriptions();

	// only trigger flight conditions if we are armed
	if (!_arming.armed) {
		return true;
	}

	const uint64_t now = hrt_absolute_time();
	bool landDetected = false;

	if (hrt_elapsed_time(&_controlState.timestamp) < 500 * 1000) {
		float val = 0.97f * _velocity_xy_filtered + 0.03f * sqrtf(_controlState.x_vel *
				_controlState.x_vel + _controlState.y_vel * _controlState.y_vel);

		if (PX4_ISFINITE(val)) {
			_velocity_xy_filtered = val;
		}

		val = 0.99f * _velocity_z_filtered + 0.01f * fabsf(_controlState.z_vel);

		if (PX4_ISFINITE(val)) {
			_velocity_z_filtered = val;
		}

		_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed.true_airspeed_m_s;

		// a leaking lowpass prevents biases from building up, but
		// gives a mostly correct response for short impulses
		_accel_horz_lp = _accel_horz_lp * 0.8f + _controlState.horz_acc_mag * 0.18f;

		// crude land detector for fixedwing
		if (_velocity_xy_filtered < _params.maxVelocity
		    && _velocity_z_filtered < _params.maxClimbRate
		    && _airspeed_filtered < _params.maxAirSpeed
		    && _accel_horz_lp < _params.maxIntVelocity) {

			// these conditions need to be stable for a period of time before we trust them
			if (now > _landDetectTrigger) {
				landDetected = true;
			}

		} else {
			// reset land detect trigger
			_landDetectTrigger = now + LAND_DETECTOR_TRIGGER_TIME;
		}

	} else {
		// Control state topic has timed out and we need to assume we're landed.
		landDetected = true;
	}

	return landDetected;
}

void FixedwingLandDetector::updateParameterCache(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
		param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
		param_get(_paramHandle.maxAirSpeed, &_params.maxAirSpeed);
		param_get(_paramHandle.maxIntVelocity, &_params.maxIntVelocity);
	}
}
