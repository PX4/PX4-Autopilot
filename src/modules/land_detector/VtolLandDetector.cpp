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
 * @file VtolLandDetector.cpp
 * Land detection algorithm for VTOL
 *
 * @author Roman Bapst <bapstroma@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <drivers/drv_hrt.h>

#include "VtolLandDetector.h"

namespace land_detector
{

VtolLandDetector::VtolLandDetector() :
	_paramHandle(),
	_params(),
	_airspeedSub(-1),
	_airspeed{},
	_was_in_air(false),
	_airspeed_filtered(0)
{
	_paramHandle.maxAirSpeed = param_find("LNDFW_AIRSPD_MAX");
}

void VtolLandDetector::_initialize_topics()
{
	MulticopterLandDetector::_initialize_topics();

	_airspeedSub = orb_subscribe(ORB_ID(airspeed));
}

void VtolLandDetector::_update_topics()
{
	MulticopterLandDetector::_update_topics();

	_orb_update(ORB_ID(airspeed), _airspeedSub, &_airspeed);
}

bool VtolLandDetector::_get_ground_contact_state()
{
	return MulticopterLandDetector::_get_ground_contact_state();
}

bool VtolLandDetector::_get_maybe_landed_state()
{

	// TODO
	return false;
}

bool VtolLandDetector::_get_landed_state()
{
	// this is returned from the mutlicopter land detector
	bool landed = MulticopterLandDetector::_get_landed_state();

	// for vtol we additionally consider airspeed
	if (hrt_elapsed_time(&_airspeed.timestamp) < 500 * 1000) {
		_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed.true_airspeed_m_s;

	} else {
		// if airspeed does not update, set it to zero and rely on multicopter land detector
		_airspeed_filtered = 0.0f;
	}

	// only consider airspeed if we have been in air before to avoid false
	// detections in the case of wind on the ground
	if (_was_in_air && _airspeed_filtered > _params.maxAirSpeed) {
		landed = false;
	}

	_was_in_air = !landed;

	return landed;
}

bool VtolLandDetector::_get_freefall_state()
{
	return MulticopterLandDetector::_get_freefall_state();
}

void VtolLandDetector::_update_params()
{
	MulticopterLandDetector::_update_params();

	param_get(_paramHandle.maxAirSpeed, &_params.maxAirSpeed);
}

}
