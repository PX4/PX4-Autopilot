/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 */

#include "FixedwingLandDetector.h"

#include <stdio.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>                 //usleep

FixedwingLandDetector::FixedwingLandDetector() :
	_landDetectedPub(-1),
	_landDetected({0, false}),

	      _vehicleLocalPositionSub(-1),
	      _vehicleLocalPosition({}),
	      _airspeedSub(-1),
	      _airspeed({}),

	      _velocity_xy_filtered(0.0f),
	      _velocity_z_filtered(0.0f),
	      _airspeed_filtered(0.0f),
	      _landDetectTrigger(0),

	      _taskShouldExit(false),
	      _taskIsRunning(false)
{
	//Advertise the first land detected uORB
	_landDetected.timestamp = hrt_absolute_time();
	_landDetected.landed = false;
	_landDetectedPub = orb_advertise(ORB_ID(vehicle_land_detected), &_landDetected);
}

FixedwingLandDetector::~FixedwingLandDetector()
{
	_taskShouldExit = true;
	close(_landDetectedPub);
}

void FixedwingLandDetector::shutdown()
{
	_taskShouldExit = true;
}

/**
* @brief Convinience function for polling uORB subscriptions
* @return true if there was new data and it was successfully copied
**/
static bool orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	//Check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

void FixedwingLandDetector::updateSubscriptions()
{
	orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	orb_update(ORB_ID(airspeed), _airspeedSub, &_airspeed);
}

void FixedwingLandDetector::landDetectorLoop()
{
	//This should never happen!
	if (_taskIsRunning) { return; }

	//Subscribe to local position and airspeed data
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_airspeedSub = orb_subscribe(ORB_ID(airspeed));

	_taskIsRunning = true;
	_taskShouldExit = false;

	while (!_taskShouldExit) {

		//First poll for new data from our subscriptions
		updateSubscriptions();

		const uint64_t now = hrt_absolute_time();
		bool landDetected = false;

		//TODO: reset filtered values on arming?
		_velocity_xy_filtered = 0.95f * _velocity_xy_filtered + 0.05f * sqrtf(_vehicleLocalPosition.vx *
					_vehicleLocalPosition.vx + _vehicleLocalPosition.vy * _vehicleLocalPosition.vy);
		_velocity_z_filtered = 0.95f * _velocity_z_filtered + 0.05f * fabsf(_vehicleLocalPosition.vz);
		_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed.true_airspeed_m_s;

		/* crude land detector for fixedwing */
		if (_velocity_xy_filtered < FW_LAND_DETECTOR_VELOCITY_MAX
		    && _velocity_z_filtered < FW_LAND_DETECTOR_CLIMBRATE_MAX
		    && _airspeed_filtered < FW_LAND_DETECTOR_AIRSPEED_MAX) {

			//These conditions need to be stable for a period of time before we trust them
			if (now > _landDetectTrigger) {
				landDetected = true;
			}

		} else {
			//reset land detect trigger
			_landDetectTrigger = now + FW_LAND_DETECTOR_TRIGGER_TIME;
		}

		//Publish if land detection state has changed
		if (_landDetected.landed != landDetected) {
			_landDetected.timestamp = now;
			_landDetected.landed = landDetected;

			/* publish the land detected broadcast */
			orb_publish(ORB_ID(vehicle_land_detected), _landDetectedPub, &_landDetected);
		}

		//Limit loop rate
		usleep(1000000 / FW_LAND_DETECTOR_UPDATE_RATE);
	}

	_taskIsRunning = false;
	_exit(0);
}

bool FixedwingLandDetector::isRunning() const
{
	return _taskIsRunning;
}
