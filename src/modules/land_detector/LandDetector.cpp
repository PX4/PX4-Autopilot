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
 * @file LandDetector.cpp
 * Land detection algorithm
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 */

#include "LandDetector.h"
#include <unistd.h>                 //usleep
#include <drivers/drv_hrt.h>

LandDetector::LandDetector() :
	_landDetectedPub(-1),
	_landDetected({0, false}),
	_taskShouldExit(false),
	_taskIsRunning(false)
{
	// ctor
}

LandDetector::~LandDetector()
{
	_taskShouldExit = true;
	close(_landDetectedPub);
}

void LandDetector::shutdown()
{
	_taskShouldExit = true;
}

void LandDetector::start()
{
	// make sure this method has not already been called by another thread
	if (isRunning()) {
		return;
	}

	// advertise the first land detected uORB
	_landDetected.timestamp = hrt_absolute_time();
	_landDetected.landed = false;
	_landDetectedPub = orb_advertise(ORB_ID(vehicle_land_detected), &_landDetected);

	// initialize land detection algorithm
	initialize();

	// task is now running, keep doing so until shutdown() has been called
	_taskIsRunning = true;
	_taskShouldExit = false;

	while (isRunning()) {

		bool landDetected = update();

		// publish if land detection state has changed
		if (_landDetected.landed != landDetected) {
			_landDetected.timestamp = hrt_absolute_time();
			_landDetected.landed = landDetected;

			// publish the land detected broadcast
			orb_publish(ORB_ID(vehicle_land_detected), _landDetectedPub, &_landDetected);
		}

		// limit loop rate
		usleep(1000000 / LAND_DETECTOR_UPDATE_RATE);
	}

	_taskIsRunning = false;
	_exit(0);
}

bool LandDetector::orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
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
