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

/*
 * @file LandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "LandDetector.h"


namespace land_detector
{


LandDetector::LandDetector() :
	_landDetectedPub(nullptr),
	_landDetected{0, false, false},
	_freefall_hysteresis(false),
	_landed_hysteresis(true),
	_taskShouldExit(false),
	_taskIsRunning(false),
	_work{}
{
	// Use Trigger time when transitioning from in-air (false) to landed (true).
	_landed_hysteresis.set_hysteresis_time_from(false, LAND_DETECTOR_TRIGGER_TIME_US);
}

LandDetector::~LandDetector()
{
	work_cancel(HPWORK, &_work);
	_taskShouldExit = true;
}

int LandDetector::start()
{
	_taskShouldExit = false;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LandDetector::_cycle_trampoline, this, 0);

	return 0;
}

void LandDetector::stop()
{
	_taskShouldExit = true;
}

void
LandDetector::_cycle_trampoline(void *arg)
{
	LandDetector *dev = reinterpret_cast<LandDetector *>(arg);

	dev->_cycle();
}

void LandDetector::_cycle()
{
	if (!_taskIsRunning) {
		// Advertise the first land detected uORB.
		_landDetected.timestamp = hrt_absolute_time();
		_landDetected.landed = false;
		_landDetected.freefall = false;

		// Initialize uORB topics.
		_initialize_topics();

		_check_params(true);

		// Task is now running, keep doing so until we need to stop.
		_taskIsRunning = true;
	}

	_check_params(false);

	_update_topics();

	_update_state();

	bool landDetected = (_state == LandDetectionState::LANDED);
	bool freefallDetected = (_state == LandDetectionState::FREEFALL);

	// Only publish very first time or when the result has changed.
	if ((_landDetectedPub == nullptr) ||
	    (_landDetected.landed != landDetected) ||
	    (_landDetected.freefall != freefallDetected)) {

		_landDetected.timestamp = hrt_absolute_time();
		_landDetected.landed = (_state == LandDetectionState::LANDED);
		_landDetected.freefall = (_state == LandDetectionState::FREEFALL);

		int instance;
		orb_publish_auto(ORB_ID(vehicle_land_detected), &_landDetectedPub, &_landDetected,
				 &instance, ORB_PRIO_DEFAULT);
	}

	if (!_taskShouldExit) {

		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&LandDetector::_cycle_trampoline, this,
			   USEC2TICK(1000000 / LAND_DETECTOR_UPDATE_RATE_HZ));

	} else {
		_taskIsRunning = false;
	}
}

void LandDetector::_check_params(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
	}
}

void LandDetector::_update_state()
{
	bool landed = _get_landed_state();
	_landed_hysteresis.set_state_and_update(landed);
	_freefall_hysteresis.set_state_and_update(_get_freefall_state());

	if (_freefall_hysteresis.get_state()) {
		_state = LandDetectionState::FREEFALL;

	} else if (_landed_hysteresis.get_state()) {
		_state = LandDetectionState::LANDED;

	} else {
		_state = LandDetectionState::FLYING;
	}

	return;
}

bool LandDetector::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
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


} // namespace land_detector
