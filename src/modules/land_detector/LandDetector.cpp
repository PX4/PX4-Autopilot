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
#include <float.h>

#include "LandDetector.h"


namespace land_detector
{


LandDetector::LandDetector() :
	_landDetectedPub(nullptr),
	_landDetected{0, false, false},
	_parameterSub(0),
	_state{},
	_freefall_hysteresis(false),
	_landed_hysteresis(true),
	_ground_contact_hysteresis(true),
	_taskShouldExit(false),
	_taskIsRunning(false),
	_total_flight_time{0},
	_takeoff_time{0},
	_work{}
{
	// Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
	_landed_hysteresis.set_hysteresis_time_from(false, LAND_DETECTOR_TRIGGER_TIME_US);
	_ground_contact_hysteresis.set_hysteresis_time_from(false, GROUND_CONTACT_TRIGGER_TIME_US);
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
		_landDetected.freefall = false;
		_landDetected.landed = false;
		_landDetected.ground_contact = false;
		_p_total_flight_time_high = param_find("LND_FLIGHT_T_HI");
		_p_total_flight_time_low = param_find("LND_FLIGHT_T_LO");

		// Initialize uORB topics.
		_initialize_topics();

		_check_params(true);

		// Task is now running, keep doing so until we need to stop.
		_taskIsRunning = true;
	}

	_check_params(false);

	_update_topics();

	hrt_abstime now = hrt_absolute_time();

	_update_state();

	float alt_max_prev = _altitude_max;
	_altitude_max = _get_max_altitude();

	bool freefallDetected = (_state == LandDetectionState::FREEFALL);
	bool landDetected = (_state == LandDetectionState::LANDED);
	bool ground_contactDetected = (_state == LandDetectionState::GROUND_CONTACT);

	// Only publish very first time or when the result has changed.
	if ((_landDetectedPub == nullptr) ||
	    (_landDetected.freefall != freefallDetected) ||
	    (_landDetected.landed != landDetected) ||
	    (_landDetected.ground_contact != ground_contactDetected) ||
	    (fabsf(_landDetected.alt_max - alt_max_prev) > FLT_EPSILON)) {

		if (!landDetected && _landDetected.landed) {
			// We did take off
			_takeoff_time = now;

		} else if (_takeoff_time != 0 && landDetected && !_landDetected.landed) {
			// We landed
			_total_flight_time += now - _takeoff_time;
			_takeoff_time = 0;
			int32_t flight_time = (_total_flight_time >> 32) & 0xffffffff;
			param_set_no_notification(_p_total_flight_time_high, &flight_time);
			flight_time = _total_flight_time & 0xffffffff;
			param_set_no_notification(_p_total_flight_time_low, &flight_time);
		}

		_landDetected.timestamp = hrt_absolute_time();
		_landDetected.freefall = (_state == LandDetectionState::FREEFALL);
		_landDetected.landed = (_state == LandDetectionState::LANDED);
		_landDetected.ground_contact = (_state == LandDetectionState::GROUND_CONTACT);
		_landDetected.alt_max = _altitude_max;

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
		int32_t flight_time;
		param_get(_p_total_flight_time_high, &flight_time);
		_total_flight_time = ((uint64_t)flight_time) << 32;
		param_get(_p_total_flight_time_low, &flight_time);
		_total_flight_time |= flight_time;
	}
}

void LandDetector::_update_state()
{
	/* ground contact and landed can be true simultaneously but only one state can be true at a particular time
	 * with higher priority for landed */
	bool freefall = _get_freefall_state();
	bool landed = _get_landed_state();
	bool groundContact = (landed || _get_ground_contact_state());

	_freefall_hysteresis.set_state_and_update(freefall);
	_landed_hysteresis.set_state_and_update(landed);
	_ground_contact_hysteresis.set_state_and_update(groundContact);

	if (_freefall_hysteresis.get_state()) {
		_state = LandDetectionState::FREEFALL;

	} else if (_landed_hysteresis.get_state()) {
		_state = LandDetectionState::LANDED;

	} else if (_ground_contact_hysteresis.get_state()) {
		_state = LandDetectionState::GROUND_CONTACT;

	} else {
		_state = LandDetectionState::FLYING;
	}
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
