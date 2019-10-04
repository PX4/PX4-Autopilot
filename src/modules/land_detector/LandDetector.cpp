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

#include "LandDetector.h"

#include <float.h>
#include <math.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

namespace land_detector
{

LandDetector::LandDetector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
	_land_detected.timestamp = hrt_absolute_time();
	_land_detected.freefall = false;
	_land_detected.landed = true;
	_land_detected.ground_contact = false;
	_land_detected.maybe_landed = false;
}

LandDetector::~LandDetector()
{
	perf_free(_cycle_perf);
}

void LandDetector::start()
{
	_check_params(true);
	ScheduleOnInterval(LAND_DETECTOR_UPDATE_INTERVAL);
}

void LandDetector::Run()
{
	perf_begin(_cycle_perf);

	_check_params(false);
	_actuator_armed_sub.update(&_actuator_armed);
	_update_topics();
	_update_state();

	const bool landDetected = (_state == LandDetectionState::LANDED);
	const bool freefallDetected = (_state == LandDetectionState::FREEFALL);
	const bool maybe_landedDetected = (_state == LandDetectionState::MAYBE_LANDED);
	const bool ground_contactDetected = (_state == LandDetectionState::GROUND_CONTACT);
	const float alt_max = _get_max_altitude() > 0.0f ? _get_max_altitude() : INFINITY;

	const bool in_ground_effect = _ground_effect_hysteresis.get_state();

	const hrt_abstime now = hrt_absolute_time();

	// publish at 1 Hz, very first time, or when the result has changed
	if ((hrt_elapsed_time(&_land_detected.timestamp) >= 1_s) ||
	    (_land_detected_pub == nullptr) ||
	    (_land_detected.landed != landDetected) ||
	    (_land_detected.freefall != freefallDetected) ||
	    (_land_detected.maybe_landed != maybe_landedDetected) ||
	    (_land_detected.ground_contact != ground_contactDetected) ||
	    (_land_detected.in_ground_effect != in_ground_effect) ||
	    (fabsf(_land_detected.alt_max - alt_max) > FLT_EPSILON)) {

		if (!landDetected && _land_detected.landed && _takeoff_time == 0) { /* only set take off time once, until disarming */
			// We did take off
			_takeoff_time = now;
		}

		_land_detected.timestamp = hrt_absolute_time();
		_land_detected.landed = landDetected;
		_land_detected.freefall = freefallDetected;
		_land_detected.maybe_landed = maybe_landedDetected;
		_land_detected.ground_contact = ground_contactDetected;
		_land_detected.alt_max = alt_max;
		_land_detected.in_ground_effect = in_ground_effect;

		int instance;
		orb_publish_auto(ORB_ID(vehicle_land_detected), &_land_detected_pub, &_land_detected,
				 &instance, ORB_PRIO_DEFAULT);
	}

	// set the flight time when disarming (not necessarily when landed, because all param changes should
	// happen on the same event and it's better to set/save params while not in armed state)
	if (_takeoff_time != 0 && !_actuator_armed.armed && _previous_armed_state) {
		_total_flight_time += now - _takeoff_time;
		_takeoff_time = 0;

		uint32_t flight_time = (_total_flight_time >> 32) & 0xffffffff;

		_param_total_flight_time_high.set(flight_time);
		_param_total_flight_time_high.commit_no_notification();

		flight_time = _total_flight_time & 0xffffffff;

		_param_total_flight_time_low.set(flight_time);
		_param_total_flight_time_low.commit_no_notification();
	}

	_previous_armed_state = _actuator_armed.armed;

	perf_end(_cycle_perf);

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}
}

void LandDetector::_check_params(const bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		_update_params();

		_update_total_flight_time();
	}
}

void LandDetector::_update_state()
{
	/* when we are landed we also have ground contact for sure but only one output state can be true at a particular time
	 * with higher priority for landed */
	const hrt_abstime now_us = hrt_absolute_time();
	_freefall_hysteresis.set_state_and_update(_get_freefall_state(), now_us);
	_landed_hysteresis.set_state_and_update(_get_landed_state(), now_us);
	_maybe_landed_hysteresis.set_state_and_update(_get_maybe_landed_state(), now_us);
	_ground_contact_hysteresis.set_state_and_update(_get_ground_contact_state(), now_us);
	_ground_effect_hysteresis.set_state_and_update(_get_ground_effect_state(), now_us);

	if (_freefall_hysteresis.get_state()) {
		_state = LandDetectionState::FREEFALL;

	} else if (_landed_hysteresis.get_state()) {
		_state = LandDetectionState::LANDED;

	} else if (_maybe_landed_hysteresis.get_state()) {
		_state = LandDetectionState::MAYBE_LANDED;

	} else if (_ground_contact_hysteresis.get_state()) {
		_state = LandDetectionState::GROUND_CONTACT;

	} else {
		_state = LandDetectionState::FLYING;
	}
}

void LandDetector::_update_total_flight_time()
{
	_total_flight_time = static_cast<uint64_t>(_param_total_flight_time_high.get()) << 32;
	_total_flight_time |= static_cast<uint32_t>(_param_total_flight_time_low.get());
}

} // namespace land_detector
