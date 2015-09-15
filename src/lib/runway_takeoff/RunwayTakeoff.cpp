/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file RunwayTakeoff.cpp
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "RunwayTakeoff.h"
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <mavlink/mavlink_log.h>
#include <mathlib/mathlib.h>

namespace runwaytakeoff
{

RunwayTakeoff::RunwayTakeoff() :
	SuperBlock(NULL, "RWTO"),
	_state(),
	_initialized(false),
	_initialized_time(0),
	_init_yaw(0),
	_climbout(false),
	_min_airspeed_scaling(1.3f),
	_runway_takeoff_enabled(this, "TKOFF"),
	_runway_takeoff_heading(this, "HDG"),
	_runway_takeoff_nav_alt(this, "NAV_ALT"),
	_runway_takeoff_throttle(this, "MAX_THR"),
	_runway_takeoff_pitch_sp(this, "PSP"),
	_airspeed_min(this, "FW_AIRSPD_MIN", false),
	_climbout_diff(this, "FW_CLMBOUT_DIFF", false)
{

	updateParams();
}

RunwayTakeoff::~RunwayTakeoff()
{

}

void RunwayTakeoff::init(float yaw)
{
	_init_yaw = yaw;
	_initialized = true;
	_state = RunwayTakeoffState::THROTTLE_RAMP;
	_initialized_time = hrt_absolute_time();
}

void RunwayTakeoff::update(float airspeed, float alt_agl, int mavlink_fd)
{
	if (_climbout_diff.get() > 0.0001f && alt_agl < _climbout_diff.get()) {
		_climbout = true;
	}

	else {
		_climbout = false;
	}

	switch (_state) {
	case RunwayTakeoffState::THROTTLE_RAMP:
		if (hrt_elapsed_time(&_initialized_time) > 1e6) {
			_state = RunwayTakeoffState::CLAMPED_TO_RUNWAY;
		}

		break;

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		if (airspeed > _airspeed_min.get() * _min_airspeed_scaling) {
			_state = RunwayTakeoffState::TAKEOFF;
			mavlink_log_info(mavlink_fd, "#audio: Takeoff airspeed reached");
		}

		break;

	case RunwayTakeoffState::TAKEOFF:
		if (alt_agl > math::max(_runway_takeoff_nav_alt.get(), _climbout_diff.get())) {
			_state = RunwayTakeoffState::FLY;
			mavlink_log_info(mavlink_fd, "#audio: Navigating to waypoint");
		}

		break;

	default:
		return;
	}
}

bool RunwayTakeoff::controlYaw()
{
	// keep controlling yaw with rudder until we have enough ground clearance
	return _state < RunwayTakeoffState::FLY;
}

float RunwayTakeoff::getPitch(float tecsPitch)
{
	if (_state <= RunwayTakeoffState::CLAMPED_TO_RUNWAY) {
		return math::radians(_runway_takeoff_pitch_sp.get());
	}

	return tecsPitch;
}

float RunwayTakeoff::getRoll(float navigatorRoll)
{
	if (_state < RunwayTakeoffState::FLY) {
		return 0.0f;
	}

	return navigatorRoll;
}

float RunwayTakeoff::getYaw(float navigatorYaw)
{
	if (_state < RunwayTakeoffState::FLY) {
		if (_runway_takeoff_heading.get() == 0) {
			// fix heading in the direction the airframe points
			return _init_yaw;

		} else if (_runway_takeoff_heading.get() == 1) {
			// or head into the direction of the takeoff waypoint
			// XXX this needs a check if the deviation from actual heading is too big (else we do a full throttle wheel turn on the ground)
			return navigatorYaw;
		}
	}

	return navigatorYaw;
}

float RunwayTakeoff::getThrottle(float tecsThrottle)
{
	switch (_state) {
	case RunwayTakeoffState::THROTTLE_RAMP: {
			float throttle = hrt_elapsed_time(&_initialized_time) / (float)2000000 *
					 _runway_takeoff_throttle.get();
			return throttle < _runway_takeoff_throttle.get() ?
			       throttle :
			       _runway_takeoff_throttle.get();
		}

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		return _runway_takeoff_throttle.get();

	default:
		return tecsThrottle;
	}
}

bool RunwayTakeoff::resetIntegrators()
{
	return _state <= RunwayTakeoffState::TAKEOFF;
}

float RunwayTakeoff::getMinPitch(float sp_min, float climbout_min, float min)
{
	if (_climbout) {
		return math::max(sp_min, climbout_min);
	}

	else {
		return min;
	}
}

void RunwayTakeoff::reset()
{
	_initialized = false;
	_state = RunwayTakeoffState::THROTTLE_RAMP;
}

}
