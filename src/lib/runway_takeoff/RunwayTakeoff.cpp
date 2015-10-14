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
	_start_sp{},
	_target_sp{},
	_throttle_ramp_time(2 * 1e6),
	_runway_takeoff_enabled(this, "TKOFF"),
	_heading_mode(this, "HDG"),
	_nav_alt(this, "NAV_ALT"),
	_takeoff_throttle(this, "MAX_THR"),
	_runway_pitch_sp(this, "PSP"),
	_max_takeoff_pitch(this, "MAX_PITCH"),
	_max_takeoff_roll(this, "MAX_ROLL"),
	_min_airspeed_scaling(this, "AIRSPD_SCL"),
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
	_climbout = true; // this is true until climbout is finished
}

void RunwayTakeoff::update(float airspeed, float alt_agl, int mavlink_fd)
{

	switch (_state) {
	case RunwayTakeoffState::THROTTLE_RAMP:
		if (hrt_elapsed_time(&_initialized_time) > _throttle_ramp_time) {
			_state = RunwayTakeoffState::CLAMPED_TO_RUNWAY;
		}

		break;

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		if (airspeed > _airspeed_min.get() * _min_airspeed_scaling.get()) {
			_state = RunwayTakeoffState::TAKEOFF;
			mavlink_log_info(mavlink_fd, "#Takeoff airspeed reached");
		}

		break;

	case RunwayTakeoffState::TAKEOFF:
		if (alt_agl > _nav_alt.get()) {
			_state = RunwayTakeoffState::CLIMBOUT;
			mavlink_log_info(mavlink_fd, "#Climbout");
		}

		break;

	case RunwayTakeoffState::CLIMBOUT:
		if (alt_agl > _climbout_diff.get()) {
			_climbout = false;
			_state = RunwayTakeoffState::FLY;
			mavlink_log_info(mavlink_fd, "#Navigating to waypoint");
		}

		break;

	default:
		return;
	}
}

/*
 * Returns true as long as we're below navigation altitude
 */
bool RunwayTakeoff::controlYaw()
{
	// keep controlling yaw directly until we start navigation
	return _state < RunwayTakeoffState::CLIMBOUT;
}

/*
 * Returns pitch setpoint to use.
 *
 * Limited (parameter) as long as the plane is on runway. Otherwise
 * use the one from TECS
 */
float RunwayTakeoff::getPitch(float tecsPitch)
{
	if (_state <= RunwayTakeoffState::CLAMPED_TO_RUNWAY) {
		return math::radians(_runway_pitch_sp.get());
	}

	return tecsPitch;
}

/*
 * Returns the roll setpoint to use.
 */
float RunwayTakeoff::getRoll(float navigatorRoll)
{
	// until we have enough ground clearance, set roll to 0
	if (_state < RunwayTakeoffState::CLIMBOUT) {
		return 0.0f;
	}

	// allow some roll during climbout
	else if (_state < RunwayTakeoffState::FLY) {
		return math::constrain(navigatorRoll,
				       math::radians(-_max_takeoff_roll.get()),
				       math::radians(_max_takeoff_roll.get()));
	}

	return navigatorRoll;
}

/*
 * Returns the yaw setpoint to use.
 */
float RunwayTakeoff::getYaw(float navigatorYaw)
{
	return navigatorYaw;
}

/*
 * Returns the throttle setpoint to use.
 *
 * Ramps up in the beginning, until it lifts off the runway it is set to
 * parameter value, then it returns the TECS throttle.
 */
float RunwayTakeoff::getThrottle(float tecsThrottle)
{
	switch (_state) {
	case RunwayTakeoffState::THROTTLE_RAMP: {
			float throttle = hrt_elapsed_time(&_initialized_time) / (float)_throttle_ramp_time *
					 _takeoff_throttle.get();
			return throttle < _takeoff_throttle.get() ?
			       throttle :
			       _takeoff_throttle.get();
		}

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		return _takeoff_throttle.get();

	default:
		return tecsThrottle;
	}
}

bool RunwayTakeoff::resetIntegrators()
{
	// reset integrators if we're still on runway
	return _state < RunwayTakeoffState::TAKEOFF;
}

/*
 * Returns the minimum pitch for TECS to use.
 *
 * In climbout we either want what was set on the waypoint (sp_min) but at least
 * the climbtout minimum pitch (parameter).
 * Otherwise use the minimum that is enforced generally (parameter).
 */
float RunwayTakeoff::getMinPitch(float sp_min, float climbout_min, float min)
{
	if (_state < RunwayTakeoffState::FLY) {
		return math::max(sp_min, climbout_min);
	}

	else {
		return min;
	}
}

/*
 * Returns the maximum pitch for TECS to use.
 *
 * Limited by parameter (if set) until climbout is done.
 */
float RunwayTakeoff::getMaxPitch(float max)
{
	// use max pitch from parameter if set (> 0.1)
	if (_state < RunwayTakeoffState::FLY && _max_takeoff_pitch.get() > 0.1f) {
		return _max_takeoff_pitch.get();
	}

	else {
		return max;
	}
}

/*
 * Returns the "previous" (start) WP for navigation.
 */
math::Vector<2> RunwayTakeoff::getPrevWP()
{
	math::Vector<2> prev_wp;
	prev_wp(0) = (float)_start_sp.lat;
	prev_wp(1) = (float)_start_sp.lon;
	return prev_wp;
}

/*
 * Returns the "current" (target) WP for navigation.
 */
math::Vector<2> RunwayTakeoff::getCurrWP(math::Vector<2> sp_curr_wp)
{
	if (_heading_mode.get() == 0 && _state < RunwayTakeoffState::FLY) {
		// navigating towards calculated direction if heading mode 0 and as long as we're in climbout
		math::Vector<2> curr_wp;
		curr_wp(0) = (float)_target_sp.lat;
		curr_wp(1) = (float)_target_sp.lon;
		return curr_wp;

	} else {
		// navigating towards next mission waypoint
		return sp_curr_wp;
	}
	
}

void RunwayTakeoff::reset()
{
	_initialized = false;
	_state = RunwayTakeoffState::THROTTLE_RAMP;
}

}
