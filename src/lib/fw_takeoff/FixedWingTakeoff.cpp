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
 * @file FixedWingTakeoff.cpp
 * Fixed wing takeoff handling for UAVs including those with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "FixedWingTakeoff.h"
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>

namespace FixedWingTakeoff
{

FixedWingTakeoff::FixedWingTakeoff() :
	SuperBlock(NULL, "FW_TO"),
	_state(),
	_initialized(false),
	_initialized_time(0),
	_init_yaw(0),
	_throttle_ramp_time(2),
	_min_airspeed_time(2),
	_start_wp(),
	_runway_takeoff(this, "RWTO"),
	_heading_mode(this, "HDG"),
	_nav_alt(this, "NAV_ALT"),
	_takeoff_throttle(this, "MAX_THR"),
	_pitch_sp(this, "PSP"),
	_max_takeoff_pitch(this, "MAX_PITCH"),
	_max_takeoff_roll(this, "MAX_ROLL"),
	_min_airspeed_scaling(this, "AIRSPD_SCL"),
	_airspeed_min(this, "FW_AIRSPD_MIN", false),
	_climbout_diff(this, "FW_CLMBOUT_DIFF", false)
{
	updateParams();
}

FixedWingTakeoff::~FixedWingTakeoff()
{

}

void FixedWingTakeoff::init(float yaw, double current_lat, double current_lon, float waypoint_alt)
{
	updateParams();

	_init_yaw = yaw;
	_initialized = true;
	_state = FixedWingTakeoffState::THROTTLE_RAMP;
	_initialized_time = hrt_absolute_time();
	_last_timestamp = 0;
	_airspeed_achieved_integrator = 0;
	_start_wp(0) = (float)current_lat;
	_start_wp(1) = (float)current_lon;
	_waypoint_alt = waypoint_alt;
}

void FixedWingTakeoff::update(float airspeed, float alt, float alt_agl,
			      double current_lat, double current_lon, orb_advert_t *mavlink_log_pub)
{

	float dt = (float)hrt_elapsed_time(&_last_timestamp) * 1e-6f;
	_last_timestamp = hrt_absolute_time();

	switch (_state) {
	case FixedWingTakeoffState::THROTTLE_RAMP:
		if (hrt_elapsed_time(&_initialized_time) > (_throttle_ramp_time * 1000 * 1000)) {
			_state = FixedWingTakeoffState::ACCELERATE;
		}

		break;

	case FixedWingTakeoffState::ACCELERATE:
		if (airspeed > _airspeed_min.get() * _min_airspeed_scaling.get()) {
			_airspeed_achieved_integrator += dt;

		} else {
			_airspeed_achieved_integrator = 0;
		}

		if (_airspeed_achieved_integrator > _min_airspeed_time) {
			_state = FixedWingTakeoffState::TAKEOFF;
			mavlink_log_info(mavlink_log_pub, "#Takeoff airspeed achieved");
		}

		break;

	case FixedWingTakeoffState::TAKEOFF:
		if (alt_agl > _nav_alt.get()) {
			_state = FixedWingTakeoffState::CLIMBOUT;

			/*
			 * If we started in heading hold mode, move the navigation start WP to the current location now.
			 * The navigator will take this as starting point to navigate towards the takeoff WP.
			 */
			if (_heading_mode.get() == 0) {
				_start_wp(0) = (float)current_lat;
				_start_wp(1) = (float)current_lon;
			}

			mavlink_log_info(mavlink_log_pub, "#Takeoff safe altitude achieved, climbing");
		}

		break;

	case FixedWingTakeoffState::CLIMBOUT:

		// move on to navigation if climbout diff is not set or less than altitude error
		if (!(_climbout_diff.get() > 0.001f) || (_waypoint_alt - alt < _climbout_diff.get())) {
			_state = FixedWingTakeoffState::FLY;
			mavlink_log_info(mavlink_log_pub, "#Climbout complete, navigating to waypoint");
		}

		break;

	default:
		break;
	}
}

/*
 * Returns true as long as we're below navigation altitude
 */
bool FixedWingTakeoff::controlYaw()
{
	// keep controlling yaw directly until we start navigation if a runway takeoff
	return runwayTakeoffEnabled() && _state < FixedWingTakeoffState::CLIMBOUT;
}

/*
 * Returns the roll setpoint to use.
 */
float FixedWingTakeoff::getRoll(float navigatorRoll)
{
	if (_state < FixedWingTakeoffState::CLIMBOUT) {
		// until we have enough ground clearance, set roll to 0
		return 0.0f;

	} else if (_state < FixedWingTakeoffState::FLY) {
		// allow some roll during climbout
		return math::constrain(navigatorRoll, math::radians(-_max_takeoff_roll.get()), math::radians(_max_takeoff_roll.get()));
	}

	return navigatorRoll;
}

/*
 * Returns the yaw setpoint to use.
 *
 * In heading hold mode (_heading_mode == 0), it returns initial yaw as long as it's on the
 * runway. When it has enough ground clearance we start navigation towards WP.
 */
float FixedWingTakeoff::getYaw(float navigatorYaw)
{
	if (_heading_mode.get() == 0 && _state < FixedWingTakeoffState::CLIMBOUT) {
		return _init_yaw;

	} else {
		return navigatorYaw;
	}
}

/*
 * Returns the airspeed setpoint to use.
 *
 * FW_AIRSPEED_MIN * FW_TO_AIRSPD_SCL until flying
 */
float FixedWingTakeoff::getAirspeed(float missionAirspeed)
{
	if (_state < FixedWingTakeoffState::FLY) {
		return _min_airspeed_scaling.get() * _airspeed_min.get();

	} else {
		return missionAirspeed;
	}
};

/*
 * Returns the throttle setpoint to use.
 *
 * Ramps up in the beginning, until it lifts off the runway it is set to
 * parameter value, then it returns the TECS throttle.
 */
float FixedWingTakeoff::getThrottle(float tecsThrottle)
{
	switch (_state) {
	case FixedWingTakeoffState::THROTTLE_RAMP: {
			float throttle_ramp_percent = hrt_elapsed_time(&_initialized_time) / (_throttle_ramp_time * 1000.0 * 1000.0);
			return math::min(_takeoff_throttle.get(), throttle_ramp_percent * _takeoff_throttle.get());
		}

	case FixedWingTakeoffState::ACCELERATE:
		return _takeoff_throttle.get();

	default:
		return tecsThrottle;
	}
}

bool FixedWingTakeoff::resetIntegrators()
{
	// reset integrators if we're still on runway
	if (runwayTakeoffEnabled()) {
		return _state < FixedWingTakeoffState::TAKEOFF;

	} else {
		return false;
	}
}

/*
 * Returns the minimum pitch for TECS to use.
 *
 * In climbout we either want what was set on the waypoint (sp_min) but at least
 * the climbout minimum pitch (parameter).
 * Otherwise use the minimum that is enforced generally (parameter).
 */
float FixedWingTakeoff::getMinPitch(float sp_min, float climbout_min, float min)
{
	if (_state <= FixedWingTakeoffState::ACCELERATE) {
		return math::radians(_pitch_sp.get());

	} else if (_state < FixedWingTakeoffState::FLY) {
		return math::max(sp_min, climbout_min);

	} else {
		return min;
	}
}

/*
 * Returns the maximum pitch for TECS to use.
 *
 * Limited by parameter (if set) until climbout is done.
 */
float FixedWingTakeoff::getMaxPitch(float max)
{
	if (_state <= FixedWingTakeoffState::ACCELERATE) {
		return math::radians(_pitch_sp.get());

	} else if (_state < FixedWingTakeoffState::FLY && _max_takeoff_pitch.get() > 0.1f) {
		// use max pitch from parameter if set (> 0.1)
		return _max_takeoff_pitch.get();

	} else {
		return max;
	}
}

/*
 * Returns the "previous" (start) WP for navigation.
 */
math::Vector<2> FixedWingTakeoff::getStartWP()
{
	return _start_wp;
}

void FixedWingTakeoff::reset()
{
	_initialized = false;
	_state = FixedWingTakeoffState::THROTTLE_RAMP;
}

}
