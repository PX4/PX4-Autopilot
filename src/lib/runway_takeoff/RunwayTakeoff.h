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
 * @file RunwayTakeoff.h
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef RUNWAYTAKEOFF_H
#define RUNWAYTAKEOFF_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <mavlink/mavlink_log.h>
#include <mathlib/mathlib.h>

namespace runwaytakeoff
{

enum RunwayTakeoffState {
	THROTTLE_RAMP = 0, /**<  */
	CLAMPED_TO_RUNWAY = 1, /**<  */
	TAKEOFF = 2, /**<  */
	FLY = 3 /**<  */
};

class __EXPORT RunwayTakeoff : public control::SuperBlock
{
public:
	RunwayTakeoff();
	~RunwayTakeoff();

	void init(float yaw);
	void update(float airspeed, float alt_agl, int mavlink_fd);

	RunwayTakeoffState getState() { return _state; };
	bool isInitialized() { return _initialized; };

	bool runwayTakeoffEnabled() { return (bool)_runway_takeoff_enabled.get(); };

	bool controlYaw();
	bool climbout() { return _climbout; };
	float getPitch(float tecsPitch);
	float getRoll(float navigatorRoll);
	float getYaw(float navigatorYaw);
	float getThrottle(float tecsThrottle);
	bool resetIntegrators();
	float getMinAirspeedScaling() { return _min_airspeed_scaling; };
	float getMinPitch(float sp_min, float climbout_min, float min);

	void reset();

protected:
private:
	/** state variables **/
	RunwayTakeoffState _state;
	bool _initialized;
	hrt_abstime _initialized_time;
	float _init_yaw;
	bool _climbout;

	/** magic values **/
	float _min_airspeed_scaling;

	/** parameters **/
	control::BlockParamInt _runway_takeoff_enabled;
	control::BlockParamInt _runway_takeoff_heading;
	control::BlockParamFloat _runway_takeoff_nav_alt;
	control::BlockParamFloat _runway_takeoff_throttle;
	control::BlockParamFloat _runway_takeoff_pitch_sp;
	control::BlockParamFloat _airspeed_min;
	control::BlockParamFloat _climbout_diff;

};

}

#endif // RUNWAYTAKEOFF_H
