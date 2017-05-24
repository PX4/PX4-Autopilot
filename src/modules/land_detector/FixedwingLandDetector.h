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
 * @file FixedwingLandDetector.h
 * Land detector implementation for fixedwing.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <uORB/topics/control_state.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/airspeed.h>

#include "LandDetector.h"

namespace land_detector
{

class FixedwingLandDetector : public LandDetector
{
public:
	FixedwingLandDetector();

protected:
	virtual void _initialize_topics() override;

	virtual void _update_params() override;

	virtual void _update_topics() override;

	virtual bool _get_landed_state() override;

	virtual bool _get_maybe_landed_state() override;

	virtual bool _get_ground_contact_state() override;

	virtual bool _get_freefall_state() override;

	virtual float _get_max_altitude() override;
private:
	struct {
		param_t maxVelocity;
		param_t maxClimbRate;
		param_t maxAirSpeed;
		param_t maxIntVelocity;
	} _paramHandle;

	struct {
		float maxVelocity;
		float maxClimbRate;
		float maxAirSpeed;
		float maxIntVelocity;
	} _params;

	int _controlStateSub;
	int _armingSub;
	int _airspeedSub;

	struct control_state_s _controlState;
	struct actuator_armed_s _arming;
	struct airspeed_s _airspeed;

	float _velocity_xy_filtered;
	float _velocity_z_filtered;
	float _airspeed_filtered;
	float _accel_horz_lp;
};

} // namespace land_detector
