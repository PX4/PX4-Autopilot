/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file uorb_blocks.h
 *
 * uorb block library code
 */

#pragma once

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_set_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <poll.h>

extern "C" {
#include <geo/geo.h>
}

#include "../blocks.hpp"
#include "UOrbSubscription.hpp"
#include "UOrbPublication.hpp"

namespace control
{

/**
 * Waypoint Guidance block
 */
class __EXPORT BlockWaypointGuidance : public SuperBlock
{
private:
	BlockLimitSym _xtYawLimit;
	BlockP _xt2Yaw;
	float _psiCmd;
public:
	BlockWaypointGuidance(SuperBlock *parent, const char *name);
	virtual ~BlockWaypointGuidance();
	void update(vehicle_global_position_s &pos,
		    vehicle_attitude_s &att,
		    vehicle_global_position_setpoint_s &posCmd,
		    vehicle_global_position_setpoint_s &lastPosCmd);
	float getPsiCmd() { return _psiCmd; }
};

/**
 * UorbEnabledAutopilot
 */
class __EXPORT BlockUorbEnabledAutopilot : public SuperBlock
{
protected:
	// subscriptions
	UOrbSubscription<vehicle_attitude_s> _att;
	UOrbSubscription<vehicle_attitude_setpoint_s> _attCmd;
	UOrbSubscription<vehicle_rates_setpoint_s> _ratesCmd;
	UOrbSubscription<vehicle_global_position_s> _pos;
	UOrbSubscription<vehicle_global_position_set_triplet_s> _posCmd;
	UOrbSubscription<manual_control_setpoint_s> _manual;
	UOrbSubscription<vehicle_status_s> _status;
	UOrbSubscription<parameter_update_s> _param_update;
	// publications
	UOrbPublication<actuator_controls_s> _actuators;
public:
	BlockUorbEnabledAutopilot(SuperBlock *parent, const char *name);
	virtual ~BlockUorbEnabledAutopilot();
};

} // namespace control

