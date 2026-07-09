/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file ExternalAvoidance.hpp
 *
 * Fuses an externally-computed avoidance setpoint (streamed by a companion
 * computer via MAVLink SET_POSITION_TARGET_LOCAL_NED) into the active
 * Position/Mission trajectory setpoint.
 *
 * This class contains NO obstacle-detection or path-planning logic. The external
 * companion owns all of that and simply streams the desired velocity/acceleration
 * deviation. PX4's only job here is to *accept and fuse* that setpoint while in
 * Position or Mission mode, then automatically resume the underlying setpoint when
 * the external stream goes stale.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/external_avoidance_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>

class ExternalAvoidance : public ModuleParams
{
public:
	ExternalAvoidance(ModuleParams *parent);
	~ExternalAvoidance() = default;

	/**
	 * Fuse a fresh external avoidance setpoint into @p setpoint, in place.
	 *
	 * No-op (leaves @p setpoint untouched, so the mission/position setpoint is
	 * "resumed") when the feature is disabled, the vehicle is not in Position or
	 * Mission mode, or no fresh external setpoint has arrived within EAV_TIMEOUT.
	 *
	 * @param setpoint  active trajectory setpoint from the flight task, modified in place
	 * @param nav_state current navigation state (vehicle_status_s::NAVIGATION_STATE_*)
	 */
	void modifySetpoint(trajectory_setpoint_s &setpoint, uint8_t nav_state);

	/** @return true if the last modifySetpoint() call actually applied a deviation */
	bool isActive() const { return _active; }

private:
	uORB::Subscription _external_avoidance_setpoint_sub{ORB_ID(external_avoidance_setpoint)};

	bool _active{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::EAV_EN>) _param_eav_en,
		(ParamFloat<px4::params::EAV_TIMEOUT>) _param_eav_timeout
	)
};
