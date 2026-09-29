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
 * @file goto_mode.h
 *
 * Multicopter-only Goto: fly to a repositioned target, then hold, via goto_setpoint/GotoControl.
 */

#pragma once

#include "navigator_mode.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/goto_setpoint.h>

class Goto : public NavigatorMode
{
public:
	Goto(Navigator *navigator);
	~Goto() = default;

	void initialize() override {}

	void on_activation() override;
	void on_active() override;
	void on_inactive() override;

	/**
	 * Set a new global target (from DO_REPOSITION). Applied right away if Goto is active, otherwise
	 * on activation if that follows within 500ms (the mode switch requested with the same command).
	 * @param heading [rad] NAN for the heading from MPC_YAW_MODE
	 * @param cruising_speed [m/s] <= 0 for the default speed
	 */
	void setTarget(double lat, double lon, float alt, float heading, float cruising_speed);

	/** Change only the altitude of the current target (DO_CHANGE_ALTITUDE). */
	void setAltitude(float alt);

	/** @return false if there is no target */
	bool getTarget(double &lat, double &lon, float &alt, float &heading) const;

private:
	struct Target {
		double lat{NAN};
		double lon{NAN};
		float alt{NAN};
		float heading{NAN};
		float cruising_speed{-1.f};
	};

	/** Apply the pending target if it is fresh. */
	void applyPendingTarget();

	/** Hold at the braking stop point in front of the vehicle. */
	void setTargetToStopPoint();

	/** Heading from MPC_YAW_MODE for a target without heading. Locked once there is no direction to point in. */
	float headingFromYawMode();

	void publishGotoSetpoint();

	uORB::Publication<goto_setpoint_s> _goto_setpoint_pub{ORB_ID(goto_setpoint)};

	MapProjection _geo_projection{};

	// Sticky global target, re-projected into the current EKF frame every cycle and republished
	// (goto_setpoint has a 500ms freshness lease).
	Target _target{};
	bool _target_valid{false};

	Target _pending_target{};
	hrt_abstime _pending_target_timestamp{0};

	// Read directly, MPC_YAW_MODE only exists on builds with multicopter support
	param_t _param_handle_mpc_yaw_mode{PARAM_INVALID};
	int32_t _param_mpc_yaw_mode{0};

	float _locked_heading{NAN};
};
