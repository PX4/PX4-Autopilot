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

#pragma once

#include <ActuatorEffectiveness.hpp>
#include <ControlAllocation.hpp>

#include <math.h>
#include <mathlib/math/Limits.hpp>
#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

/**
 * Drives the VEHICLE_CMD_ACTUATOR_GROUP_TEST state machine: holds a fixed
 * torque, thrust or collective tilt on a single functional group for a short
 * duration so the operator can verify actuator motion before flight.
 *
 * Pre-conditions (enforced at start, cancelled if not given while running):
 *  - torque and collective tilt groups: prearmed or armed
 *  - thrust groups: armed
 *  - landed
 *
 * Losing the required state mid-check results in a CANCELLED ack.
 */
class ActuatorGroupPreflightCheck
{
public:
	static constexpr int NUM_AXES = ControlAllocation::NUM_AXES;
	static constexpr int MAX_NUM_MATRICES = ActuatorEffectiveness::MAX_NUM_MATRICES;

	ActuatorGroupPreflightCheck() = default;
	~ActuatorGroupPreflightCheck() = default;

	bool isActive() const { return _running; }

	// Poll vehicle_command for a group test request and start it if conditions allow.
	void handleCommand(hrt_abstime now, bool is_tiltrotor);

	// Finish the check after the group-specific duration, or abort if the required arming state is lost.
	void updateState(hrt_abstime now);

	// Apply the test torque/thrust setpoint and/or collective tilt override.
	void applyOverrides(matrix::Vector<float, NUM_AXES> (&c)[MAX_NUM_MATRICES], bool is_vtol,
			    ActuatorEffectiveness &effectiveness);

private:
	static constexpr hrt_abstime PREFLIGHT_CHECK_DURATION_US = 500'000;  // 500 ms

	static bool isThrust(uint8_t group);
	static bool isKnownGroup(uint8_t group);

	// Returns
	//  - nullptr, if the command is acceptable
	//  - char* with rejection reason, if not
	// On rejection, also writes the appropriate VEHICLE_CMD_RESULT_* into ack_result.
	const char *validateCommand(uint8_t group, bool is_tiltrotor, uint8_t &ack_result);

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	void start(hrt_abstime now, uint8_t group, float input, uint8_t nav_state, const vehicle_command_s &cmd);
	void stop(uint8_t result, hrt_abstime now);
	void sendAck(const vehicle_command_s &cmd, uint8_t result, hrt_abstime now);

	bool _running{false};
	struct ActiveCheck {
		uint8_t           group;
		float             input;
		hrt_abstime       started;
		uint8_t           started_nav_state;
		vehicle_command_s last_command;
	} _active_check{};
};
