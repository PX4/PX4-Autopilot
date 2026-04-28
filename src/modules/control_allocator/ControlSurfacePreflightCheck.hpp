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

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

/**
 * Drives the VEHICLE_CMD_ACTUATOR_GROUP_TEST state machine: holds a fixed
 * torque (or collective tilt) on a single axis for a short duration so the
 * operator can verify control-surface motion before flight. Runs only while
 * disarmed.
 */
class ControlSurfacePreflightCheck
{
public:
	static constexpr int NUM_AXES = ControlAllocation::NUM_AXES;
	static constexpr int MAX_NUM_MATRICES = ActuatorEffectiveness::MAX_NUM_MATRICES;

	ControlSurfacePreflightCheck() = default;
	~ControlSurfacePreflightCheck() = default;

	bool isActive() const { return _running; }

	/** Poll vehicle_command for a preflight-check request and start it if conditions allow. */
	void handleCommand(hrt_abstime now, bool armed, bool is_tiltrotor);

	/** Finish the check after PREFLIGHT_CHECK_DURATION, or abort if armed mid-check. */
	void updateState(hrt_abstime now, bool armed);

	/**
	 * Apply the test torque setpoint and/or collective-tilt override for this tick.
	 * Always called so the tilt override is cleared once the check ends.
	 */
	void applyOverrides(matrix::Vector<float, NUM_AXES> (&c)[MAX_NUM_MATRICES], bool is_vtol,
			    ActuatorEffectiveness &effectiveness);

private:
	static constexpr uint32_t PREFLIGHT_CHECK_DURATION = 500'000;  // 500 ms

	void sendAck(uint8_t result, hrt_abstime now);

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	bool _running{false};
	uint8_t _axis{0};
	float _input{0.0f};
	vehicle_command_s _last_command{};
	hrt_abstime _started{0};
};
