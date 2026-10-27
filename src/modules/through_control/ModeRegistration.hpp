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
 * @file ModeRegistration.hpp
 *
 * Registers THROUGH as an external flight mode through uORB (same mechanism as
 * mc_nn_control / the ROS 2 interface library), configures its control mode
 * flags and answers the commander arming check requests (condition one).
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace through
{

class ModeRegistration
{
public:
	/** Reason why the mode cannot be selected (condition one). None = can run. */
	enum class ArmingFailure : uint8_t {
		None = 0,
		NotInAir,
		AltitudeLow,
		NotLocked,
		TargetStale,
		TargetInvalid,
	};

	enum class State : uint8_t {
		Unregistered,
		WaitingForReply,
		Registered,
		Failed,
	};

	static constexpr int MAX_REGISTRATION_ATTEMPTS = 10;

	/** @param name mode name shown in the GCS (max 24 characters) */
	explicit ModeRegistration(const char *name);

	/** Drive the registration: send the request and retry at 1 Hz until a reply arrives. */
	void update(hrt_abstime now);

	/**
	 * Answer a pending arming_check_request (if any). Must be called every cycle:
	 * the commander only accepts replies within 50 ms of its request.
	 */
	void handleArmingCheckRequest(ArmingFailure failure);

	/** Unregister the mode (only effective while disarmed, VERIFICATION.md V1). */
	void unregister();

	State state() const { return _state; }
	bool registered() const { return _state == State::Registered; }
	bool failed() const { return _state == State::Failed; }
	int8_t modeId() const { return _mode_id; }
	int8_t armingCheckId() const { return _arming_check_id; }

	static const char *armingFailureString(ArmingFailure failure);

private:
	void sendRegistrationRequest(hrt_abstime now);
	void publishControlModeConfig();
	static void addArmingFailureEvent(arming_check_reply_s &reply, ArmingFailure failure);

	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};

	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};

	char _name[sizeof(register_ext_component_request_s::name)] {};

	State _state{State::Unregistered};
	uint64_t _request_id{0};
	int _num_attempts{0};
	hrt_abstime _last_request_time{0};

	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};
};

} // namespace through
