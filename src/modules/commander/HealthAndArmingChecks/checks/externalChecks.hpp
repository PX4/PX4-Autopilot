/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "../Common.hpp"
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

static_assert((1ull << arming_check_reply_s::HEALTH_COMPONENT_INDEX_AVOIDANCE) == (uint64_t)
	      health_component_t::avoidance, "enum definition missmatch");

class ExternalChecks : public HealthAndArmingCheckBase
{
public:
	static constexpr int MAX_NUM_REGISTRATIONS = 8;

	ExternalChecks() = default;
	~ExternalChecks() = default;

	void setExternalNavStates(uint8_t first_external_nav_state, uint8_t last_external_nav_state);

	void checkAndReport(const Context &context, Report &reporter) override;

	bool hasFreeRegistrations() const { return _active_registrations_mask != (1u << MAX_NUM_REGISTRATIONS) - 1; }
	/**
	 * Add registration
	 * @param nav_mode_id associated mode, -1 if none
	 * @param replaces_nav_state replaced mode, -1 if none
	 * @return registration id, or -1
	 */
	int addRegistration(int8_t nav_mode_id, int8_t replaces_nav_state);
	bool removeRegistration(int registration_id, int8_t nav_mode_id);
	void update();

	bool isUnresponsive(int registration_id);

private:
	static constexpr hrt_abstime REQUEST_TIMEOUT = 50_ms;
	static constexpr hrt_abstime UPDATE_INTERVAL = 300_ms;
	static_assert(REQUEST_TIMEOUT < UPDATE_INTERVAL, "keep timeout < update interval");
	static constexpr int NUM_NO_REPLY_UNTIL_UNRESPONSIVE = 3; ///< Mode timeout = this value * UPDATE_INTERVAL

	void checkNonRegisteredModes(const Context &context, Report &reporter) const;

	bool registrationValid(int reg_idx) const { return ((1u << reg_idx) & _active_registrations_mask) != 0; }

	struct Registration {
		~Registration() { delete reply; }

		int8_t nav_mode_id{-1}; ///< associated mode, -1 if none
		int8_t replaces_nav_state{-1};

		uint8_t num_no_response{0};
		bool unresponsive{false};
		uint8_t total_num_unresponsive{0};
		arming_check_reply_s *reply{nullptr};
	};

	unsigned _active_registrations_mask{0};
	Registration _registrations[MAX_NUM_REGISTRATIONS] {};

	uint8_t _first_external_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;
	uint8_t _last_external_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;

	// Current requests (async updates)
	hrt_abstime _last_update{0};
	unsigned _reply_received_mask{0};
	bool _had_timeout{false};

	uint8_t _current_request_id{0};

	uORB::Subscription _arming_check_reply_sub{ORB_ID(arming_check_reply)};

	uORB::Publication<arming_check_request_s> _arming_check_request_pub{ORB_ID(arming_check_request)};
};
