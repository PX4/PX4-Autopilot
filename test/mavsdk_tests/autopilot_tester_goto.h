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

#include "autopilot_tester_loiter.h"

#include <atomic>
#include <chrono>
#include <cstdint>

// Test helper for the multicopter-only Goto mode (NAVIGATION_STATE_GOTO). Adds point-arrival and
// nav_state checks on top of AutopilotTesterLoiter's reposition commands and trajectory checks.
class AutopilotTesterGoto : public AutopilotTesterLoiter
{
public:
	AutopilotTesterGoto() = default;
	~AutopilotTesterGoto() = default;

	// Wait until the current NAVIGATION_STATE (as reported over HEARTBEAT's custom_mode) matches
	// the given PX4_CUSTOM_MAIN_MODE_*/PX4_CUSTOM_SUB_MODE_* pair.
	void wait_until_nav_state(uint8_t expected_main_mode, uint8_t expected_sub_mode, std::chrono::seconds timeout);

	// Wait until horizontally within acceptance_radius_m of target.
	void wait_until_reaches(LocalCoordinate target, float acceptance_radius_m, std::chrono::seconds timeout);

	// Send DO_CHANGE_SPEED with a ground speed.
	void change_speed(float speed_m_s);

	// Assert the horizontal ground speed stays at or below max_speed_m_s for the whole duration.
	void check_max_horizontal_speed(float max_speed_m_s, std::chrono::seconds duration);

	// Horizontal distance between two home-relative coordinates.
	static double horizontal_distance(const LocalCoordinate &a, const LocalCoordinate &b);

private:
	// Subscribed once, never unsubscribed (MAVSDK 3.17.2 deprecated subscribe_message(id, nullptr)
	// and add_mavlink_message_callback exposes no handle for the replacement); polled instead.
	void ensure_heartbeat_subscribed();
	std::atomic<uint32_t> _last_custom_mode{0};
	bool _heartbeat_subscribed{false};
};
