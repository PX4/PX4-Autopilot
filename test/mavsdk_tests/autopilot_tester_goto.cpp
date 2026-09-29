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

#include "autopilot_tester_goto.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <thread>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

using namespace mavsdk;

namespace
{

// Sample rate for the trajectory monitors.
constexpr double POSITION_RATE_HZ = 10.0;

} // namespace

double AutopilotTesterGoto::horizontal_distance(const LocalCoordinate &a, const LocalCoordinate &b)
{
	return std::hypot(a.north_m - b.north_m, a.east_m - b.east_m);
}

void AutopilotTesterGoto::ensure_heartbeat_subscribed()
{
	if (_heartbeat_subscribed) {
		return;
	}

	_heartbeat_subscribed = true;

	add_mavlink_message_callback(MAVLINK_MSG_ID_HEARTBEAT,
	[this](const mavlink_message_t &message) {
		mavlink_heartbeat_t heartbeat;
		mavlink_msg_heartbeat_decode(&message, &heartbeat);
		_last_custom_mode.store(heartbeat.custom_mode);
	});
}

void AutopilotTesterGoto::wait_until_nav_state(uint8_t expected_main_mode, uint8_t expected_sub_mode,
		std::chrono::seconds timeout)
{
	ensure_heartbeat_subscribed();

	// px4_custom_mode layout (src/modules/commander/px4_custom_mode.h), not includable here.
	const auto matches = [this, expected_main_mode, expected_sub_mode]() {
		const uint32_t custom_mode = _last_custom_mode.load();
		const uint8_t main_mode = (custom_mode >> 16) & 0xFF;
		const uint8_t sub_mode = (custom_mode >> 24) & 0xFF;
		return main_mode == expected_main_mode && sub_mode == expected_sub_mode;
	};

	const auto deadline = std::chrono::steady_clock::now() + timeout;
	bool reached = matches();

	while (!reached && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		reached = matches();
	}

	REQUIRE(reached);
}

void AutopilotTesterGoto::wait_until_reaches(LocalCoordinate target, float acceptance_radius_m,
		std::chrono::seconds timeout)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	auto prom = std::promise<void> {};
	auto fut = prom.get_future();
	std::atomic<bool> reported{false};

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Telemetry::PositionVelocityNed sample) {
		if (AutopilotTesterLoiter::horizontal_distance(sample, target) <= acceptance_radius_m && !reported.exchange(true)) {
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
	std::cout << time_str() << "Reached Goto target" << std::endl;
}

void AutopilotTesterGoto::check_max_horizontal_speed(float max_speed_m_s, std::chrono::seconds duration)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Telemetry::PositionVelocityNed sample) {
		CHECK(std::hypot(sample.velocity.north_m_s, sample.velocity.east_m_s) <= max_speed_m_s);
	});

	sleep_for(duration);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
}

void AutopilotTesterGoto::change_speed(float speed_m_s)
{
	REQUIRE(getAction()->set_current_speed(speed_m_s) == Action::Result::Success);
}
