/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester_loiter.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <future>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

using namespace mavsdk;
using namespace mavsdk::geometry;

namespace
{

// MAVLink command IDs (kept as literals to match the style of the figure-eight tester and avoid
// pulling extra mavlink enum headers).
constexpr uint16_t CMD_DO_ORBIT = 34;
constexpr uint16_t CMD_DO_FIGURE_EIGHT = 35;
constexpr uint16_t CMD_DO_REPOSITION = 192;

// MAV_DO_REPOSITION_FLAGS_CHANGE_MODE: request a switch into Hold if not already there.
constexpr float REPOSITION_FLAG_CHANGE_MODE = 1.f;

// Sample rate for the trajectory monitors.
constexpr double POSITION_RATE_HZ = 10.0;

using Sample = Telemetry::PositionVelocityNed;

} // namespace

double AutopilotTesterLoiter::horizontal_distance(const Sample &sample, const LocalCoordinate &c)
{
	return std::hypot(sample.position.north_m - c.north_m, sample.position.east_m - c.east_m);
}

void AutopilotTesterLoiter::enter_hold_and_wait(std::chrono::seconds timeout)
{
	REQUIRE(getAction()->hold() == Action::Result::Success);

	const auto start = std::chrono::steady_clock::now();

	while (getTelemetry()->flight_mode() != Telemetry::FlightMode::Hold) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		REQUIRE(std::chrono::steady_clock::now() - start < timeout);
	}
}

void AutopilotTesterLoiter::command_orbit(LocalCoordinate center, float radius_m, float rel_alt_m)
{
	const auto global = get_coordinate_transformation().global_from_local(center);

	MavlinkPassthrough::CommandInt cmd{};
	cmd.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	cmd.target_compid = getMavlinkPassthrough()->get_target_compid();
	cmd.command = CMD_DO_ORBIT;
	cmd.frame = MAV_FRAME_GLOBAL_INT;
	cmd.param1 = radius_m; // radius, sign encodes direction (positive == clockwise)
	cmd.param2 = NAN; // tangential velocity: use default
	cmd.param3 = NAN;
	cmd.param4 = NAN;
	cmd.x = static_cast<int32_t>(global.latitude_deg * 1e7);
	cmd.y = static_cast<int32_t>(global.longitude_deg * 1e7);
	cmd.z = getHome().absolute_altitude_m + rel_alt_m;

	send_custom_mavlink_command(cmd);
}

void AutopilotTesterLoiter::command_figure_eight(LocalCoordinate center, float major_axis_m, float minor_axis_m,
		float rel_alt_m)
{
	const auto global = get_coordinate_transformation().global_from_local(center);

	MavlinkPassthrough::CommandInt cmd{};
	cmd.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	cmd.target_compid = getMavlinkPassthrough()->get_target_compid();
	cmd.command = CMD_DO_FIGURE_EIGHT;
	cmd.frame = MAV_FRAME_GLOBAL_INT;
	cmd.param1 = major_axis_m; // major radius, sign encodes direction
	cmd.param2 = minor_axis_m; // minor radius
	cmd.param3 = NAN;
	cmd.param4 = 0.f; // orientation
	cmd.x = static_cast<int32_t>(global.latitude_deg * 1e7);
	cmd.y = static_cast<int32_t>(global.longitude_deg * 1e7);
	cmd.z = getHome().absolute_altitude_m + rel_alt_m;

	send_custom_mavlink_command(cmd);
}

void AutopilotTesterLoiter::command_reposition(LocalCoordinate target, float rel_alt_m)
{
	const auto global = get_coordinate_transformation().global_from_local(target);

	MavlinkPassthrough::CommandInt cmd{};
	cmd.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	cmd.target_compid = getMavlinkPassthrough()->get_target_compid();
	cmd.command = CMD_DO_REPOSITION;
	cmd.frame = MAV_FRAME_GLOBAL_INT;
	cmd.param1 = -1.f; // ground speed: use default
	cmd.param2 = REPOSITION_FLAG_CHANGE_MODE;
	cmd.param3 = 0.f;
	cmd.param4 = NAN; // yaw: unchanged
	cmd.x = static_cast<int32_t>(global.latitude_deg * 1e7);
	cmd.y = static_cast<int32_t>(global.longitude_deg * 1e7);
	cmd.z = getHome().absolute_altitude_m + rel_alt_m;

	send_custom_mavlink_command(cmd);
}

void AutopilotTesterLoiter::command_hold_here()
{
	MavlinkPassthrough::CommandInt cmd{};
	cmd.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	cmd.target_compid = getMavlinkPassthrough()->get_target_compid();
	cmd.command = CMD_DO_REPOSITION;
	cmd.frame = MAV_FRAME_GLOBAL_INT;
	cmd.param1 = -1.f; // ground speed: use default
	cmd.param2 = REPOSITION_FLAG_CHANGE_MODE;
	cmd.param3 = 0.f;
	cmd.param4 = NAN; // yaw: unchanged
	cmd.x = INT32_MAX; // latitude ignored -> NaN
	cmd.y = INT32_MAX; // longitude ignored -> NaN
	cmd.z = NAN; // altitude ignored -> keep current
	send_custom_mavlink_command(cmd);
}

void AutopilotTesterLoiter::command_reposition_altitude(float rel_alt_m)
{
	MavlinkPassthrough::CommandInt cmd{};
	cmd.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	cmd.target_compid = getMavlinkPassthrough()->get_target_compid();
	cmd.command = CMD_DO_REPOSITION;
	cmd.frame = MAV_FRAME_GLOBAL_INT;
	cmd.param1 = -1.f; // ground speed: use default
	cmd.param2 = REPOSITION_FLAG_CHANGE_MODE;
	cmd.param3 = 0.f;
	cmd.param4 = NAN; // yaw: unchanged
	cmd.x = INT32_MAX; // latitude ignored -> NaN (position unchanged)
	cmd.y = INT32_MAX; // longitude ignored -> NaN (position unchanged)
	cmd.z = getHome().absolute_altitude_m + rel_alt_m; // only altitude changes
	send_custom_mavlink_command(cmd);
}

void AutopilotTesterLoiter::wait_until_on_loiter(LocalCoordinate center, float radius_m, float radius_tol_m,
		std::chrono::seconds settle, std::chrono::seconds timeout)
{
	// Flying out to the loiter circle only crosses the radial band once and then stays on it, so the
	// first band entry marks establishment; `settle` (measured in simulator time) then lets the loiter
	// stabilize before the caller asserts on it.
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	auto prom = std::promise<void> {};
	auto fut = prom.get_future();
	std::atomic<bool> reported{false};

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		const double d = horizontal_distance(sample, center);

		if (d >= (radius_m - radius_tol_m) && d <= (radius_m + radius_tol_m) && !reported.exchange(true)) {
			prom.set_value();
		}
	});

	// wait_for is only an upper bound (wall-clock); the future resolves as soon as the loiter is
	// reached, which happens quickly under a high simulation speed factor.
	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);

	sleep_for(settle);
	std::cout << time_str() << "Established on loiter" << std::endl;
}

void AutopilotTesterLoiter::wait_until_relative_altitude_above(float threshold_m, std::chrono::seconds timeout)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	auto prom = std::promise<void> {};
	auto fut = prom.get_future();
	std::atomic<bool> reported{false};

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		const float rel_alt = -sample.position.down_m; // home-relative, up-positive

		if (rel_alt >= threshold_m && !reported.exchange(true)) {
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
}

void AutopilotTesterLoiter::check_stays_on_loiter(LocalCoordinate center, float radius_m, float radius_tol_m,
		std::chrono::seconds duration)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		const double d = horizontal_distance(sample, center);
		CHECK(d >= (radius_m - radius_tol_m));
		CHECK(d <= (radius_m + radius_tol_m));
	});

	sleep_for(duration);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
}

void AutopilotTesterLoiter::check_stays_within(LocalCoordinate center, float max_radius_m,
		std::chrono::seconds duration)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		CHECK(horizontal_distance(sample, center) <= max_radius_m);
	});

	sleep_for(duration);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
}

void AutopilotTesterLoiter::check_never_reaches(LocalCoordinate point, float min_dist_m,
		std::chrono::seconds duration)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		CHECK(horizontal_distance(sample, point) >= min_dist_m);
	});

	sleep_for(duration);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
}

void AutopilotTesterLoiter::check_holds_altitude(float rel_alt_m, float tol_m, std::chrono::seconds duration)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		const float rel_alt = -sample.position.down_m; // home-relative, up-positive
		CHECK(std::abs(rel_alt - rel_alt_m) <= tol_m);
	});

	sleep_for(duration);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
}

void AutopilotTesterLoiter::check_climbs_on_loiter(LocalCoordinate center, float radius_m, float radius_tol_m,
		float target_rel_alt_m, float alt_tol_m, std::chrono::seconds timeout)
{
	getTelemetry()->set_rate_position_velocity_ned(POSITION_RATE_HZ);

	auto prom = std::promise<void> {};
	auto fut = prom.get_future();
	std::atomic<bool> reported{false};

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
	[&](Sample sample) {
		// Invariant: never leave the radial band around the loiter center while climbing.
		const double d = horizontal_distance(sample, center);
		CHECK(d >= (radius_m - radius_tol_m));
		CHECK(d <= (radius_m + radius_tol_m));

		const float rel_alt = -sample.position.down_m;

		if (std::abs(rel_alt - target_rel_alt_m) <= alt_tol_m && !reported.exchange(true)) {
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	getTelemetry()->unsubscribe_position_velocity_ned(handle);
	std::cout << time_str() << "Climbed to target altitude while staying on loiter" << std::endl;
}

float AutopilotTesterLoiter::current_relative_altitude()
{
	// Use NED down (up-positive) to match the altitude source used by the trajectory monitors.
	return -getTelemetry()->position_velocity_ned().position.down_m;
}

AutopilotTesterLoiter::LocalCoordinate AutopilotTesterLoiter::current_local_position()
{
	const auto pos = getTelemetry()->position_velocity_ned().position;
	return LocalCoordinate{pos.north_m, pos.east_m};
}

float AutopilotTesterLoiter::default_loiter_radius()
{
	const auto result = getParams()->get_param_float("NAV_LOITER_RAD");
	CHECK(result.first == Param::Result::Success);
	return result.second;
}
