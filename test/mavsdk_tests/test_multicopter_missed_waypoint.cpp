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

#include "autopilot_tester.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace
{
struct PositionSample {
	mavlink_global_position_int_t position{};
	mavlink_home_position_t home{};
	int sequence{-1};
	bool have_home{false};
};

struct SampleBuffer {
	std::mutex mutex;
	std::deque<PositionSample> positions;
	mavlink_home_position_t home{};
	int sequence{-1};
	bool have_home{false};
};

class MissedWaypointRecoveryTester : public AutopilotTester
{
public:
	void run()
	{
		connect(connection_url);
		wait_until_ready();

		set_param_float("NAV_ACC_RAD", 2.f);
		set_param_float("NAV_MC_ALT_RAD", 0.8f);
		set_param_float("MPC_XY_VEL_MAX", 12.f);
		set_param_float("MPC_XY_CRUISE", 10.f);
		set_param_float("MPC_Z_V_AUTO_DN", 1.f);
		REQUIRE(getTelemetry()->set_rate_position(50.f) == Telemetry::Result::Success);

		auto state = std::make_shared<SampleBuffer>();
		auto *mavlink = getMavlinkPassthrough();
		const uint8_t system_id = mavlink->get_target_sysid();

		struct Subscriptions {
			MavlinkPassthrough *mavlink;
			std::vector<std::pair<uint16_t, MavlinkPassthrough::MessageHandle>> handles;

			~Subscriptions()
			{
				for (const auto &handle : handles) {
					mavlink->unsubscribe_message(handle.first, handle.second);
				}
			}
		} subscriptions{mavlink, {}};

		subscriptions.handles.emplace_back(MAVLINK_MSG_ID_HOME_POSITION,
			mavlink->subscribe_message(MAVLINK_MSG_ID_HOME_POSITION, [state, system_id](const mavlink_message_t &message) {
			if (message.sysid != system_id) {
				return;
			}

			std::lock_guard<std::mutex> lock(state->mutex);
			mavlink_msg_home_position_decode(&message, &state->home);
			state->have_home = true;
		}));

		subscriptions.handles.emplace_back(MAVLINK_MSG_ID_MISSION_CURRENT,
			mavlink->subscribe_message(MAVLINK_MSG_ID_MISSION_CURRENT, [state, system_id](const mavlink_message_t &message) {
			if (message.sysid != system_id) {
				return;
			}

			mavlink_mission_current_t progress{};
			mavlink_msg_mission_current_decode(&message, &progress);
			std::lock_guard<std::mutex> lock(state->mutex);
			state->sequence = progress.seq;
		}));

		subscriptions.handles.emplace_back(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
			mavlink->subscribe_message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, [state, system_id](const mavlink_message_t &message) {
			if (message.sysid != system_id) {
				return;
			}

			PositionSample sample{};
			mavlink_msg_global_position_int_decode(&message, &sample.position);
			std::lock_guard<std::mutex> lock(state->mutex);
			sample.home = state->home;
			sample.sequence = state->sequence;
			sample.have_home = state->have_home;
			state->positions.push_back(sample);
		}));

		const auto home = get_home_position();
		const auto coordinate_transformation = get_coordinate_transformation();
		auto mission_item = [&](uint32_t sequence, uint32_t command, double north_m, float altitude_m) {
			const auto global = coordinate_transformation.global_from_local({north_m, 0.0});
			MissionRaw::MissionItem item{};
			item.seq = sequence;
			item.command = command;
			item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			item.current = sequence == 0 ? 1U : 0U;
			item.autocontinue = 1;
			item.param1 = 0.f;
			item.param2 = command == MAV_CMD_NAV_WAYPOINT ? 2.f : 0.f;
			item.param3 = 0.f;
			item.param4 = 0.f;
			item.x = static_cast<int32_t>(std::lround(global.latitude_deg * 1e7));
			item.y = static_cast<int32_t>(std::lround(global.longitude_deg * 1e7));
			item.z = altitude_m;
			item.mission_type = MAV_MISSION_TYPE_MISSION;
			return item;
		};

		std::vector<MissionRaw::MissionItem> mission{
			mission_item(0, MAV_CMD_NAV_TAKEOFF, 0.0, 20.f),
			mission_item(1, MAV_CMD_NAV_WAYPOINT, 60.0, 20.f),
			mission_item(2, MAV_CMD_NAV_WAYPOINT, 100.0, 20.f),
			mission_item(3, MAV_CMD_NAV_WAYPOINT, 140.0, 20.f),
			mission_item(4, MAV_CMD_NAV_LAND, 140.0, 0.f)
		};
		const CoordinateTransformation target_frame({mission[2].x / 1e7, mission[2].y / 1e7});

		REQUIRE(getMissionRaw()->upload_mission(mission) == MissionRaw::Result::Success);
		sleep_for(std::chrono::seconds(1));
		arm();
		REQUIRE(getMissionRaw()->start_mission() == MissionRaw::Result::Success);

		bool home_change_sent = false;
		bool home_change_accepted = false;
		bool approached = false;
		bool crossed = false;
		bool missed = false;
		bool returned = false;
		bool accepted_after_return = false;
		bool diverged = false;
		double target_home_altitude_m = static_cast<double>(home.altitude) / 1000.0 - 5.0;
		double last_along_m = -1e9;
		double maximum_distance_m = 0.0;
		double outward_since_s = -1.0;
		double outward_start_distance_m = 0.0;

		const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);

		while (std::chrono::steady_clock::now() < deadline && !accepted_after_return && !diverged) {
			std::deque<PositionSample> samples;

			{
				std::lock_guard<std::mutex> lock(state->mutex);
				samples.swap(state->positions);
			}

			for (const auto &sample : samples) {
				const auto local = target_frame.local_from_global({sample.position.lat / 1e7, sample.position.lon / 1e7});
				const double along_m = local.north_m;
				const double cross_m = local.east_m;

				if (!home_change_sent && sample.have_home && sample.sequence == 2
				    && along_m >= -12.0 && along_m <= -6.0 && std::fabs(cross_m) < 1.0
				    && sample.position.vx > 500
				    && std::fabs(sample.position.alt / 1000.0 - (sample.home.altitude / 1000.0 + 20.0)) < 0.8) {
					target_home_altitude_m = sample.home.altitude / 1000.0 - 5.0;
					MavlinkPassthrough::CommandInt command{};
					command.target_sysid = system_id;
					command.target_compid = mavlink->get_target_compid();
					command.command = MAV_CMD_DO_SET_HOME;
					command.frame = MAV_FRAME_GLOBAL_INT;
					command.param1 = 0.f;
					command.param2 = 0.f;
					command.param3 = 0.f;
					command.param4 = 0.f;
					command.x = sample.home.latitude;
					command.y = sample.home.longitude;
					command.z = static_cast<float>(target_home_altitude_m);
					home_change_sent = true;
					home_change_accepted = mavlink->send_command_int(command) == MavlinkPassthrough::Result::Success;

					MavlinkPassthrough::CommandLong request{};
					request.target_sysid = system_id;
					request.target_compid = mavlink->get_target_compid();
					request.command = MAV_CMD_REQUEST_MESSAGE;
					request.param1 = MAVLINK_MSG_ID_HOME_POSITION;
					(void)mavlink->send_command_long(request);
				}

				if (!home_change_sent || !home_change_accepted || !sample.have_home
				    || std::fabs(sample.home.altitude / 1000.0 - target_home_altitude_m) >= 0.15) {
					last_along_m = along_m;
					continue;
				}

				const double altitude_error_m = sample.position.alt / 1000.0 - (target_home_altitude_m + 20.0);
				const bool pending = sample.sequence == 2
						&& getTelemetry()->flight_mode() == Telemetry::FlightMode::Mission;
				const double distance_m = std::hypot(along_m, cross_m);

				if (pending && along_m < -2.0 && std::fabs(cross_m) < 2.0) {
					approached = true;
				}

				if (approached && pending && last_along_m <= 0.0 && along_m > 0.0
				    && std::fabs(cross_m) < 2.0 && std::fabs(altitude_error_m) > 0.8) {
					crossed = true;
				}

				if (crossed && pending && along_m > 2.1 && std::fabs(altitude_error_m) > 0.8) {
					missed = true;
				}

				if (missed) {
					maximum_distance_m = std::max(maximum_distance_m, distance_m);

					if (maximum_distance_m > 2.1 && distance_m <= 2.0 && std::fabs(altitude_error_m) <= 0.8
					    && sample.position.vx < -10) {
						returned = true;
					}

					if (returned && sample.sequence > 2) {
						accepted_after_return = true;
					}

					const double radial_velocity_m_s = distance_m > 0.01
							 ? (along_m * sample.position.vx / 100.0 + cross_m * sample.position.vy / 100.0) / distance_m
							 : 0.0;

					if (pending && distance_m > 2.1 && radial_velocity_m_s > 0.3 && std::fabs(altitude_error_m) <= 0.8) {
						const double time_s = sample.position.time_boot_ms / 1000.0;

						if (outward_since_s < 0.0) {
							outward_since_s = time_s;
							outward_start_distance_m = distance_m;
						}

						if (time_s - outward_since_s >= 10.0 && distance_m - outward_start_distance_m >= 10.0) {
							diverged = true;
						}

					} else {
						outward_since_s = -1.0;
					}
				}

				last_along_m = along_m;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		CAPTURE(home_change_sent);
		CAPTURE(home_change_accepted);
		CAPTURE(approached);
		CAPTURE(crossed);
		CAPTURE(missed);
		CAPTURE(returned);
		CAPTURE(accepted_after_return);
		CAPTURE(diverged);
		REQUIRE(home_change_sent);
		REQUIRE(home_change_accepted);
		REQUIRE(missed);
		REQUIRE_FALSE(diverged);
		REQUIRE(returned);
		REQUIRE(accepted_after_return);

		bool mission_complete = false;
		const auto mission_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);

		while (std::chrono::steady_clock::now() < mission_deadline) {
			const auto result = getMissionRaw()->is_mission_finished();

			if (result.first == MissionRaw::Result::Success && result.second) {
				mission_complete = true;
				break;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}

		REQUIRE(mission_complete);
		wait_until_disarmed(std::chrono::seconds(60));
	}

private:
	void set_param_float(const std::string &name, float value)
	{
		REQUIRE(getParams()->set_param_float(name, value) == Param::Result::Success);
		const auto result = getParams()->get_param_float(name);
		REQUIRE(result.first == Param::Result::Success);
		REQUIRE(std::fabs(result.second - value) < 0.01f);
	}
};
} // namespace

TEST_CASE("Multicopter recovers after missing a waypoint during a home-altitude change", "[multicopter][mission]")
{
	MissedWaypointRecoveryTester tester;
	tester.run();
}
