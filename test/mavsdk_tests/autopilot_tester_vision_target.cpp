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

#include "autopilot_tester_vision_target.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Vector3d = std::array<double, 3>;

namespace
{
constexpr double mission_offset_north_m = -3.0;
constexpr auto telemetry_timeout = 2s;
constexpr auto observation_poll_interval = 1ms;
}

void AutopilotTesterVisionTarget::connect(const std::string uri)
{
	AutopilotTester::connect(uri);
	_direct = std::make_unique<MavlinkDirect>(get_system());

	// TODO: Change development.xml to common.xml when TARGET_* are promoted.
	std::ifstream xml(std::string(PX4_MAVLINK_XML_DIR) + "/development.xml");
	REQUIRE(xml.good());
	const std::string content{std::istreambuf_iterator<char>(xml), std::istreambuf_iterator<char>()};
	REQUIRE(_direct->load_custom_xml(content) == MavlinkDirect::Result::Success);

	// Without VTE, TARGET_RELATIVE can bypass the estimator and feed precland directly.
	const auto enabled = getParams()->get_param_int("VTE_EN");
	REQUIRE(enabled.first == Param::Result::Success);
	REQUIRE(enabled.second == 1);
}

void AutopilotTesterVisionTarget::set_landing_target(
	const CoordinateTransformation::LocalCoordinate &offset_from_home_m)
{
	store_home();
	_target = getHome();
	REQUIRE(std::isfinite(_target.absolute_altitude_m));
	CoordinateTransformation home_frame({_target.latitude_deg, _target.longitude_deg});
	const auto target = home_frame.global_from_local(offset_from_home_m);
	_target.latitude_deg = target.latitude_deg;
	_target.longitude_deg = target.longitude_deg;
}

void AutopilotTesterVisionTarget::start_landing_mission()
{
	// An approximate mission location: vision must correct the landing to the true pad.
	CoordinateTransformation target_frame({_target.latitude_deg, _target.longitude_deg});
	const auto coarse_pad = target_frame.global_from_local({mission_offset_north_m, 0.0});
	MissionRaw::MissionItem waypoint{};
	waypoint.current = 1;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	waypoint.autocontinue = 1;
	waypoint.mission_type = MAV_MISSION_TYPE_MISSION;
	waypoint.param2 = 1.f; // Waypoint acceptance radius in metres.
	waypoint.param4 = NAN;
	waypoint.x = std::llround(coarse_pad.latitude_deg * 1e7);
	waypoint.y = std::llround(coarse_pad.longitude_deg * 1e7);
	waypoint.z = getTelemetry()->position().relative_altitude_m;
	auto land = waypoint;
	land.seq = 1;
	land.current = 0;
	land.command = MAV_CMD_NAV_LAND;
	land.param2 = 2.f; // Required precision landing.
	land.param3 = NAN;
	land.z = 0.f;
	REQUIRE(getMissionRaw()->upload_mission({waypoint, land}) == MissionRaw::Result::Success);
	sleep_for(1s); // Allow PX4 to publish mission availability, as in the other mission helpers.
	REQUIRE(getMissionRaw()->start_mission() == MissionRaw::Result::Success);
}

void AutopilotTesterVisionTarget::precision_land(Scenario scenario, std::chrono::seconds timeout)
{
	// VTE_AID_MASK: target GNSS position (bit 0), vehicle GNSS velocity (bit 1),
	// vision (bit 2), mission position (bit 3). Home aiding is always disabled.
	int aid_mask = 7;

	switch (scenario) {
	case Scenario::Vision: aid_mask = 4; break;

	case Scenario::Gnss: aid_mask = 3; break;

	case Scenario::Mission: aid_mask = 14; break;

	default: break;
	}

	REQUIRE(getParams()->set_param_int("VTE_AID_MASK", aid_mask) == Param::Result::Success);

	if (scenario == Scenario::Mission) {
		start_landing_mission();

	} else {
		MavlinkPassthrough::CommandLong command{};
		command.target_sysid = getMavlinkPassthrough()->get_target_sysid();
		command.target_compid = getMavlinkPassthrough()->get_target_compid();
		command.command = MAV_CMD_DO_SET_MODE;
		command.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
		command.param2 = 4; // PX4_CUSTOM_MAIN_MODE_AUTO
		command.param3 = 9; // PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND (required precision landing)
		REQUIRE(getMavlinkPassthrough()->send_command_long(command) == MavlinkPassthrough::Result::Success);
	}

	// Use simulator time for flight deadlines so speedup does not change the scenario.
	const auto start_us = getTelemetry()->ground_truth().timestamp_us;
	REQUIRE(start_us != 0);
	const auto timeout_us = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count();
	auto last_sample_received = Clock::now();
	uint64_t last_sample = start_us;
	uint64_t vision_loss_us = 0;
	unsigned spoofed_samples = 0;
	bool vision_enabled = scenario != Scenario::Mission;
	bool mission_correction_checked = false;
	constexpr float minimum_disturbance_height_m = 3.f;
	constexpr double acquisition_radius_m = 0.75;
	constexpr double corrected_radius_m = 0.5;
	CoordinateTransformation target_frame({_target.latitude_deg, _target.longitude_deg});

	while (getTelemetry()->armed() && last_sample - start_us < static_cast<uint64_t>(timeout_us)) {
		const auto vehicle = getTelemetry()->ground_truth();

		if (vehicle.timestamp_us != last_sample) {
			REQUIRE(std::isfinite(vehicle.latitude_deg));
			REQUIRE(std::isfinite(vehicle.longitude_deg));
			REQUIRE(std::isfinite(vehicle.absolute_altitude_m));
			const auto relative = target_frame.local_from_global({vehicle.latitude_deg, vehicle.longitude_deg});
			const double distance_m = std::hypot(relative.north_m, relative.east_m);
			const float height_m = vehicle.absolute_altitude_m - _target.absolute_altitude_m;

			if (scenario == Scenario::Mission) {
				// Below the default search altitude, so searching alone cannot expose vision.
				constexpr float mission_vision_height_m = 9.f;

				if (!vision_enabled && height_m < mission_vision_height_m) {
					// Verify the aircraft is over the mission position before exposing the true pad.
					const double mission_error_m = std::hypot(relative.north_m - mission_offset_north_m, relative.east_m);
					CAPTURE(mission_error_m, distance_m);
					REQUIRE(mission_error_m < acquisition_radius_m);
					REQUIRE(distance_m > std::abs(mission_offset_north_m) - acquisition_radius_m);
					vision_enabled = true;
				}

				constexpr float correction_check_height_m = 4.f;

				if (vision_enabled && height_m < correction_check_height_m) {
					// Allow the controller time to complete the 3 m correction before checking alignment.
					CAPTURE(distance_m);
					REQUIRE(distance_m < corrected_radius_m);
					mission_correction_checked = true;
				}
			}

			constexpr float vision_loss_height_m = 8.f;
			constexpr auto minimum_bias_overlap = 4s;

			if (scenario == Scenario::VisionLoss && vision_enabled
			    && height_m < vision_loss_height_m && height_m > minimum_disturbance_height_m
			    && distance_m < corrected_radius_m
			    && std::chrono::microseconds(vehicle.timestamp_us - start_us) >= minimum_bias_overlap) {
				vision_enabled = false;
				vision_loss_us = vehicle.timestamp_us;
			}

			// Spoof only after acquisition, while descending with time left to react.
			constexpr float spoof_start_height_m = 8.f;
			const bool start_spoof = scenario == Scenario::GnssJump && distance_m < acquisition_radius_m
						 && height_m < spoof_start_height_m && height_m > minimum_disturbance_height_m;
			const bool spoof_gnss = start_spoof || spoofed_samples > 0;
			constexpr double gnss_bias_east_m = 2.0;
			double gnss_offset_east_m = scenario == Scenario::VisionLoss ? gnss_bias_east_m : 0.0;

			if (spoof_gnss) {
				constexpr double gnss_jump_east_m = 200.0;
				gnss_offset_east_m = gnss_jump_east_m;
			}

			send_target_observations(scenario, vehicle, vision_enabled, gnss_offset_east_m);

			if (spoof_gnss) {
				++spoofed_samples;
				CAPTURE(distance_m);
				// Catch a transient excursion even if vision subsequently recovers the landing.
				constexpr double spoof_max_distance_m = 2.0;
				REQUIRE(distance_m < spoof_max_distance_m);
			}

			if (vision_loss_us != 0) {
				CAPTURE(distance_m);
				REQUIRE(distance_m < corrected_radius_m);
			}

			last_sample = vehicle.timestamp_us;
			last_sample_received = Clock::now();
		}

		// Do not keep publishing a stale observation if ground-truth telemetry stops.
		if (Clock::now() - last_sample_received >= telemetry_timeout) {
			FAIL("Ground-truth telemetry stopped during precision landing");
		}

		std::this_thread::sleep_for(observation_poll_interval);
	}

	REQUIRE_FALSE(getTelemetry()->armed());

	if (scenario == Scenario::Mission) {
		REQUIRE(mission_correction_checked);
	}

	if (scenario == Scenario::GnssJump) {
		constexpr unsigned minimum_spoofed_samples = 10;
		REQUIRE(spoofed_samples >= minimum_spoofed_samples);
	}

	if (scenario == Scenario::VisionLoss) {
		constexpr auto minimum_vision_outage = 8s;
		REQUIRE(vision_loss_us != 0);
		REQUIRE(std::chrono::microseconds(last_sample - vision_loss_us) >= minimum_vision_outage);
	}
}

void AutopilotTesterVisionTarget::send_target_observations(Scenario scenario, const Telemetry::GroundTruth &vehicle,
		bool vision_enabled, double gnss_offset_east_m)
{
	if (scenario != Scenario::Gnss && vision_enabled) {
		CoordinateTransformation target_frame({_target.latitude_deg, _target.longitude_deg});
		const auto vehicle_ne = target_frame.local_from_global({vehicle.latitude_deg, vehicle.longitude_deg});
		const Vector3d target_ned_m{-vehicle_ne.north_m, -vehicle_ne.east_m,
					    vehicle.absolute_altitude_m - _target.absolute_altitude_m};
		// A fixed virtual sensor yawed 90 degrees from NED: sensor (x,y,z) = (east,-north,down).
		// This exercises q_sensor without relying on the aircraft's estimated attitude.
		const Vector3d target_sensor_m{target_ned_m[1], -target_ned_m[0], target_ned_m[2]};
		constexpr int frame_other = 3; // TARGET_OBS_FRAME_OTHER
		constexpr Vector3d position_std_m{0.1, 0.1, 0.1};
		std::ostringstream fields;
		fields << std::setprecision(10) << "\"frame\":" << frame_other
		       << ",\"type\":" << LANDING_TARGET_TYPE_VISION_FIDUCIAL
		       << ",\"x\":" << target_sensor_m[0] << ",\"y\":" << target_sensor_m[1] << ",\"z\":" << target_sensor_m[2]
		       << ",\"pos_std\":[" << position_std_m[0] << ',' << position_std_m[1] << ',' << position_std_m[2] << ']'
		       << R"(,"q_sensor":[0.70710678,0,0,0.70710678])";
		send_message("TARGET_RELATIVE", fields.str());
	}

	if (scenario != Scenario::Vision && scenario != Scenario::Mission) {
		constexpr int position_capability = 1; // TARGET_ABSOLUTE_SENSOR_CAPABILITY_POSITION
		constexpr double degrees_to_mavlink = 1e7;
		CoordinateTransformation target_frame({_target.latitude_deg, _target.longitude_deg});
		const auto reported_target = target_frame.global_from_local({0.0, gnss_offset_east_m});
		std::ostringstream fields;
		fields << std::setprecision(10)
		       << "\"sensor_capabilities\":" << position_capability
		       << ",\"lat\":" << std::llround(reported_target.latitude_deg * degrees_to_mavlink)
		       << ",\"lon\":" << std::llround(reported_target.longitude_deg * degrees_to_mavlink)
		       << ",\"alt\":" << _target.absolute_altitude_m << R"(,"position_std":[0.1,0.1])";
		send_message("TARGET_ABSOLUTE", fields.str());
	}
}

void AutopilotTesterVisionTarget::send_message(const std::string &name, const std::string &fields)
{
	// Generate observations immediately, with UNIX timestamps as required by TARGET_*.
	// No TIMESYNC: these flight tests use PX4's unsynchronized arrival-time fallback.
	const auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
					  std::chrono::system_clock::now().time_since_epoch()).count();
	MavlinkDirect::MavlinkMessage message{};
	message.message_name = name;
	message.fields_json = "{\"timestamp\":" + std::to_string(timestamp_us) + "," + fields + "}";
	REQUIRE(_direct->send_message(message) == MavlinkDirect::Result::Success);
}

void AutopilotTesterVisionTarget::check_landed_at_target(float acceptance_radius_m)
{
	CHECK(getTelemetry()->landed_state() == Telemetry::LandedState::OnGround);
	// Wait for a fresh simulator position after disarming before checking touchdown.
	const auto previous_sample = getTelemetry()->ground_truth().timestamp_us;
	const auto deadline = Clock::now() + telemetry_timeout;

	while (getTelemetry()->ground_truth().timestamp_us <= previous_sample && Clock::now() < deadline) {
		std::this_thread::sleep_for(observation_poll_interval);
	}

	REQUIRE(getTelemetry()->ground_truth().timestamp_us > previous_sample);
	CoordinateTransformation target_frame({_target.latitude_deg, _target.longitude_deg});
	const auto vehicle = getTelemetry()->ground_truth();
	const auto relative = target_frame.local_from_global({vehicle.latitude_deg, vehicle.longitude_deg});
	const double landing_error_m = std::hypot(relative.north_m, relative.east_m);
	std::cout << "Landing error: " << landing_error_m << " m" << std::endl;
	CAPTURE(landing_error_m, acceptance_radius_m);
	CHECK(landing_error_m < acceptance_radius_m);
}
