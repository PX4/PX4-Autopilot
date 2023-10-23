
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

#include "autopilot_tester_rtl.h"

#include "math_helpers.h"
#include <cmath>
#include <iostream>
#include <future>
#include <thread>
#include <unistd.h>

#include <mavsdk/plugins/telemetry/telemetry.h>


void AutopilotTesterRtl::connect(const std::string uri)
{
	AutopilotTester::connect(uri);
}

void AutopilotTesterRtl::set_rtl_type(int rtl_type)
{
	CHECK(getParams()->set_param_int("RTL_TYPE", rtl_type) == Param::Result::Success);
}

void AutopilotTesterRtl::set_rtl_appr_force(int rtl_appr_force)
{
	CHECK(getParams()->set_param_int("RTL_APPR_FORCE", rtl_appr_force) == Param::Result::Success);
}

void AutopilotTesterRtl::set_takeoff_land_requirements(int req)
{
	CHECK(getParams()->set_param_int("MIS_TKO_LAND_REQ", req) == Param::Result::Success);
}

void AutopilotTesterRtl::upload_custom_mission(std::chrono::seconds timeout)
{
	std::promise<void> prom;
	auto fut = prom.get_future();

	uint8_t mission_type = _custom_mission[0].mission_type;
	// Register callback to mission item request.
	add_mavlink_message_callback(MAVLINK_MSG_ID_MISSION_REQUEST_INT, [this, mission_type,
	      &prom](const mavlink_message_t &message) {

		mavlink_mission_request_int_t request_int_message;
		mavlink_msg_mission_request_int_decode(&message, &request_int_message);

		if (request_int_message.mission_type == mission_type) {
			// send requested element.
			mavlink_message_t mission_item_int_mavlink_message;
			mavlink_msg_mission_item_int_encode(getMavlinkPassthrough()->get_our_sysid(), getMavlinkPassthrough()->get_our_compid(),
							    &mission_item_int_mavlink_message, &(_custom_mission[request_int_message.seq]));
			send_custom_mavlink_message(mission_item_int_mavlink_message);

			if (request_int_message.seq + 1U == _custom_mission.size()) {
				add_mavlink_message_callback(MAVLINK_MSG_ID_MISSION_REQUEST_INT, nullptr);
				prom.set_value();
			}
		}
	});

	// send mission count
	mavlink_mission_count_t mission_count_message;
	mission_count_message.count = _custom_mission.size();
	mission_count_message.target_system = getMavlinkPassthrough()->get_target_sysid();
	mission_count_message.target_component = getMavlinkPassthrough()->get_target_compid();
	mission_count_message.mission_type = mission_type;

	mavlink_message_t mission_count_mavlink_message;
	mavlink_msg_mission_count_encode(getMavlinkPassthrough()->get_our_sysid(), getMavlinkPassthrough()->get_our_compid(),
					 &mission_count_mavlink_message, &mission_count_message);

	send_custom_mavlink_message(mission_count_mavlink_message);

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTesterRtl::add_home_to_rally_point()
{
	add_local_rally_point({0., 0.});
}

void AutopilotTesterRtl::add_home_with_approaches_to_rally_point()
{
	add_local_rally_point({0., 0.});
	add_approaches_to_point({0., 0.});
}

void AutopilotTesterRtl::add_local_rally_point(mavsdk::geometry::CoordinateTransformation::LocalCoordinate
		local_coordinate)
{
	_rally_points.push_back(local_coordinate);

	mavsdk::geometry::CoordinateTransformation ct(get_coordinate_transformation());
	mavsdk::geometry::CoordinateTransformation::GlobalCoordinate pos(ct.global_from_local(local_coordinate));

	// Set rally point
	mavlink_mission_item_int_t tmp_mission_item;
	tmp_mission_item.param1 = 0.f;
	tmp_mission_item.param2 = 0.f;
	tmp_mission_item.param3 = 0.f;
	tmp_mission_item.param4 = 0.f;
	tmp_mission_item.x = static_cast<int32_t>(pos.latitude_deg * 1E7);
	tmp_mission_item.y = static_cast<int32_t>(pos.longitude_deg * 1E7);
	tmp_mission_item.z = 0.f;
	tmp_mission_item.seq = static_cast<uint16_t>(_custom_mission.size());
	tmp_mission_item.command = MAV_CMD_NAV_RALLY_POINT;
	tmp_mission_item.target_system = getMavlinkPassthrough()->get_target_sysid();
	tmp_mission_item.target_component = getMavlinkPassthrough()->get_target_compid();
	tmp_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	tmp_mission_item.current = 0;
	tmp_mission_item.autocontinue = 0;
	tmp_mission_item.mission_type = MAV_MISSION_TYPE_RALLY;

	_custom_mission.push_back(tmp_mission_item);
}

void AutopilotTesterRtl::add_local_rally_with_approaches_point(
	mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_coordinate)
{
	add_local_rally_point(local_coordinate);
	add_approaches_to_point(local_coordinate);
}

void AutopilotTesterRtl::add_approaches_to_point(mavsdk::geometry::CoordinateTransformation::LocalCoordinate
		local_coordinate)
{

	mavsdk::geometry::CoordinateTransformation ct(get_coordinate_transformation());

	// Set north loiter to alt
	mavsdk::geometry::CoordinateTransformation::LocalCoordinate tmp_coordinate{local_coordinate};
	tmp_coordinate.north_m += 200.;
	mavsdk::geometry::CoordinateTransformation::GlobalCoordinate pos(ct.global_from_local(tmp_coordinate));
	mavlink_mission_item_int_t tmp_mission_item;
	tmp_mission_item.param1 = 0.f;
	tmp_mission_item.param2 = 80.f;
	tmp_mission_item.param3 = 0.f;
	tmp_mission_item.param4 = 0.f;
	tmp_mission_item.x = static_cast<int32_t>(pos.latitude_deg * 1E7);
	tmp_mission_item.y = static_cast<int32_t>(pos.longitude_deg * 1E7);
	tmp_mission_item.z = 15.f;
	tmp_mission_item.seq = static_cast<uint16_t>(_custom_mission.size());
	tmp_mission_item.command = MAV_CMD_NAV_LOITER_TO_ALT;
	tmp_mission_item.target_system = getMavlinkPassthrough()->get_target_sysid();
	tmp_mission_item.target_component = getMavlinkPassthrough()->get_target_compid();
	tmp_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	tmp_mission_item.current = 0;
	tmp_mission_item.autocontinue = 0;
	tmp_mission_item.mission_type = MAV_MISSION_TYPE_RALLY;

	_custom_mission.push_back(tmp_mission_item);

	// Set east loiter to alt
	tmp_coordinate = local_coordinate;
	tmp_coordinate.east_m += 200.;
	pos = ct.global_from_local(tmp_coordinate);
	tmp_mission_item.x = static_cast<int32_t>(pos.latitude_deg * 1E7);
	tmp_mission_item.y = static_cast<int32_t>(pos.longitude_deg * 1E7);
	tmp_mission_item.seq = static_cast<uint16_t>(_custom_mission.size());

	_custom_mission.push_back(tmp_mission_item);
}

void AutopilotTesterRtl::check_rally_point_within(float acceptance_radius_m)
{
	auto old_home(getHome());
	mavsdk::geometry::CoordinateTransformation ct({old_home.latitude_deg, old_home.longitude_deg});
	Telemetry::GroundTruth land_coord{};
	mavsdk::geometry::CoordinateTransformation::GlobalCoordinate pos;
	bool within_rally_point{false};

	for (const auto &rally_point : _rally_points) {
		pos = ct.global_from_local(rally_point);
		land_coord.latitude_deg = pos.latitude_deg;
		land_coord.longitude_deg = pos.longitude_deg;
		within_rally_point |= ground_truth_horizontal_position_close_to(land_coord, acceptance_radius_m);
	}

	CHECK(within_rally_point);
}

void AutopilotTesterRtl::check_rtl_approaches(float acceptance_radius_m, std::chrono::seconds timeout)
{
	auto prom = std::promise<bool> {};
	auto fut = prom.get_future();
	auto ct = get_coordinate_transformation();
	auto return_rtl_alt = getParams()->get_param_float("RTL_RETURN_ALT");
	auto descend_rtl_alt = getParams()->get_param_float("RTL_DESCEND_ALT");
	REQUIRE(return_rtl_alt.first == Param::Result::Success);
	REQUIRE(descend_rtl_alt.first == Param::Result::Success);

	getTelemetry()->subscribe_position_velocity_ned([&prom, acceptance_radius_m, return_rtl_alt, descend_rtl_alt, ct,
	       this](Telemetry::PositionVelocityNed position_velocity_ned) {

		if ((-position_velocity_ned.position.down_m < return_rtl_alt.second - 3.)
		    && (position_velocity_ned.velocity.down_m_s > 0.05)) {
			// We started to loiter down so we should be on the approach loiter
			bool on_approach_loiter(false);

			for (const auto mission_item : _custom_mission) {
				if (mission_item.command == MAV_CMD_NAV_LOITER_TO_ALT) {
					mavsdk::geometry::CoordinateTransformation::LocalCoordinate pos(ct.local_from_global({static_cast<double>(mission_item.x) / 1E7, static_cast<double>(mission_item.y) / 1E7}));
					double rel_distance_to_center = sqrt(sq(position_velocity_ned.position.north_m - pos.north_m) + sq(
							position_velocity_ned.position.east_m - pos.east_m));

					if ((rel_distance_to_center > (mission_item.param2 - acceptance_radius_m))
					    && (rel_distance_to_center > (mission_item.param2 + acceptance_radius_m))) {
						on_approach_loiter |= true;

						if (-position_velocity_ned.position.down_m < descend_rtl_alt.second + 3.) {
							// We reached the altitude
							getTelemetry()->subscribe_position_velocity_ned(nullptr);
							prom.set_value(true);
							return;

						}
					}
				}
			}

			if (!on_approach_loiter) {
				getTelemetry()->subscribe_position_velocity_ned(nullptr);
				prom.set_value(false);

			}
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	REQUIRE(fut.get());
}
