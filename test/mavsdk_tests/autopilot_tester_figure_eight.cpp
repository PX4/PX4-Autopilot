
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

#include "autopilot_tester_figure_eight.h"

#include <cmath>
#include <float.h>
#include <future>
#include <vector>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavlink/development/mavlink_msg_figure_eight_execution_status.h>

using namespace mavsdk::geometry;

void AutopilotTesterFigureEight::execute_figure_eight()
{

	MavlinkPassthrough::CommandInt figure_eight_command;

	auto ct = get_coordinate_transformation();
	const auto global_center = ct.global_from_local(_figure_eight.center);

	figure_eight_command.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	figure_eight_command.target_compid = getMavlinkPassthrough()->get_target_compid();
	figure_eight_command.command = 35; // Figure eight command
	figure_eight_command.frame = MAV_FRAME_GLOBAL_INT;
	figure_eight_command.param1 = _figure_eight.major_axis;
	figure_eight_command.param2 = _figure_eight.minor_axis;
	figure_eight_command.param3 = NAN;
	figure_eight_command.param4 = _figure_eight.orientation;
	figure_eight_command.x = static_cast<int32_t>(global_center.latitude_deg * 1E7);
	figure_eight_command.y = static_cast<int32_t>(global_center.longitude_deg * 1E7);
	figure_eight_command.z = _figure_eight.alt;

	send_custom_mavlink_command(figure_eight_command);
}

void AutopilotTesterFigureEight::set_figure_eight(const double major_axis, const double minor_axis,
		const double orientation, const double home_offset_N, const double home_offset_E, const double rel_alt)
{
	_figure_eight.major_axis = major_axis;
	_figure_eight.minor_axis = minor_axis;
	_figure_eight.orientation = orientation;
	_figure_eight.alt = getHome().absolute_altitude_m + rel_alt;
	_figure_eight.center = {home_offset_N, home_offset_E};
}

void AutopilotTesterFigureEight::check_tracks_figure_eight(std::chrono::seconds timeout, double corridor_radius_m)
{
	auto prom = std::promise<bool> {};
	auto fut = prom.get_future();

	const double cos_or = cos(_figure_eight.orientation);
	const double sin_or = sin(_figure_eight.orientation);

	std::vector<CoordinateTransformation::LocalCoordinate> figure_eight_point_of_interest;
	figure_eight_point_of_interest.push_back(_figure_eight.center);
	figure_eight_point_of_interest.push_back({_figure_eight.center.north_m + cos_or * (abs(_figure_eight.major_axis) - _figure_eight.minor_axis) - sin_or * (- _figure_eight.minor_axis), _figure_eight.center.east_m + sin_or * (abs(_figure_eight.major_axis) - _figure_eight.minor_axis) + cos_or * (- _figure_eight.minor_axis)});
	figure_eight_point_of_interest.push_back({_figure_eight.center.north_m + cos_or * (abs(_figure_eight.major_axis)) - sin_or * 0., _figure_eight.center.east_m + sin_or * (abs(_figure_eight.major_axis)) + cos_or * 0.});
	figure_eight_point_of_interest.push_back({_figure_eight.center.north_m + cos_or * (abs(_figure_eight.major_axis) - _figure_eight.minor_axis) - sin_or * (_figure_eight.minor_axis), _figure_eight.center.east_m + sin_or * (abs(_figure_eight.major_axis) - _figure_eight.minor_axis) + cos_or * (_figure_eight.minor_axis)});
	figure_eight_point_of_interest.push_back({_figure_eight.center.north_m + cos_or * (-abs(_figure_eight.major_axis) + _figure_eight.minor_axis) - sin_or * (- _figure_eight.minor_axis), _figure_eight.center.east_m + sin_or * (-abs(_figure_eight.major_axis) + _figure_eight.minor_axis) + cos_or * (- _figure_eight.minor_axis)});
	figure_eight_point_of_interest.push_back({_figure_eight.center.north_m + cos_or * (-abs(_figure_eight.major_axis)) - sin_or * 0., _figure_eight.center.east_m + sin_or * (-abs(_figure_eight.major_axis)) + cos_or * (0.)});
	figure_eight_point_of_interest.push_back({_figure_eight.center.north_m + cos_or * (-abs(_figure_eight.major_axis) + _figure_eight.minor_axis) - sin_or * (+ _figure_eight.minor_axis), _figure_eight.center.east_m + sin_or * (-abs(_figure_eight.major_axis) + _figure_eight.minor_axis) + cos_or * (_figure_eight.minor_axis)});

	std::vector<int32_t> order_to_fly;

	if (_figure_eight.major_axis > 0) {
		order_to_fly = std::vector<int32_t> {0, 1, 2, 3, 0, 4, 5, 6, 0};

	} else {
		order_to_fly = std::vector<int32_t> {0, 3, 2, 1, 0, 6, 5, 4, 0};
	}

	getTelemetry()->subscribe_position_velocity_ned([&figure_eight_point_of_interest, &prom, corridor_radius_m,
					 &order_to_fly, this](Telemetry::PositionVelocityNed position_velocity_ned) {
		static size_t index{0};
		int32_t close_index{-1};

		for (size_t interest_point_index{0}; interest_point_index < figure_eight_point_of_interest.size();
		     interest_point_index++) {
			if ((abs(position_velocity_ned.position.north_m - figure_eight_point_of_interest[interest_point_index].north_m) <
			     corridor_radius_m) &&
			    (abs(position_velocity_ned.position.east_m - figure_eight_point_of_interest[interest_point_index].east_m) <
			     corridor_radius_m)) {
				close_index = static_cast<int32_t>(interest_point_index);
				break;
			}
		}

		if (close_index >= 0) {
			if (close_index == order_to_fly[index]) { // Still at the same point already found
				// Do nothing

			} else if (close_index == order_to_fly[index + 1]) { // reached the next expected point
				index++;

			} else { // reached an out of order point

				if (index > 0U) { // only set to false if we already hve passed the first center point
					getTelemetry()->subscribe_position_velocity_ned(nullptr);
					prom.set_value(false);
				}
			}
		}

		if (index + 1 == order_to_fly.size()) {
			getTelemetry()->subscribe_position_velocity_ned(nullptr);
			prom.set_value(true);
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	CHECK(fut.get() == true);
}

void AutopilotTesterFigureEight::check_receive_execution_status(std::chrono::seconds timeout)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	auto ct = get_coordinate_transformation();
	const auto global_center = ct.global_from_local(_figure_eight.center);

	add_mavlink_message_callback(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, [&prom, global_center,
	       this](const mavlink_message_t &message) {
		add_mavlink_message_callback(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, nullptr);
		mavlink_figure_eight_execution_status_t status_message;

		mavlink_msg_figure_eight_execution_status_decode(&message, &status_message);
		CHECK(abs(status_message.major_radius - _figure_eight.major_axis) < 1E-4);
		CHECK(abs(status_message.minor_radius - _figure_eight.minor_axis) < 1E-4);
		CHECK(abs(status_message.orientation - _figure_eight.orientation) < 1E-7);
		CHECK(status_message.x == static_cast<int32_t>(global_center.latitude_deg * 1E7));
		CHECK(status_message.y == static_cast<int32_t>(global_center.longitude_deg * 1E7));
		CHECK(abs(status_message.z - _figure_eight.alt) < 1E-4);

		prom.set_value();

		return true;
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}
