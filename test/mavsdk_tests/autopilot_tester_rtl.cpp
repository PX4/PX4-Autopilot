
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

void AutopilotTesterRtl::upload_rally_points()
{
	REQUIRE(getMissionRaw()->upload_rally_points(_rally_points) == MissionRaw::Result::Success);
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
	_local_rally_points.push_back(local_coordinate);

	mavsdk::geometry::CoordinateTransformation ct(get_coordinate_transformation());
	mavsdk::geometry::CoordinateTransformation::GlobalCoordinate pos(ct.global_from_local(local_coordinate));

	// Set rally point
	mavsdk::MissionRaw::MissionItem tmp_mission_item;
	tmp_mission_item.param1 = 0.f;
	tmp_mission_item.param2 = 0.f;
	tmp_mission_item.param3 = 0.f;
	tmp_mission_item.param4 = 0.f;
	tmp_mission_item.x = static_cast<int32_t>(pos.latitude_deg * 1E7);
	tmp_mission_item.y = static_cast<int32_t>(pos.longitude_deg * 1E7);
	tmp_mission_item.z = 0.f;
	tmp_mission_item.seq = static_cast<uint16_t>(_rally_points.size());
	tmp_mission_item.command = MAV_CMD_NAV_RALLY_POINT;
	tmp_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	// FIXME: this is currently required with MAVSDK even though it doesn't make much sense for rally points
	tmp_mission_item.current = _rally_points.empty() ? 1 : 0;
	tmp_mission_item.autocontinue = 0;
	tmp_mission_item.mission_type = MAV_MISSION_TYPE_RALLY;

	_rally_points.push_back(tmp_mission_item);
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
	mavsdk::MissionRaw::MissionItem tmp_mission_item;
	tmp_mission_item.param1 = 0.f;
	tmp_mission_item.param2 = 80.f;
	tmp_mission_item.param3 = 0.f;
	tmp_mission_item.param4 = 0.f;
	tmp_mission_item.x = static_cast<int32_t>(pos.latitude_deg * 1E7);
	tmp_mission_item.y = static_cast<int32_t>(pos.longitude_deg * 1E7);
	tmp_mission_item.z = 15.f;
	tmp_mission_item.seq = static_cast<uint16_t>(_rally_points.size());
	tmp_mission_item.command = MAV_CMD_NAV_LOITER_TO_ALT;
	tmp_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	// FIXME: this is currently required with MAVSDK even though it doesn't make much sense for rally points
	tmp_mission_item.current = _rally_points.empty() ? 1 : 0;
	tmp_mission_item.autocontinue = 0;
	tmp_mission_item.mission_type = MAV_MISSION_TYPE_RALLY;

	_rally_points.push_back(tmp_mission_item);

	// Set east loiter to alt
	tmp_coordinate = local_coordinate;
	tmp_coordinate.east_m += 200.;
	pos = ct.global_from_local(tmp_coordinate);
	tmp_mission_item.x = static_cast<int32_t>(pos.latitude_deg * 1E7);
	tmp_mission_item.y = static_cast<int32_t>(pos.longitude_deg * 1E7);
	tmp_mission_item.seq = static_cast<uint16_t>(_rally_points.size());

	_rally_points.push_back(tmp_mission_item);
}

void AutopilotTesterRtl::check_rally_point_within(float acceptance_radius_m)
{
	auto old_home(getHome());
	mavsdk::geometry::CoordinateTransformation ct({old_home.latitude_deg, old_home.longitude_deg});
	Telemetry::GroundTruth land_coord{};
	mavsdk::geometry::CoordinateTransformation::GlobalCoordinate pos;
	bool within_rally_point{false};

	for (const auto &rally_point : _local_rally_points) {
		pos = ct.global_from_local(rally_point);
		land_coord.latitude_deg = pos.latitude_deg;
		land_coord.longitude_deg = pos.longitude_deg;
		within_rally_point |= ground_truth_horizontal_position_close_to(land_coord, acceptance_radius_m);
	}

	CHECK(within_rally_point);
}

void AutopilotTesterRtl::load_qgc_mission_and_geofence_here(const std::string &plan_file)
{
	auto import_result = getMissionRaw()->import_qgroundcontrol_mission(plan_file);
	REQUIRE(import_result.first == mavsdk::MissionRaw::Result::Success);

	auto &data = import_result.second;
	REQUIRE(!data.mission_items.empty());

	// Same offset semantics as move_mission_raw_here, but applied to mission and geofence
	// together so the fences keep their plan-relative positions.
	const auto pos = getTelemetry()->position();
	REQUIRE(std::isfinite(pos.latitude_deg));
	REQUIRE(std::isfinite(pos.longitude_deg));

	const int32_t offset_x = data.mission_items[0].x - static_cast<int32_t>(1e7 * pos.latitude_deg);
	const int32_t offset_y = data.mission_items[0].y - static_cast<int32_t>(1e7 * pos.longitude_deg);

	// Only items in a global position frame carry lat/lon in x/y. For MAV_FRAME_MISSION
	// items (e.g. RTL, DO_* commands) x/y are generic params p5/p6, so offsetting them
	// corrupts the params and the item is rejected by the mission-param validation.
	auto is_global_position_frame = [](uint32_t frame) {
		switch (frame) {
		case MAV_FRAME_GLOBAL:
		case MAV_FRAME_GLOBAL_INT:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
		case MAV_FRAME_GLOBAL_TERRAIN_ALT:
		case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
			return true;

		default:
			return false;
		}
	};

	auto apply_offset = [&](std::vector<mavsdk::MissionRaw::MissionItem> &items) {
		for (auto &item : items) {
			if (!is_global_position_frame(item.frame)) {
				continue;
			}

			item.x -= offset_x;
			item.y -= offset_y;
		}
	};

	apply_offset(data.mission_items);
	apply_offset(data.geofence_items);

	REQUIRE(getMissionRaw()->upload_mission(data.mission_items) == mavsdk::MissionRaw::Result::Success);

	_geofence_shapes.clear();

	if (!data.geofence_items.empty()) {
		REQUIRE(getMissionRaw()->upload_geofence(data.geofence_items) == mavsdk::MissionRaw::Result::Success);

		// Parse MAVLink fence items into shape records in local NED.
		// Polygon vertex items (5001/5002) are emitted as a run with param1 = vertex count;
		// circle items (5003/5004) are one-shot with param1 = radius_m.
		constexpr uint16_t kCmdPolygonInclusion = 5001;
		constexpr uint16_t kCmdPolygonExclusion = 5002;
		constexpr uint16_t kCmdCircleInclusion  = 5003;
		constexpr uint16_t kCmdCircleExclusion  = 5004;

		const auto ct = get_coordinate_transformation();

		auto flush_open_polygon = [this](GeofenceShape & open) {
			if (!open.vertices.empty()) {
				_geofence_shapes.push_back(std::move(open));
			}

			open = GeofenceShape{};
		};

		GeofenceShape open{};
		size_t vertex_target = 0;

		for (const auto &item : data.geofence_items) {
			const double lat = static_cast<double>(item.x) / 1e7;
			const double lon = static_cast<double>(item.y) / 1e7;
			const auto local = ct.local_from_global({lat, lon});

			if (item.command == kCmdPolygonInclusion || item.command == kCmdPolygonExclusion) {
				const auto kind = (item.command == kCmdPolygonInclusion)
						  ? GeofenceShape::Kind::PolygonInclusion
						  : GeofenceShape::Kind::PolygonExclusion;

				// Start a new polygon when kind changes or the previous one is full.
				if (open.vertices.empty() || open.kind != kind
				    || open.vertices.size() >= vertex_target) {
					flush_open_polygon(open);
					open.kind = kind;
					vertex_target = static_cast<size_t>(item.param1);
				}

				open.vertices.push_back(local);

			} else if (item.command == kCmdCircleInclusion || item.command == kCmdCircleExclusion) {
				flush_open_polygon(open);
				GeofenceShape circle{};
				circle.kind = (item.command == kCmdCircleInclusion)
					      ? GeofenceShape::Kind::CircleInclusion
					      : GeofenceShape::Kind::CircleExclusion;
				circle.center = local;
				circle.radius_m = static_cast<double>(item.param1);
				_geofence_shapes.push_back(std::move(circle));
			}
		}

		flush_open_polygon(open);
	}

	sleep_for(std::chrono::seconds(1));
}

void AutopilotTesterRtl::start_monitoring_geofence_breach()
{
	REQUIRE(!_geofence_shapes.empty()); // load_qgc_mission_and_geofence_here() must run first
	REQUIRE(!_geofence_monitor_active);

	auto ct = get_coordinate_transformation();
	_geofence_breached.store(false);
	_geofence_monitor_active.store(true);

	_geofence_monitor_handle = getTelemetry()->subscribe_ground_truth(
	[this, ct](Telemetry::GroundTruth gt) {
		if (!_geofence_monitor_active.load()) {
			return;
		}

		if (std::isnan(gt.latitude_deg) || std::isnan(gt.longitude_deg)) {
			return;
		}

		const auto local = ct.local_from_global({gt.latitude_deg, gt.longitude_deg});

		for (size_t i = 0; i < _geofence_shapes.size(); ++i) {
			if (point_breaches_shape(local.north_m, local.east_m, _geofence_shapes[i])) {
				if (!_geofence_breached.exchange(true)) {
					std::cout << time_str() << "GEOFENCE BREACH (shape #" << i
						  << ") at NED (" << local.north_m << ", "
						  << local.east_m << ")\n";
				}

				break;
			}
		}
	});
}

void AutopilotTesterRtl::check_no_geofence_breach()
{
	if (_geofence_monitor_active.exchange(false)) {
		getTelemetry()->unsubscribe_ground_truth(_geofence_monitor_handle);
	}

	CHECK(!_geofence_breached.load());
}

bool AutopilotTesterRtl::point_in_polygon_local(
	double north_m, double east_m,
	const std::vector<mavsdk::geometry::CoordinateTransformation::LocalCoordinate> &poly)
{
	bool inside = false;
	const size_t n = poly.size();

	for (size_t i = 0, j = n - 1; i < n; j = i++) {
		const double ni = poly[i].north_m;
		const double ei = poly[i].east_m;
		const double nj = poly[j].north_m;
		const double ej = poly[j].east_m;

		const bool crosses = ((ei > east_m) != (ej > east_m))
				     && (north_m < (nj - ni) * (east_m - ei) / (ej - ei) + ni);

		if (crosses) {
			inside = !inside;
		}
	}

	return inside;
}

bool AutopilotTesterRtl::point_breaches_shape(double north_m, double east_m, const GeofenceShape &shape)
{
	switch (shape.kind) {
	case GeofenceShape::Kind::PolygonExclusion:
		return shape.vertices.size() >= 3
		       && point_in_polygon_local(north_m, east_m, shape.vertices);

	case GeofenceShape::Kind::PolygonInclusion:
		return shape.vertices.size() >= 3
		       && !point_in_polygon_local(north_m, east_m, shape.vertices);

	case GeofenceShape::Kind::CircleExclusion: {
			const double dn = north_m - shape.center.north_m;
			const double de = east_m - shape.center.east_m;
			return std::sqrt(dn * dn + de * de) < shape.radius_m;
		}

	case GeofenceShape::Kind::CircleInclusion: {
			const double dn = north_m - shape.center.north_m;
			const double de = east_m - shape.center.east_m;
			return std::sqrt(dn * dn + de * de) > shape.radius_m;
		}
	}

	return false;
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

	Telemetry::PositionVelocityNedHandle handle = getTelemetry()->subscribe_position_velocity_ned(
				[&prom, &handle, acceptance_radius_m, return_rtl_alt, descend_rtl_alt, ct,
	       this](Telemetry::PositionVelocityNed position_velocity_ned) {

		if ((-position_velocity_ned.position.down_m < return_rtl_alt.second - 3.)
		    && (position_velocity_ned.velocity.down_m_s > 0.05)) {
			// We started to loiter down so we should be on the approach loiter
			bool on_approach_loiter(false);

			for (const auto mission_item : _rally_points) {
				if (mission_item.command == MAV_CMD_NAV_LOITER_TO_ALT) {
					mavsdk::geometry::CoordinateTransformation::LocalCoordinate pos(ct.local_from_global({static_cast<double>(mission_item.x) / 1E7, static_cast<double>(mission_item.y) / 1E7}));
					double rel_distance_to_center = sqrt(sq(position_velocity_ned.position.north_m - pos.north_m) + sq(
							position_velocity_ned.position.east_m - pos.east_m));

					if ((rel_distance_to_center > (mission_item.param2 - acceptance_radius_m))
					    && (rel_distance_to_center > (mission_item.param2 + acceptance_radius_m))) {
						on_approach_loiter |= true;

						if (-position_velocity_ned.position.down_m < descend_rtl_alt.second + 3.) {
							// We reached the altitude
							getTelemetry()->unsubscribe_position_velocity_ned(handle);
							prom.set_value(true);
							return;

						}
					}
				}
			}

			if (!on_approach_loiter) {
				getTelemetry()->unsubscribe_position_velocity_ned(handle);
				prom.set_value(false);

			}
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
	REQUIRE(fut.get());
}
