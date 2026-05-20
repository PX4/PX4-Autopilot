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

#include "autopilot_tester.h"

#include <atomic>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>


class AutopilotTesterRtl : public AutopilotTester
{
public:
	AutopilotTesterRtl() = default;
	~AutopilotTesterRtl() = default;

	void set_rtl_type(int rtl_type);
	void set_rtl_appr_force(int rtl_appr_force);
	void set_takeoff_land_requirements(int req);
	void add_home_to_rally_point();
	void add_home_with_approaches_to_rally_point();
	void add_local_rally_point(mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_coordinate);
	void add_local_rally_with_approaches_point(mavsdk::geometry::CoordinateTransformation::LocalCoordinate
			local_coordinate);
	void connect(const std::string uri);
	void check_rally_point_within(float acceptance_radius_m);
	void check_rtl_approaches(float acceptance_radius_m, std::chrono::seconds timeout);
	void upload_rally_points();

	// Import a QGC .plan and upload both the mission and any geofence polygons/circles it
	// contains. Mission and geofence are shifted by the same offset so the fences stay in
	// their plan-relative positions (same semantics as load_qgc_mission_raw_and_move_here).
	// All fence shapes (inclusion/exclusion, polygon/circle) are cached in local NED for the
	// breach monitor.
	void load_qgc_mission_and_geofence_here(const std::string &plan_file);

	// Subscribe to ground-truth position and assert (latched) that the vehicle never breaches
	// any of the cached fence shapes. A breach is entering an exclusion shape or leaving an
	// inclusion shape. Call after load_qgc_mission_and_geofence_here().
	void start_monitoring_geofence_breach();

	// Unsubscribe and CHECK() that no breach was observed.
	void check_no_geofence_breach();

private:
	struct GeofenceShape {
		enum class Kind { PolygonInclusion, PolygonExclusion, CircleInclusion, CircleExclusion };
		Kind kind;
		std::vector<mavsdk::geometry::CoordinateTransformation::LocalCoordinate> vertices{}; // polygons only
		mavsdk::geometry::CoordinateTransformation::LocalCoordinate center{}; // circles only
		double radius_m{0.0}; // circles only
	};

	void add_approaches_to_point(mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_coordinate);

	static bool point_in_polygon_local(
		double north_m, double east_m,
		const std::vector<mavsdk::geometry::CoordinateTransformation::LocalCoordinate> &poly);

	static bool point_breaches_shape(double north_m, double east_m, const GeofenceShape &shape);

	std::unique_ptr<mavsdk::Failure> _failure{};
	std::vector<mavsdk::MissionRaw::MissionItem> _rally_points{};
	std::vector<mavsdk::geometry::CoordinateTransformation::LocalCoordinate> _local_rally_points{};

	std::vector<GeofenceShape> _geofence_shapes{};
	mavsdk::Telemetry::GroundTruthHandle _geofence_monitor_handle{};
	std::atomic<bool> _geofence_breached{false};
	std::atomic<bool> _geofence_monitor_active{false};
};
