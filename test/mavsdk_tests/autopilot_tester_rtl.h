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

#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/action/action.h>


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
	/* NOTE mavsdk mission upload should be used when possible. Only use this when uploading a mission which is not yet suppported by mavsdk.
	 * Used here to to test the new way of uploading approaches for rally points. */
	void upload_custom_mission(std::chrono::seconds timeout);


private:
	void add_approaches_to_point(mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_coordinate);

	std::unique_ptr<mavsdk::Failure> _failure{};
	std::vector<mavlink_mission_item_int_t> _custom_mission{};
	std::vector<mavsdk::geometry::CoordinateTransformation::LocalCoordinate> _rally_points{};
};
