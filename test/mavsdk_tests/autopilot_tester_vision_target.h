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

#include "autopilot_tester.h"
#include <mavsdk/plugins/mavlink_direct/mavlink_direct.h>

class AutopilotTesterVisionTarget : public AutopilotTester
{
public:
	enum class Scenario { Vision, Gnss, VisionAndGnss, Mission, GnssJump, VisionLoss };

	void connect(const std::string uri);
	void set_landing_target(const CoordinateTransformation::LocalCoordinate &offset_from_home_m);
	void precision_land(Scenario scenario, std::chrono::seconds timeout);
	void check_landed_at_target(float acceptance_radius_m);

private:
	void start_landing_mission();
	void check_target_yaw();
	void send_target_observations(Scenario scenario, const Telemetry::GroundTruth &vehicle,
				      bool vision_enabled, double gnss_offset_east_m);
	void send_message(const std::string &name, const std::string &fields);

	std::unique_ptr<MavlinkDirect> _direct;
	Telemetry::GroundTruth _target{};
};
