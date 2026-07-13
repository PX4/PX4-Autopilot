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

/**
 * @file detect_and_avoid_crosstrack_test.cpp
 * @brief Tests specific to the crosstrack DAA standard.
 *
 * This file is only built when CONFIG_NAVIGATOR_ADSB_F3442 is disabled; it tests the
 * single-threshold crosstrack behavior and the traffic-heading input requirement
 * that only applies to the crosstrack standard.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "detect_and_avoid_test_common.h"

// Activation fails closed when a crosstrack gate parameter is invalid.
TEST_F(DetectAndAvoidTest, ActivationFailsWithInvalidCrosstrackParams)
{
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	const float negative_value = -10.f;
	param_set(param_handle(px4::params::NAV_TRAFF_A_HOR), &negative_value);

	navigator->get_detect_and_avoid()->on_activation();

	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();
}

// A runtime NAV_TRAFF_AVOID change doesn't re-evaluate the current buffer, but the next conflict
// uses the refreshed action.
TEST_F(DetectAndAvoidTest, RuntimeCrosstrackTerminateUpdateAppliesOnNextConflict)
{
	const int warn_only = action_param_value(DaaAction::kWarnOnly);
	const int terminate_action = action_param_value(DaaAction::kTerminate);
	param_set(param_handle(px4::params::NAV_TRAFF_AVOID), &warn_only);
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{0.f, 0.f, 0.f};
	const float conflict_distance = 200.f;
	const float resolve_distance = 5000.f;
	const float traffic_heading = 3.f * M_PI_2_F;
	const float traffic_direction = M_PI_2_F;
	const float hor_velocity = 30.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	const auto publish_crosstrack_sample = [&](const float distance) {
		publish_traffic_and_check([&]() {
			navigator->get_detect_and_avoid()->fake_traffic(
				fake_traffic_report(14545057, "DDF0A1", distance)
				.with_direction(traffic_direction)
				.with_heading(traffic_heading)
				.with_altitude_diff(0.f)
				.with_velocity(hor_velocity, 0.f)
				.from_ownship(lat_uav, lon_uav, alt_uav));
		});
	};

	publish_crosstrack_sample(conflict_distance);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlHigh);
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.get().has_action);
	EXPECT_FALSE(_vehicle_command_sub.update());

	param_set(param_handle(px4::params::NAV_TRAFF_AVOID), &terminate_action);
	publish_parameter_update();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	navigator->check_traffic();
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_FALSE(_vehicle_command_sub.update());

	publish_crosstrack_sample(resolve_distance);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlNone);
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.get().has_action);

	publish_crosstrack_sample(conflict_distance);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlHigh);
	EXPECT_TRUE(_detect_and_avoid_most_urgent_sub.get().has_action);
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION);
}

// Crosstrack mode fails closed on a NaN/Inf traffic heading even with VALID_HEADING set.
TEST_F(DetectAndAvoidTest, CrosstrackRejectsNonFiniteTrafficHeading)
{
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;

	for (const float heading : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
		drain_detect_and_avoid_topic();

		transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
					  alt_uav, 30.f, 0.f, flags);
		tr.heading = heading;
		publish_transponder_report_and_check(tr);

		EXPECT_FALSE(_detect_and_avoid_sub.update());

		conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
		EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
	}
}

// Crosstrack prediction requires every ownship velocity
TEST_F(DetectAndAvoidTest, CrosstrackRejectsInvalidOwnshipVelocityFlags)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(false);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel, hrt_absolute_time(), 0.f, false, true);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  alt_uav, 0.f, 0.f, flags);
	publish_transponder_report_and_check(tr);

	EXPECT_FALSE(_detect_and_avoid_sub.update());
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);
}
