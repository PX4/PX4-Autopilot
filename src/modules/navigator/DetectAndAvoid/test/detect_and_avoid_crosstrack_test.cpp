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
 * single-threshold crosstrack behavior and the input requirements (finite traffic and
 * ownship headings) that only apply to the crosstrack standard.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "detect_and_avoid_test_common.h"

// WHY: Activation must fail closed when a crosstrack gate parameter is invalid.
// WHAT: Inject a negative crosstrack separation and verify the module refuses to activate.
TEST_F(DetectAndAvoidTest, ActivationFailsWithInvalidCrosstrackParams)
{
	// GIVEN: The module activated cleanly with the default parameter set.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	// WHEN: A required crosstrack parameter is invalid.
	const float negative_value = -10.f;
	param_set(param_handle(px4::params::NAV_TRAFF_A_HOR), &negative_value);

	navigator->get_detect_and_avoid()->on_activation();

	// THEN: Activation fails closed.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();
}

// WHY: Runtime NAV_TRAFF_AVOID action updates must not re-evaluate the current buffer, but later conflict escalations must use the refreshed action value.
// WHAT: Start with a crosstrack conflict in warn-only mode, switch NAV_TRAFF_AVOID to terminate at runtime, verify no immediate command is sent, then clear and re-trigger the conflict and confirm termination is commanded.
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
			navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", distance, traffic_direction, traffic_heading,
					0.f, hor_velocity, 0.f, 1, lat_uav, lon_uav, alt_uav);
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

// WHY: Crosstrack mode depends on a finite traffic heading and must fail closed if the heading value is NaN or Inf.
// WHAT: Publish invalid heading values with the VALID_HEADING flag set, and verify no output or buffered conflict is produced.
TEST_F(DetectAndAvoidTest, CrosstrackRejectsNonFiniteTrafficHeading)
{
	// GIVEN: DetectAndAvoid is built in crosstrack mode.
	recreate_navigator();

	// GIVEN: Ownship is initialized and traffic headings are marked as valid.
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

	// WHEN: Traffic is published with non-finite heading values.
	for (const float heading : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
		drain_detect_and_avoid_topic();

		transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
					  alt_uav, 30.f, 0.f, flags);
		tr.heading = heading;
		publish_transponder_report_and_check(tr);

		// THEN: No detect_and_avoid output is published and no conflict is buffered.
		EXPECT_FALSE(_detect_and_avoid_sub.update());

		conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
		EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
	}
}

// WHY: A hovering ownship has no meaningful course-over-ground, so crosstrack mode must use the local-position yaw instead of atan2f(0, 0).
// WHAT: Publish a hovering ownship with unavailable local yaw and verify crosstrack processing fails instead of inventing a north heading.
TEST_F(DetectAndAvoidTest, HoveringCrosstrackRejectsNonFiniteOwnshipHeading)
{
	// GIVEN: DetectAndAvoid is built in crosstrack mode.
	recreate_navigator();

	// GIVEN: Ownship is hovering and local-position yaw is unavailable.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{0.f, 0.f, 0.f};

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(false);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel, hrt_absolute_time(), std::numeric_limits<float>::quiet_NaN());
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  alt_uav, 30.f, 0.f, flags);
	tr.heading = 3.f * M_PI_2_F;

	// WHEN: A traffic report that would otherwise be processed by crosstrack mode is received.
	publish_transponder_report_and_check(tr);

	// THEN: No output is published and no conflict is buffered because ownship yaw is not finite.
	EXPECT_FALSE(_detect_and_avoid_sub.update());

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
}
