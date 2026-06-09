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
 * @file detect_and_avoid_f3442_test.cpp
 * @brief Tests specific to the ASTM F3442 DAA standard.
 *
 * This file is only built when CONFIG_NAVIGATOR_ADSB_F3442 is enabled; it tests the
 * four nested alert volumes, the per-level actions, and the default velocity handling
 * that only exist in F3442 builds.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "detect_and_avoid_test_common.h"

// WHY: Activation must fail closed when an F3442 alert-volume parameter is invalid.
// WHAT: Inject negative volume radii and verify the module refuses to activate.
TEST_F(DetectAndAvoidTest, ActivationFailsWithInvalidF3442Params)
{
	// GIVEN: The module activated cleanly with the default parameter set.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	// WHEN: A required F3442 parameter is invalid.
	const float negative_value = -10;
	param_set(param_handle(px4::params::DAA_LVL_CRIT_RAD), &negative_value);

	navigator->get_detect_and_avoid()->on_activation();

	// THEN: Activation fails closed.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();

	param_set(param_handle(px4::params::DAA_LVL_HIGH_RAD), &negative_value);

	navigator->get_detect_and_avoid()->on_activation();
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();
}

// WHY: Explicit deactivation should clear the published most-urgent state so downstream consumers never keep stale traffic after DAA stops.
// WHAT: Publish a critical conflict, call on_inactivation(), and verify a cleared detect_and_avoid_most_urgent sample is republished.
TEST_F(DetectAndAvoidTest, OnInactivationRepublishesClearedMostUrgentState)
{
	// GIVEN: Ownship is initialized and DetectAndAvoid has already published a critical most-urgent conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: A critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	// THEN: A kDaaConflictLvlCritical conflict is published.
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlCritical);
	drain_detect_and_avoid_most_urgent_topic();

	// WHEN: The module is explicitly inactivated.
	navigator->get_detect_and_avoid()->on_inactivation();

	wait_until([&]() {
		return _detect_and_avoid_most_urgent_sub.updated();
	});

	// THEN: A no-conflict most-urgent sample is republished.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	expect_empty_most_urgent_status(_detect_and_avoid_most_urgent_sub.get());
}

// WHY: Failed activation should clear the published most-urgent state so downstream consumers never keep stale traffic after DAA stops.
// WHAT: Start from a published critical conflict, force activation failure with invalid F3442 parameters, and verify the replacement most-urgent sample is cleared.
TEST_F(DetectAndAvoidTest, FailedActivationPublishesBenignNoConflictOutput)
{
	// GIVEN: Ownship is initialized and DetectAndAvoid has already published a critical most-urgent conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: A critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	// THEN: A kDaaConflictLvlCritical conflict is published.
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlCritical);
	drain_detect_and_avoid_most_urgent_topic();

	const float invalid_critical_radius = -10.f;
	param_set(param_handle(px4::params::DAA_LVL_CRIT_RAD), &invalid_critical_radius);

	// WHEN: Activation is retried with invalid F3442 parameters.
	navigator->get_detect_and_avoid()->on_activation();

	wait_until([&]() {
		return _detect_and_avoid_most_urgent_sub.updated();
	});

	// THEN: Activation fails and republishes a no-conflict most-urgent sample.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	expect_empty_most_urgent_status(_detect_and_avoid_most_urgent_sub.get());
}

// WHY: The optional default vertical traffic speed directly changes F3442 conflict classification.
// WHAT: Run the same traffic scenario with the fallback velocity enabled and disabled and compare the reported conflict level.
TEST_F(DetectAndAvoidTest, DefaultVelocity)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float traffic_hor_vel = 10000000.f;
	const float traffic_ver_vel = 10000000.f;
	const float alt_diff_out_of_conflict = 1000.f; // Outside all conflict zones if default vertical speed is used.
	conflict_info_s conflict{};

	// WHEN: Default traffic vertical velocity handling is enabled before activation.
	const int daa_en_dflt_vel = 1;
	param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &daa_en_dflt_vel);
	recreate_navigator();

	// GIVEN: The vehicle is armed, airborne, and in mission mode.
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// THEN: Navigator state reflects the published ownship data.
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_EQ(navigator->get_vstatus()->arming_state, vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_EQ(navigator->get_vstatus()->nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	PX4_DEBUG("F_TEST DAA: Default vel enabled.");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(14545057, "DDF0A1", 10)
		.with_altitude_diff(alt_diff_out_of_conflict)
		.with_velocity(traffic_hor_vel, traffic_ver_vel)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The encounter stays out of conflict.
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// WHEN: Default traffic vertical velocity handling is disabled before re-activation.
	PX4_DEBUG("F_TEST DAA: Default vel disabled.");
	const int daa_disable_dflt_vel = 0;
	param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &daa_disable_dflt_vel);
	recreate_navigator();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(14545057, "DDF0A1", 10)
		.with_altitude_diff(alt_diff_out_of_conflict)
		.with_velocity(traffic_hor_vel, traffic_ver_vel)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The same encounter escalates into the augmented NMAC level.
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);
}

// WHY: The nominal flow needs to cover self-detection, conflict prioritization, de-escalation, and final buffer cleanup.
// WHAT: Inject ownship traffic plus multiple external conflicts of different severities and verify the most urgent conflict evolves correctly.
TEST_F(DetectAndAvoidTest, BasicBehavior)
{
	// GIVEN: Ownship is initialized and traffic is expressed relative to it.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	// GIVEN: The vehicle is armed, airborne, and in mission mode.
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// THEN: Navigator state reflects the published ownship data.
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_EQ(navigator->get_vstatus()->arming_state, vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_EQ(navigator->get_vstatus()->nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	// GIVEN: Representative traffic parameters for the staged conflict sequence.
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f; // Not used because default velocity handling is enabled.

	conflict_info_s conflict{};

	// WHEN: No traffic has been processed yet.
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The most urgent conflict starts at none.
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// GIVEN: Self identifiers are configured for both primary and secondary ICAO encodings.
	const uint32_t own_icao = 10436515;
	param_set(param_handle(px4::params::ADSB_ICAO_ID), &own_icao);

	const uint32_t own_icao_2 = 9882899;
	param_set(param_handle(px4::params::ADSB_ICAO_ID_2), &own_icao_2);
	recreate_navigator();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	// NMAC breach:
	const float in_nmac_distance = breach_dist.nmac_hor - 1;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1;
	const float in_nmac_overall_dist = sqrtf(in_nmac_distance * in_nmac_distance +
					   in_nmac_alt_diff * in_nmac_alt_diff);

	// WC breach:
	const float in_wc_distance = breach_dist.nmac_hor + 1;
	const float in_wc_alt_diff =  breach_dist.nmac_vert + 1;
	const float in_wc_overall_dist = sqrtf(in_wc_distance * in_wc_distance +
					       in_wc_alt_diff * in_wc_alt_diff);

	// No more WC breach:
	const float no_more_wc_distance = breach_dist.wc_hor + 1;
	const float no_more_wc_alt_diff = breach_dist.wc_vert + 1;
	const float no_more_wc_overall_dist = sqrtf(no_more_wc_distance * no_more_wc_distance +
					      no_more_wc_alt_diff * no_more_wc_alt_diff);

	// No more Aug NMAC breach:
	const float no_more_aug_nmac_distance = breach_dist.aug_nmac_hor + 1;
	const float no_more_aug_nmac_alt_diff = breach_dist.aug_nmac_vert + 1;
	const float no_more_aug_nmac_overall_dist = sqrtf(no_more_aug_nmac_distance * no_more_aug_nmac_distance +
			no_more_aug_nmac_alt_diff * no_more_aug_nmac_alt_diff);

	// No more Aug WC breach:
	const float no_conflict_distance = breach_dist.aug_wc_hor + 1;
	const float no_conflict_alt_diff = breach_dist.aug_wc_vert + 1;

	// WHEN: Traffic matches the configured ownship identities.
	PX4_DEBUG("---- F_TEST DAA: own icao");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(own_icao, "9F3FA3", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: Self detections do not create conflicts.
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	PX4_DEBUG("---- F_TEST DAA: own secondary icao");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(own_icao_2, "96CD13", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// WHEN: A critical NMAC breach is introduced.
	const uint32_t icao_ddfa0a1 = 14545057;
	PX4_DEBUG("---- F_TEST DAA: DDF0A1 NMAC breach");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(icao_ddfa0a1, "DDF0A1", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});
	conflict = expect_most_urgent_conflict(detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, icao_ddfa0a1);
	EXPECT_EQ(conflict.aircraft_dist, in_nmac_overall_dist);

	// WHEN: A weaker WC breach is added while the critical conflict is still active.
	const uint32_t icao_6e9f7b = 7249787;
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B WC conflict, not most important.");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", in_wc_distance)
		.with_altitude_diff(in_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level,
		  detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL); // DDF0A1 conflict is still the most important
	EXPECT_EQ(conflict.aircraft_dist, in_nmac_overall_dist);
	EXPECT_EQ(conflict.encoded_id.id, icao_ddfa0a1);

	// WHEN: The critical conflict is resolved while the WC breach remains.
	PX4_DEBUG("---- F_TEST DAA: DDF0A1 No more conflict, 6E9F7B WC conflict");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(icao_ddfa0a1, "DDF0A1", no_conflict_distance)
			.with_altitude_diff(no_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});
	conflict = expect_most_urgent_conflict(detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, icao_6e9f7b);
	EXPECT_EQ(conflict.aircraft_dist, in_wc_overall_dist);

	// WHEN: The remaining traffic de-escalates from WC to augmented NMAC.
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", no_more_wc_distance)
		.with_altitude_diff(no_more_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, icao_6e9f7b);
	EXPECT_EQ(conflict.aircraft_dist, no_more_wc_overall_dist);

	// WHEN: The traffic de-escalates again into augmented WC only.
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", no_more_aug_nmac_distance)
		.with_altitude_diff(no_more_aug_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, icao_6e9f7b);
	EXPECT_EQ(conflict.aircraft_dist, no_more_aug_nmac_overall_dist);

	// WHEN: The last remaining traffic exits all conflict zones.
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", no_conflict_distance)
		.with_altitude_diff(no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The conflict buffer fully de-escalates back to none.
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
}

// WHY: F3442 conflict zones are nested, so a disabled inner-zone action must fall back to the next enabled breached zone.
// WHAT: Disable the critical-zone action, keep the high-zone action enabled, trigger a critical conflict, and check that land is commanded.
TEST_F(DetectAndAvoidTest, DisabledHigherPriorityConflictFallsBackToEnabledZone)
{
	// GIVEN: Critical conflicts are configured to take no action while high conflicts still command land.
	const int critical_action = action_param_value(DaaAction::kDisabled);
	const int high_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	param_set(param_handle(px4::params::DAA_LVL_HIGH_ACT), &high_action);
	recreate_navigator();

	// GIVEN: Ownship is initialized in a mission state with representative motion.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: Traffic breaches the critical NMAC volume.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
	check_highest_conflict(kDaaConflictLvlCritical, true);

	// THEN: The module falls back to the next enabled breached action and commands land.
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
}

// WHY: Resetting the module must clear buffered conflicts so stale traffic cannot survive a reactivation.
// WHAT: Create a critical conflict, reactivate the module, then verify a later low-priority conflict is treated as the only remaining traffic.
TEST_F(DetectAndAvoidTest, ResetClearsTrafficBuffer)
{
	// GIVEN: Ownship is initialized with a traffic encounter that becomes critically urgent.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const uint32_t critical_traffic_icao = 14545057;
	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: The critical conflict is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(critical_traffic_icao, "DDF0A1", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.id, critical_traffic_icao);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// WHEN: The module is deactivated and activated again.
	navigator->get_detect_and_avoid()->on_inactivation();
	navigator->get_detect_and_avoid()->on_activation();
	ASSERT_TRUE(navigator->get_detect_and_avoid()->is_activated());
	sync_navigator_topics();

	const uint32_t low_traffic_icao = 7249787;
	const float low_conflict_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_conflict_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	// WHEN: A new low-priority conflict is published after the reset.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(low_traffic_icao, "6E9F7B", low_conflict_distance)
			.with_altitude_diff(low_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The old critical conflict is gone and only the new traffic remains.
	EXPECT_EQ(conflict.encoded_id.id, low_traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlLow);
}

// WHY: The traffic subscription is queued, and dropping earlier reports would let a later low-priority message hide a real hazard.
// WHAT: Publish two reports back to back in one cycle and verify the critical one is still selected as most urgent.
TEST_F(DetectAndAvoidTest, ProcessesAllQueuedTrafficReports)
{
	// GIVEN: Ownship is initialized and two traffic reports will arrive in the same processing cycle.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const uint32_t critical_traffic_icao = 14545057;
	const uint32_t low_traffic_icao = 7249787;
	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;
	const float low_conflict_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_conflict_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	// WHEN: The highest-priority conflict is queued first and a lower-priority one second.
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(critical_traffic_icao, "DDF0A1", in_nmac_distance)
		.with_altitude_diff(in_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(low_traffic_icao, "6E9F7B", low_conflict_distance)
			.with_altitude_diff(low_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The queue is drained completely and the critical conflict still wins.
	EXPECT_EQ(conflict.encoded_id.id, critical_traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// WHY: Zero latitude is a valid coordinate and must not be rejected as an uninitialized value.
// WHAT: Publish traffic on the equator and verify it still produces a conflict.
TEST_F(DetectAndAvoidTest, AcceptsTrafficOnZeroLatitude)
{
	// GIVEN: Ownship is positioned on the equator, where latitude is exactly zero.
	const double lat_uav = 0.0;
	const double lon_uav = 1.0;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const uint32_t traffic_icao = 14545057;
	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	transponder_report_s tr = create_transponder_report(traffic_icao, "DDF0A1", 0.0, lon_uav, alt_uav + 1.f, 0.f, 0.f,
				  flags);

	// WHEN: Traffic is published with that zero-latitude coordinate.
	publish_transponder_report_and_check(tr);

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The valid equatorial coordinate is processed as a real conflict.
	EXPECT_EQ(conflict.encoded_id.id, traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// WHY: Runtime parameter changes must stop and restart DAA cleanly so operators never see stale outputs and re-enabling works without a reboot.
// WHAT: Create a conflict, disable the module and verify the cleared publication, then re-enable it and confirm traffic processing resumes.
TEST_F(DetectAndAvoidTest, RuntimeDisableAndReenableUpdatesState)
{
	// GIVEN: Ownship is initialized with an active critical conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: The critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// WHEN: The feature is disabled at runtime and a parameter update is published.
	const int daa_en = 0;
	param_set(param_handle(px4::params::DAA_EN), &daa_en);
	publish_parameter_update();

	ASSERT_TRUE(navigator->get_detect_and_avoid()->is_activated());
	navigator->check_traffic();
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(status.conflict_level, kDaaConflictLvlNone);
	EXPECT_FALSE(status.has_action);
	EXPECT_GT(status.timestamp, 0u);

	// WHEN: The feature is re-enabled and ownship plus traffic are published again.
	// The parameter update subscription is throttled to 1 s, so we wait past that
	// window to make sure the second update is actually picked up.
	px4_usleep(1100000);

	const int daa_reenable = 1;
	param_set(param_handle(px4::params::DAA_EN), &daa_reenable);
	publish_parameter_update();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	navigator->check_traffic();
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: Disabling clears the published state and re-enabling restores normal processing.
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// WHY: Runtime notification throttling must fail safe even if parameter bounds are bypassed, otherwise a negative interval can silence all on-ground DAA warnings.
// WHAT: Force a negative notification interval before activation, trigger a landed critical conflict that requires action, and verify the warning is still emitted immediately.
TEST_F(DetectAndAvoidTest, NegativeNotificationIntervalIsClampedToZero)
{
	const int negative_notification_interval = -1;
	const int critical_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_NOTIF_STATE), &negative_notification_interval);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(true);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel);
	sync_navigator_topics();
	drain_mavlink_logs();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	wait_for_topic_update(_mavlink_log_sub);
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_TRUE(any_log_contains(logs, "do not takeoff"));
}

// WHY: The first landed warning should not depend on system uptime; otherwise large positive notification intervals can suppress the initial safety message.
// WHAT: Configure a long notification interval, trigger a landed critical conflict that requires action, and verify the first warning is emitted immediately.
TEST_F(DetectAndAvoidTest, FirstLandedWarningIsImmediateWithPositiveInterval)
{
	// GIVEN: A long positive notification interval is configured before activation and ownship is landed and armed.
	const int long_notification_interval = 24 * 60 * 60;
	const int critical_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_NOTIF_STATE), &long_notification_interval);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(true);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel);
	sync_navigator_topics();
	drain_mavlink_logs();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: A landed critical conflict that requires action is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	wait_for_topic_update(_mavlink_log_sub);
	const std::vector<std::string> logs = drain_mavlink_logs();

	// THEN: The first on-ground warning is emitted immediately instead of waiting for the interval to elapse.
	EXPECT_TRUE(any_log_contains(logs, "do not takeoff"));
}

// WHY: Runtime action updates must not re-evaluate the current buffer, but later conflict escalations must use the refreshed action value.
// WHAT: Start with a medium conflict, change the high-level action to hold at runtime, verify the parameter update alone publishes no command, then escalate to high and confirm hold is commanded.
TEST_F(DetectAndAvoidTest, RuntimeF3442ActionUpdateAppliesOnNextEscalationOnly)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float medium_distance = breach_dist.aug_nmac_hor - 1.f;
	const float medium_alt_diff = breach_dist.aug_nmac_vert - 1.f;
	const float high_distance = breach_dist.wc_hor - 1.f;
	const float high_alt_diff = breach_dist.wc_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", medium_distance)
			.with_altitude_diff(medium_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlMedium);
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.get().has_action);
	EXPECT_FALSE(_vehicle_command_sub.update());

	const int high_action_hold = action_param_value(DaaAction::kPositionHoldMode);
	param_set(param_handle(px4::params::DAA_LVL_HIGH_ACT), &high_action_hold);
	publish_parameter_update();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	navigator->check_traffic();
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_FALSE(_vehicle_command_sub.update());

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", high_distance)
			.with_altitude_diff(high_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlHigh);
	EXPECT_TRUE(_detect_and_avoid_most_urgent_sub.get().has_action);
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE);
}

// WHY: DAA must stop exposing buffered conflicts once the uav's position or velocity timestamps go stale.
// WHAT: Create an active conflict, then republish uav's state with old timestamps and verify the conflict buffer and most-urgent topic reset to no-conflict.
TEST_F(DetectAndAvoidTest, ClearsConflictStateWhenOwnshipDataGoesStale)
{
	// GIVEN: Ownship is initialized with an active critical conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: The critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	drain_detect_and_avoid_most_urgent_topic();

	// WHEN: Ownship position and velocity are republished with stale timestamps.
	const hrt_abstime stale_timestamp = hrt_absolute_time() - 2_s;
	publish_global_pos(lat_uav, lon_uav, alt_uav, stale_timestamp);
	publish_local_pos_vel(uav_vel, stale_timestamp);
	wait_until([&]() {
		sync_navigator_topics();
		return navigator->get_global_position()->timestamp == stale_timestamp &&
		       navigator->get_local_position()->timestamp == stale_timestamp;
	});

	wait_until([&]() {
		navigator->check_traffic();
		conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
		return conflict.conflict_level == kDaaConflictLvlNone && _detect_and_avoid_most_urgent_sub.updated();
	});

	// THEN: The buffered conflict and the most-urgent publication are both cleared.
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_FALSE(status.has_action);
	EXPECT_EQ(status.conflict_level, kDaaConflictLvlNone);
}

// WHY: Numeric collisions between ID encodings must not merge unrelated aircraft into one buffer slot.
// WHAT: Use an ICAO and a callsign that encode to the same integer and verify their conflicts remain independent across updates and removals.
TEST_F(DetectAndAvoidTest, DifferentEncodingsDoNotShareBufferSlot)
{
	// GIVEN: Ownship is initialized and the DAA publications are drained.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();
	drain_detect_and_avoid_most_urgent_topic();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const char colliding_callsign[] = "ABC";
	const uint64_t colliding_unique_id = DaaEncodedId::callsign_to_uint64(colliding_callsign);
	ASSERT_GT(colliding_unique_id, 0u);
	ASSERT_LE(colliding_unique_id, static_cast<uint64_t>(UINT32_MAX));

	const uint32_t colliding_icao = static_cast<uint32_t>(colliding_unique_id);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;
	const float low_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_alt_diff = breach_dist.aug_nmac_vert + 1.f;
	const float no_conflict_distance = breach_dist.aug_wc_hor + 1.f;
	const float no_conflict_alt_diff = breach_dist.aug_wc_vert + 1.f;

	// WHEN: ICAO traffic occupies the colliding numeric slot first.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(colliding_icao, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	ASSERT_EQ(conflict.encoded_id.id, colliding_icao);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
	ASSERT_TRUE(_detect_and_avoid_sub.update());
	const detect_and_avoid_s &first_report = _detect_and_avoid_sub.get();
	EXPECT_EQ(first_report.unique_id, colliding_icao);
	EXPECT_EQ(first_report.unique_id_encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &first_status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(first_status.unique_id, colliding_icao);
	EXPECT_EQ(first_status.unique_id_encoding, detect_and_avoid_most_urgent_s::UNIQUE_ID_ENCODING_ICAO);

	// WHEN: Callsign-based traffic that hashes to the same numeric value is added.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(0, colliding_callsign, low_distance)
			.with_altitude_diff(low_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	ASSERT_EQ(conflict.encoded_id.id, colliding_icao);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// WHEN: The ICAO conflict resolves while the callsign-based traffic remains active.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(colliding_icao, "DDF0A1", no_conflict_distance)
			.with_altitude_diff(no_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The callsign traffic remains in its own slot and becomes the most urgent entry.
	EXPECT_EQ(conflict.encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
	EXPECT_EQ(conflict.encoded_id.id, colliding_unique_id);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlLow);
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &callsign_status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(callsign_status.unique_id, colliding_unique_id);
	EXPECT_EQ(callsign_status.unique_id_encoding, detect_and_avoid_most_urgent_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
}

// WHY: A new conflict that is immediately the most urgent one should notify the operator only once.
// WHAT: Inject a single warning-level conflict and verify it emits one combined "new and main" message instead of separate new and main logs.
TEST_F(DetectAndAvoidTest, NewMostUrgentConflictUsesCombinedNotification)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_mavlink_logs();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New and Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 0u);
	EXPECT_FALSE(any_log_contains(logs, "DAA New DDF0A1"));
}

// WHY: If a new conflict becomes the most urgent one without changing the highest conflict level, the operator still needs a combined main notification.
// WHAT: Add a closer conflict at the same level as the current main conflict and verify it emits the combined "new and main" message immediately.
TEST_F(DetectAndAvoidTest, NewMostUrgentConflictSameLevelStillNotifiesAsMain)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float far_critical_distance = breach_dist.nmac_hor - 1.f;
	const float close_critical_distance = breach_dist.nmac_hor - 10.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", far_critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	drain_mavlink_logs();

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(7249787, "6E9F7B", close_critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.id, 7249787u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New and Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 0u);
	EXPECT_FALSE(any_log_contains(logs, "DAA New 6E9F7B"));
	EXPECT_TRUE(any_log_contains(logs, "DAA New and Main: 6E9F7B lvl 4."));
}

// WHY: Escalating the primary conflict should not also emit a secondary-traffic notification for the same update.
// WHAT: Raise the most urgent conflict while another traffic item remains active and verify only the main-status log is emitted.
TEST_F(DetectAndAvoidTest, MostUrgentEscalationDoesNotSendSecondaryNotification)
{
	// GIVEN: One primary medium conflict and one secondary low conflict are active.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float medium_distance = breach_dist.wc_hor + 1.f;
	const float medium_alt_diff = breach_dist.wc_vert + 1.f;
	const float high_distance = breach_dist.nmac_hor + 1.f;
	const float high_alt_diff = breach_dist.nmac_vert + 1.f;
	const float low_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", medium_distance)
			.with_altitude_diff(medium_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(7249787, "6E9F7B", low_distance)
			.with_altitude_diff(low_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	drain_mavlink_logs();

	// WHEN: The most urgent conflict escalates but stays the most urgent entry.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", high_distance)
			.with_altitude_diff(high_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.id, 14545057u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlHigh);

	// THEN: Only the most-urgent notification is emitted for that update.
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 0u);
}

// WHY: De-escalating the primary conflict should not also emit a secondary-traffic notification for the same update.
// WHAT: Lower the most urgent conflict while another traffic item remains active and verify only the main-status log is emitted.
TEST_F(DetectAndAvoidTest, MostUrgentDeescalationDoesNotSendSecondaryNotification)
{
	// GIVEN: One primary critical conflict and one secondary low conflict are active.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;
	const float high_distance = breach_dist.nmac_hor + 1.f;
	const float high_alt_diff = breach_dist.nmac_vert + 1.f;
	const float low_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(7249787, "6E9F7B", low_distance)
			.with_altitude_diff(low_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	drain_mavlink_logs();

	// WHEN: The most urgent conflict de-escalates but stays the most urgent entry.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", high_distance)
			.with_altitude_diff(high_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.id, 14545057u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlHigh);

	// THEN: Only the most-urgent notification is emitted for that update.
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 0u);
}

// WHY: Secondary traffic still needs explicit operator feedback when its conflict resolves.
// WHAT: Resolve a non-primary conflict and verify the secondary solved notification is emitted while the primary conflict remains active.
TEST_F(DetectAndAvoidTest, SecondaryConflictResolutionSendsSecondaryNotification)
{
	// GIVEN: One primary critical conflict and one secondary high conflict are active.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;
	const float high_distance = breach_dist.nmac_hor + 1.f;
	const float high_alt_diff = breach_dist.nmac_vert + 1.f;
	const float no_conflict_distance = breach_dist.aug_wc_hor + 1.f;
	const float no_conflict_alt_diff = breach_dist.aug_wc_vert + 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(7249787, "6E9F7B", high_distance)
			.with_altitude_diff(high_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	drain_mavlink_logs();

	// WHEN: The secondary conflict resolves while the primary conflict remains active.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(7249787, "6E9F7B", no_conflict_distance)
			.with_altitude_diff(no_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.encoded_id.id, 14545057u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// THEN: The operator receives a single secondary solved notification.
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 1u);
	EXPECT_TRUE(any_log_contains(logs, "solved"));
}

// WHY: Buffer-full conditions can repeat quickly, so ignored-traffic warnings must be rate-limited to stay actionable.
// WHAT: Fill the buffer, inject the same ignored traffic twice in quick succession, and verify only the first attempt logs an ignore warning.
TEST_F(DetectAndAvoidTest, IgnoredTrafficNotificationIsThrottled)
{
	// GIVEN: The conflict buffer is filled with active traffic.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	const float buffer_distance = breach_dist.aug_nmac_hor + 1.f;
	const float buffer_alt_diff = breach_dist.aug_nmac_vert + 1.f;
	const float ignored_distance = breach_dist.aug_wc_hor - 1.f;
	const float ignored_alt_diff = breach_dist.aug_wc_vert - 1.f;
	const uint32_t traffic_ids[kDaaMaxTraffic] {14545057, 7249787, 6593425, 3318901, 5207278};

	for (uint8_t i = 0; i < kDaaMaxTraffic; ++i) {
		publish_traffic_and_check([&]() {
			navigator->get_detect_and_avoid()->fake_traffic(
				fake_traffic_report(traffic_ids[i], "DDF0A1", buffer_distance)
				.with_direction(static_cast<float>(i))
				.with_altitude_diff(buffer_alt_diff)
				.with_velocity(hor_velocity, ver_velocity)
				.from_ownship(lat_uav, lon_uav, alt_uav));
		});
	}

	drain_mavlink_logs();

	// WHEN: The same ignored traffic is injected twice back to back.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(0x123456, "123456", ignored_distance)
			.with_altitude_diff(ignored_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	const std::vector<std::string> first_logs = drain_mavlink_logs();
	EXPECT_TRUE(any_log_contains(first_logs, "ignored"));

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(0x123456, "123456", ignored_distance)
			.with_altitude_diff(ignored_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	const std::vector<std::string> second_logs = drain_mavlink_logs();

	// THEN: Only the first ignored insertion emits a warning.
	EXPECT_FALSE(any_log_contains(second_logs, "ignored"));
}

// WHY: The bounded conflict buffer must keep the most important traffic and evict less urgent entries deterministically.
// WHAT: Fill the buffer with ordered priorities, insert better and worse conflicts, then resolve them step by step while checking the chosen top conflict.
TEST_F(DetectAndAvoidTest, BufferFull)
{
	/*
	Test behavior when buffer is full ( max 5 traffic).
	Define traffic with priorities (1,2,...,7).
	The lower the priority the more important the traffic (the closest)

	1. Fill buffer with [6, 5, 4, 3, 2]
	2. New traffic:
		Add traff 7:	--> State remains [6, 5, 4, 3, 2]
		Add traff 0:	--> New state [5, 4, 3, 2, 0]
		Add traff 1:	--> New state [4, 3, 2, 1, 0]
		Resolve traff 0: --> New state: [4, 3, 2, 1]
		Add traff 7:	--> New state: [7, 4, 3, 2, 1]
		Resolve traff 3: --> New state: [7, 4, 2, 1]
		Resolve traff 2: --> New state: [7, 4, 1]
		Resolve traff 1: --> New state: [7, 4]
		Resolve traff 4: --> New state: [7]
		Resolve traff 7: --> New state: []
	*/

	// GIVEN: Ownship is initialized and traffic will be expressed relative to it.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	// GIVEN: The vehicle is armed, airborne, and in mission mode.
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// THEN: Navigator state reflects the published ownship data.
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_EQ(navigator->get_vstatus()->arming_state, vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_EQ(navigator->get_vstatus()->nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	// GIVEN: Representative traffic parameters for a full buffer-priority exercise.
	const float hor_velocity = 20.f;
	const float ver_velocity = 10000000.f; // Not used because DAA_EN_DFLT_VEL is enabled.

	conflict_info_s conflict{};

	// WHEN: No traffic has been processed yet.
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The most urgent conflict starts at none.
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	const breach_distances_s breach_dist = get_DFLT_breach_distances(uav_vel, hor_velocity);

	// Priority 0, NMAC breach:
	const uint32_t prio_0_icao_96CD13 = 9882899;
	const uint8_t prio_0_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL;
	const float prio_0_in_nmac_distance = breach_dist.nmac_hor - 5;
	const float prio_0_in_nmac_alt_diff = breach_dist.nmac_vert - 5;
	const float prio_0_in_nmac_overall_dist = sqrtf(prio_0_in_nmac_distance * prio_0_in_nmac_distance +
			prio_0_in_nmac_alt_diff * prio_0_in_nmac_alt_diff);

	// Priority 1, NMAC breach:
	const uint32_t prio_1_icao_A72BC8 = 10955720;
	const uint8_t prio_1_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL;
	const float prio_1_in_nmac_distance = breach_dist.nmac_hor - 1;
	const float prio_1_in_nmac_alt_diff = breach_dist.nmac_vert - 1;
	const float prio_1_in_nmac_overall_dist = sqrtf(prio_1_in_nmac_distance * prio_1_in_nmac_distance +
			prio_1_in_nmac_alt_diff * prio_1_in_nmac_alt_diff);

	// Priority 2, WC breach at NMAC limit
	const uint32_t prio_2_icao_36A887 = 3582087;
	const uint8_t prio_2_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH;
	const float prio_2_in_wc_distance = breach_dist.nmac_hor + 1;
	const float prio_2_in_wc_alt_diff =  breach_dist.nmac_vert + 1;
	const float prio_2_in_wc_overall_dist = sqrtf(prio_2_in_wc_distance * prio_2_in_wc_distance +
						prio_2_in_wc_alt_diff * prio_2_in_wc_alt_diff);

	// Priority 3, WC breach at WC limit
	const uint32_t prio_3_icao_32A475 = 3318901;
	const uint8_t prio_3_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH;
	const float prio_3_in_wc_distance = breach_dist.wc_hor - 1;
	const float prio_3_in_wc_alt_diff = breach_dist.wc_vert - 1;
	const float prio_3_in_wc_overall_dist = sqrtf(prio_3_in_wc_distance * prio_3_in_wc_distance +
						prio_3_in_wc_alt_diff * prio_3_in_wc_alt_diff);

	// Priority 4, Aug NMAC breach at WC limit
	const uint32_t prio_4_icao_BAF2B4 = 12251828;
	const uint8_t prio_4_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM;
	const float prio_4_in_aug_nmac_distance = breach_dist.wc_hor + 1;
	const float prio_4_in_aug_nmac_alt_diff = breach_dist.wc_vert + 1;
	const float prio_4_in_aug_nmac_overall_dist = sqrtf(prio_4_in_aug_nmac_distance * prio_4_in_aug_nmac_distance +
			prio_4_in_aug_nmac_alt_diff * prio_4_in_aug_nmac_alt_diff);

	// Priority 5, Aug NMAC breach at Aug WC limit
	const uint32_t prio_5_icao_4F74EE = 5207278;
	const uint8_t prio_5_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM;
	const float prio_5_in_aug_nmac_distance = breach_dist.aug_nmac_hor - 1;
	const float prio_5_in_aug_nmac_alt_diff = breach_dist.aug_nmac_vert - 1;
	const float prio_5_in_aug_nmac_overall_dist = sqrtf(prio_5_in_aug_nmac_distance * prio_5_in_aug_nmac_distance +
			prio_5_in_aug_nmac_alt_diff * prio_5_in_aug_nmac_alt_diff);

	// Priority 6, Aug WC breach at Aug NMAC limit
	const uint32_t prio_6_icao_2F1BF1 = 3087345;
	const uint8_t prio_6_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	const float prio_6_in_aug_wc_distance = breach_dist.aug_nmac_hor + 1;
	const float prio_6_in_aug_wc_alt_diff = breach_dist.aug_nmac_vert + 1;
	const float prio_6_in_aug_wc_overall_dist = sqrtf(prio_6_in_aug_wc_distance * prio_6_in_aug_wc_distance +
			prio_6_in_aug_wc_alt_diff * prio_6_in_aug_wc_alt_diff);

	// Priority 7, Aug WC breach at No conflict limit
	const uint32_t prio_7_icao_6F0C1B = 7277595;
	const uint8_t prio_7_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	const float prio_7_in_aug_wc_distance = breach_dist.aug_wc_hor - 1;
	const float prio_7_in_aug_wc_alt_diff = breach_dist.aug_wc_vert - 1;
	const float prio_7_in_aug_wc_overall_dist = sqrtf(prio_7_in_aug_wc_distance * prio_7_in_aug_wc_distance +
			prio_7_in_aug_wc_alt_diff * prio_7_in_aug_wc_alt_diff);

	// Priority 8, No conflict
	// const uint8_t prio_8_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
	const float prio_8_no_conflict_distance = breach_dist.aug_wc_hor + 1;
	const float prio_8_no_conflict_alt_diff = breach_dist.aug_wc_vert + 1;

	// WHEN: The buffer is filled with priorities 6, 5, 4, 3, 2.
	PX4_DEBUG("---- F_TEST DAA: Prio 6, 2F1BF1 Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_6_icao_2F1BF1, "2F1BF1", prio_6_in_aug_wc_distance)
		.with_altitude_diff(prio_6_in_aug_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_6_conflict_level, prio_6_icao_2F1BF1);
	EXPECT_EQ(conflict.aircraft_dist, prio_6_in_aug_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 5, 4F74EE Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_5_icao_4F74EE, "4F74EE", prio_5_in_aug_nmac_distance)
		.with_altitude_diff(prio_5_in_aug_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_5_conflict_level, prio_5_icao_4F74EE);
	EXPECT_EQ(conflict.aircraft_dist, prio_5_in_aug_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 4, BAF2B4 Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_4_icao_BAF2B4, "BAF2B4", prio_4_in_aug_nmac_distance)
		.with_altitude_diff(prio_4_in_aug_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_4_conflict_level, prio_4_icao_BAF2B4);
	EXPECT_EQ(conflict.aircraft_dist, prio_4_in_aug_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 3, 32A475 WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_3_icao_32A475, "32A475", prio_3_in_wc_distance)
		.with_altitude_diff(prio_3_in_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_3_conflict_level, prio_3_icao_32A475);
	EXPECT_EQ(conflict.aircraft_dist, prio_3_in_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 2, 36A887 WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_2_icao_36A887, "36A887", prio_2_in_wc_distance)
		.with_altitude_diff(prio_2_in_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_2_conflict_level, prio_2_icao_36A887);
	EXPECT_EQ(conflict.aircraft_dist, prio_2_in_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// THEN: Priority 2 is the most urgent conflict before overflow cases are exercised.

	// WHEN: A lower-priority conflict is inserted into the full buffer.
	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_7_icao_6F0C1B, "6F0C1B", prio_7_in_aug_wc_distance)
		.with_altitude_diff(prio_7_in_aug_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_2_conflict_level, prio_2_icao_36A887);

	// THEN: The existing buffer contents are preserved because the new traffic is less urgent.
	// Buffer [6, 5, 4, 3, 2]
	EXPECT_EQ(conflict.aircraft_dist, prio_2_in_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: A new highest-priority conflict arrives.
	PX4_DEBUG("---- F_TEST DAA: Prio 0, 96CD13 NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_0_icao_96CD13, "96CD13", prio_0_in_nmac_distance)
		.with_altitude_diff(prio_0_in_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_0_conflict_level, prio_0_icao_96CD13);

	// THEN: The weakest buffered conflict is removed and priority 0 becomes most urgent.
	// Buffer: [5, 4, 3, 2, 0]
	EXPECT_EQ(conflict.aircraft_dist, prio_0_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: A second high-priority conflict is inserted.
	PX4_DEBUG("---- F_TEST DAA: Prio 1, A72BC8 NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_1_icao_A72BC8, "A72BC8", prio_1_in_nmac_distance)
		.with_altitude_diff(prio_1_in_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_0_conflict_level, prio_0_icao_96CD13);

	// THEN: Priority 0 remains most urgent while the buffer reorders around it.
	// Buffer: [4, 3, 2, 1, 0]
	EXPECT_EQ(conflict.aircraft_dist, prio_0_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: The top-priority conflict resolves.
	PX4_DEBUG("---- F_TEST DAA: Prio 0, 96CD13 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_0_icao_96CD13, "96CD13", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_1_conflict_level, prio_1_icao_A72BC8);

	// THEN: Priority 1 becomes the new most urgent conflict.
	// Buffer: [4, 3, 2, 1]
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: The buffer has room again and a low-priority conflict is added back.
	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_7_icao_6F0C1B, "6F0C1B", prio_7_in_aug_wc_distance)
		.with_altitude_diff(prio_7_in_aug_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_1_conflict_level, prio_1_icao_A72BC8);

	// THEN: The added low-priority traffic does not affect the top conflict.
	// Buffer: [7, 4, 3, 2, 1]
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: Mid-priority conflicts resolve one by one.
	PX4_DEBUG("---- F_TEST DAA: Prio 3, 32A475 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_3_icao_32A475, "32A475", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_1_conflict_level, prio_1_icao_A72BC8);
	// Buffer: [7, 4, 2, 1]
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 2, 36A887 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_2_icao_36A887, "36A887", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_1_conflict_level, prio_1_icao_A72BC8);
	// Buffer: [7, 4, 1]
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 1, A72BC8 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_1_icao_A72BC8, "A72BC8", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_4_conflict_level, prio_4_icao_BAF2B4);
	// Buffer: [7, 4]
	EXPECT_EQ(conflict.aircraft_dist, prio_4_in_aug_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 4, BAF2B4 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_4_icao_BAF2B4, "BAF2B4", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_7_conflict_level, prio_7_icao_6F0C1B);
	// Buffer: [7]
	EXPECT_EQ(conflict.aircraft_dist, prio_7_in_aug_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_7_icao_6F0C1B, "6F0C1B", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// THEN: The buffer returns to an empty no-conflict state.
	// Buffer: []
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));
}
