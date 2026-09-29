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

// Activation fails closed when an F3442 alert-volume parameter is invalid.
TEST_F(DetectAndAvoidTest, ActivationFailsWithInvalidF3442Params)
{
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	const float negative_value = -10;
	param_set(param_handle(px4::params::DAA_LVL_CRIT_RAD), &negative_value);

	navigator->get_detect_and_avoid()->on_activation();

	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();

	param_set(param_handle(px4::params::DAA_LVL_HIGH_RAD), &negative_value);

	navigator->get_detect_and_avoid()->on_activation();
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();
}

// on_inactivation() republishes a cleared most-urgent sample so consumers drop stale traffic.
TEST_F(DetectAndAvoidTest, OnInactivationRepublishesClearedMostUrgentState)
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

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlCritical);
	drain_detect_and_avoid_most_urgent_topic();

	navigator->get_detect_and_avoid()->on_inactivation();

	wait_until([&]() {
		return _detect_and_avoid_most_urgent_sub.updated();
	});

	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	expect_empty_most_urgent_status(_detect_and_avoid_most_urgent_sub.get());
}

// A failed re-activation (invalid params) also republishes a cleared most-urgent sample.
TEST_F(DetectAndAvoidTest, FailedActivationPublishesBenignNoConflictOutput)
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

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlCritical);
	drain_detect_and_avoid_most_urgent_topic();

	const float invalid_critical_radius = -10.f;
	param_set(param_handle(px4::params::DAA_LVL_CRIT_RAD), &invalid_critical_radius);

	navigator->get_detect_and_avoid()->on_activation();

	wait_until([&]() {
		return _detect_and_avoid_most_urgent_sub.updated();
	});

	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	expect_empty_most_urgent_status(_detect_and_avoid_most_urgent_sub.get());
}

// The optional default vertical traffic speed changes F3442 classification: the same encounter is
// clear with it enabled and a conflict with it disabled.
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

	// default vertical velocity handling enabled
	const int daa_en_dflt_vel = 1;
	param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &daa_en_dflt_vel);
	recreate_navigator();

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// navigator picked up the ownship data
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

	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// same encounter with default vertical velocity disabled
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

	// now escalates into the augmented NMAC level
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);
}

// F3442 retains its static alert volumes by replacing unavailable ownship velocity groups with zero.
TEST_F(DetectAndAvoidTest, InvalidOwnshipVelocityUsesStaticF3442Bounds)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const matrix::Vector3f invalid_velocity{nan, nan, nan};

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(false);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(invalid_velocity, hrt_absolute_time(), 0.f, false, false);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  alt_uav, 0.f, 0.f, flags);
	publish_transponder_report_and_check(tr);

	ASSERT_TRUE(_detect_and_avoid_sub.update());
	EXPECT_EQ(_detect_and_avoid_sub.get().conflict_level, kDaaConflictLvlCritical);
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlCritical);
}

// Nominal flow: self-detection, conflict prioritization, de-escalation, and final buffer cleanup.
TEST_F(DetectAndAvoidTest, BasicBehavior)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// navigator picked up the ownship data
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_EQ(navigator->get_vstatus()->arming_state, vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_EQ(navigator->get_vstatus()->nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f; // Not used because default velocity handling is enabled.

	conflict_info_s conflict{};

	// no traffic yet -> none
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// configure self identifiers for primary and secondary ICAO
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

	// traffic matching the configured ownship identities (primary then secondary ICAO)
	PX4_DEBUG("---- F_TEST DAA: own icao");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(own_icao, "9F3FA3", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// self detections create no conflict
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

	// critical NMAC breach
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

	// weaker WC breach added; the critical conflict stays most urgent
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

	// critical resolved, WC remains -> WC becomes most urgent
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

	// de-escalate WC -> augmented NMAC
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", no_more_wc_distance)
		.with_altitude_diff(no_more_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, icao_6e9f7b);
	EXPECT_EQ(conflict.aircraft_dist, no_more_wc_overall_dist);

	// de-escalate -> augmented WC only
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", no_more_aug_nmac_distance)
		.with_altitude_diff(no_more_aug_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, icao_6e9f7b);
	EXPECT_EQ(conflict.aircraft_dist, no_more_aug_nmac_overall_dist);

	// last traffic exits all zones
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(icao_6e9f7b, "6E9F7B", no_conflict_distance)
		.with_altitude_diff(no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// buffer fully de-escalated to none
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
}

// F3442 zones are nested: a disabled inner-zone action falls back to the next enabled breached zone
// (critical disabled, high=land -> a critical breach commands land).
TEST_F(DetectAndAvoidTest, DisabledHigherPriorityConflictFallsBackToEnabledZone)
{
	const int critical_action = action_param_value(DaaAction::kDisabled);
	const int high_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	param_set(param_handle(px4::params::DAA_LVL_HIGH_ACT), &high_action);
	recreate_navigator();

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

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", in_nmac_distance)
			.with_altitude_diff(in_nmac_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
	check_highest_conflict(kDaaConflictLvlCritical);

	// falls back to the high zone's action: land
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
}

// reset()/reactivation clears buffered conflicts, so a later low-priority report is the only traffic.
TEST_F(DetectAndAvoidTest, ResetClearsTrafficBuffer)
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

	const uint32_t critical_traffic_icao = 14545057;
	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;

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

	// deactivate then activate again
	navigator->get_detect_and_avoid()->on_inactivation();
	navigator->get_detect_and_avoid()->on_activation();
	ASSERT_TRUE(navigator->get_detect_and_avoid()->is_activated());
	sync_navigator_topics();

	const uint32_t low_traffic_icao = 7249787;
	const float low_conflict_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_conflict_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	// new low-priority conflict after the reset
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(low_traffic_icao, "6E9F7B", low_conflict_distance)
			.with_altitude_diff(low_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// old critical conflict gone, only the new traffic remains
	EXPECT_EQ(conflict.encoded_id.id, low_traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlLow);
}

// The traffic queue is fully drained: a critical report queued before a low one still wins.
TEST_F(DetectAndAvoidTest, ProcessesAllQueuedTrafficReports)
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

	const uint32_t critical_traffic_icao = 14545057;
	const uint32_t low_traffic_icao = 7249787;
	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;
	const float low_conflict_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_conflict_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	// critical queued first, lower-priority second
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

	EXPECT_EQ(conflict.encoded_id.id, critical_traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// Zero latitude is a valid coordinate (equator), not an uninitialized value, and still conflicts.
TEST_F(DetectAndAvoidTest, AcceptsTrafficOnZeroLatitude)
{
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

	publish_transponder_report_and_check(tr);

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	EXPECT_EQ(conflict.encoded_id.id, traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// Disabling DAA at runtime clears its output; re-enabling resumes processing without a reboot.
TEST_F(DetectAndAvoidTest, RuntimeDisableAndReenableUpdatesState)
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

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(14545057, "DDF0A1", critical_distance)
			.with_altitude_diff(critical_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// disable at runtime via param update
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

	// re-enable; the parameter_update sub is throttled to 1 s, so wait past that window
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

	// processing restored after re-enable
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// A negative DAA_NOTIF_STATE is clamped to 0, so the on-ground warning still fires immediately.
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

// The first landed warning fires immediately, even with a large notification interval configured.
TEST_F(DetectAndAvoidTest, FirstLandedWarningIsImmediateWithPositiveInterval)
{
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

// A runtime action change doesn't re-evaluate the current buffer, but the next escalation uses it.
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

// Stale ownship position/velocity timestamps clear the buffer and the most-urgent topic.
TEST_F(DetectAndAvoidTest, ClearsConflictStateWhenOwnshipDataGoesStale)
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
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

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

	// republish ownship pose/velocity with stale timestamps
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

	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_FALSE(status.has_action);
	EXPECT_EQ(status.conflict_level, kDaaConflictLvlNone);
}

// Same numeric id under different encodings (ICAO vs callsign) stays in separate buffer slots.
TEST_F(DetectAndAvoidTest, DifferentEncodingsDoNotShareBufferSlot)
{
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

	// ICAO traffic takes the colliding numeric slot first
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

	// callsign traffic hashing to the same numeric value
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

	// resolve the ICAO conflict; the callsign traffic stays in its own slot and becomes most urgent
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(
			fake_traffic_report(colliding_icao, "DDF0A1", no_conflict_distance)
			.with_altitude_diff(no_conflict_alt_diff)
			.with_velocity(hor_velocity, ver_velocity)
			.from_ownship(lat_uav, lon_uav, alt_uav));
	});

	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	EXPECT_EQ(conflict.encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
	EXPECT_EQ(conflict.encoded_id.id, colliding_unique_id);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlLow);
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &callsign_status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(callsign_status.unique_id, colliding_unique_id);
	EXPECT_EQ(callsign_status.unique_id_encoding, detect_and_avoid_most_urgent_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
}

// A new conflict that is immediately the most urgent emits one combined "new and main" message.
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

// A closer conflict at the same level as the current main still emits the combined "new and main".
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

// Escalating the primary conflict emits only the main-status log, not a secondary one.
TEST_F(DetectAndAvoidTest, MostUrgentEscalationDoesNotSendSecondaryNotification)
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

	// most urgent escalates, staying the most urgent entry
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

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 0u);
}

// De-escalating the primary conflict emits only the main-status log, not a secondary one.
TEST_F(DetectAndAvoidTest, MostUrgentDeescalationDoesNotSendSecondaryNotification)
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

	// most urgent de-escalates, staying the most urgent entry
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

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 0u);
}

// Resolving a secondary conflict emits a secondary "solved" notification while the primary remains.
TEST_F(DetectAndAvoidTest, SecondaryConflictResolutionSendsSecondaryNotification)
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

	// resolve the secondary while the primary remains active
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

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 1u);
	EXPECT_TRUE(any_log_contains(logs, "solved"));
}

// Ignored-traffic warnings are rate-limited: the same buffer-full report twice in a row logs once.
TEST_F(DetectAndAvoidTest, IgnoredTrafficNotificationIsThrottled)
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

	const float buffer_distance = breach_dist.aug_nmac_hor + 1.f;
	const float buffer_alt_diff = breach_dist.aug_nmac_vert + 1.f;
	const float ignored_distance = breach_dist.aug_wc_hor - 1.f;
	const float ignored_alt_diff = breach_dist.aug_wc_vert - 1.f;
	const uint32_t traffic_ids[kDaaMaxTraffic] {14545057, 7249787, 6593425, 3318901, 5207278};

	// fill the buffer with active traffic
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

	// same ignored traffic injected twice back to back
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

	// second insertion is throttled
	EXPECT_FALSE(any_log_contains(second_logs, "ignored"));
}

// The bounded buffer keeps the most important traffic and evicts the least urgent deterministically.
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

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// navigator picked up the ownship data
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_EQ(navigator->get_vstatus()->arming_state, vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_EQ(navigator->get_vstatus()->nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	const float hor_velocity = 20.f;
	const float ver_velocity = 10000000.f; // Not used because DAA_EN_DFLT_VEL is enabled.

	conflict_info_s conflict{};

	// no traffic yet -> none
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

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

	// fill the buffer: priorities 6, 5, 4, 3, 2
	PX4_DEBUG("---- F_TEST DAA: Prio 6, 2F1BF1 Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_6_icao_2F1BF1, "2F1BF1", prio_6_in_aug_wc_distance)
		.with_altitude_diff(prio_6_in_aug_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_6_conflict_level, prio_6_icao_2F1BF1);
	EXPECT_EQ(conflict.aircraft_dist, prio_6_in_aug_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	PX4_DEBUG("---- F_TEST DAA: Prio 5, 4F74EE Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_5_icao_4F74EE, "4F74EE", prio_5_in_aug_nmac_distance)
		.with_altitude_diff(prio_5_in_aug_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_5_conflict_level, prio_5_icao_4F74EE);
	EXPECT_EQ(conflict.aircraft_dist, prio_5_in_aug_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	PX4_DEBUG("---- F_TEST DAA: Prio 4, BAF2B4 Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_4_icao_BAF2B4, "BAF2B4", prio_4_in_aug_nmac_distance)
		.with_altitude_diff(prio_4_in_aug_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_4_conflict_level, prio_4_icao_BAF2B4);
	EXPECT_EQ(conflict.aircraft_dist, prio_4_in_aug_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	PX4_DEBUG("---- F_TEST DAA: Prio 3, 32A475 WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_3_icao_32A475, "32A475", prio_3_in_wc_distance)
		.with_altitude_diff(prio_3_in_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_3_conflict_level, prio_3_icao_32A475);
	EXPECT_EQ(conflict.aircraft_dist, prio_3_in_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	PX4_DEBUG("---- F_TEST DAA: Prio 2, 36A887 WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_2_icao_36A887, "36A887", prio_2_in_wc_distance)
		.with_altitude_diff(prio_2_in_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_2_conflict_level, prio_2_icao_36A887);
	EXPECT_EQ(conflict.aircraft_dist, prio_2_in_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	// lower-priority into a full buffer: rejected, contents preserved
	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_7_icao_6F0C1B, "6F0C1B", prio_7_in_aug_wc_distance)
		.with_altitude_diff(prio_7_in_aug_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_2_conflict_level, prio_2_icao_36A887);

	// Buffer [6, 5, 4, 3, 2]
	EXPECT_EQ(conflict.aircraft_dist, prio_2_in_wc_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	// new highest-priority conflict evicts the weakest
	PX4_DEBUG("---- F_TEST DAA: Prio 0, 96CD13 NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_0_icao_96CD13, "96CD13", prio_0_in_nmac_distance)
		.with_altitude_diff(prio_0_in_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_0_conflict_level, prio_0_icao_96CD13);

	// Buffer: [5, 4, 3, 2, 0]
	EXPECT_EQ(conflict.aircraft_dist, prio_0_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	// second high-priority conflict; prio 0 stays most urgent
	PX4_DEBUG("---- F_TEST DAA: Prio 1, A72BC8 NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_1_icao_A72BC8, "A72BC8", prio_1_in_nmac_distance)
		.with_altitude_diff(prio_1_in_nmac_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_0_conflict_level, prio_0_icao_96CD13);

	// Buffer: [4, 3, 2, 1, 0]
	EXPECT_EQ(conflict.aircraft_dist, prio_0_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	// resolve the top-priority conflict -> prio 1 becomes most urgent
	PX4_DEBUG("---- F_TEST DAA: Prio 0, 96CD13 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_0_icao_96CD13, "96CD13", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_1_conflict_level, prio_1_icao_A72BC8);

	// Buffer: [4, 3, 2, 1]
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	// room again: add back a low-priority conflict (top unaffected)
	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_7_icao_6F0C1B, "6F0C1B", prio_7_in_aug_wc_distance)
		.with_altitude_diff(prio_7_in_aug_wc_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = expect_most_urgent_conflict(prio_1_conflict_level, prio_1_icao_A72BC8);

	// Buffer: [7, 4, 3, 2, 1]
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);

	check_highest_conflict(conflict.conflict_level);

	// resolve mid-priority conflicts one by one
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

	check_highest_conflict(conflict.conflict_level);

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

	check_highest_conflict(conflict.conflict_level);

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

	check_highest_conflict(conflict.conflict_level);

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

	check_highest_conflict(conflict.conflict_level);

	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B resolve");
	navigator->get_detect_and_avoid()->fake_traffic(
		fake_traffic_report(prio_7_icao_6F0C1B, "6F0C1B", prio_8_no_conflict_distance)
		.with_altitude_diff(prio_8_no_conflict_alt_diff)
		.with_velocity(hor_velocity, ver_velocity)
		.from_ownship(lat_uav, lon_uav, alt_uav));
	navigator->check_traffic();
	conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	// Buffer: []
	EXPECT_EQ(conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	check_highest_conflict(conflict.conflict_level);
}
