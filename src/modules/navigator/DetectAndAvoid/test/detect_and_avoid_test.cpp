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
 * @file detect_and_avoid_test.cpp
 * @brief DetectAndAvoid functional tests common to all DAA standards.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "detect_and_avoid_test_common.h"

// Activation succeeds with default params and is refused when DAA_EN is cleared.
TEST_F(DetectAndAvoidTest, OnActivation)
{
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	navigator->get_detect_and_avoid()->on_inactivation();
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());

	// disable via param before activation
	const int daa_en = 0;
	param_t param_daa_en = param_handle(px4::params::DAA_EN);
	param_set(param_daa_en, &daa_en);

	navigator->get_detect_and_avoid()->on_activation();

	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();
}

// Primary and secondary ownship ICAO default to -1 (disabled), so unrelated traffic is not suppressed.
TEST_F(DetectAndAvoidTest, PrimaryAndSecondaryIcaoDefaultsDisabled)
{
	int32_t primary_icao = 0;
	ASSERT_EQ(param_get(param_handle(px4::params::ADSB_ICAO_ID), &primary_icao), 0);
	EXPECT_EQ(primary_icao, -1);

	int32_t secondary_icao = 0;
	ASSERT_EQ(param_get(param_handle(px4::params::ADSB_ICAO_ID_2), &secondary_icao), 0);
	EXPECT_EQ(secondary_icao, -1);
}

// stop_fake_traffic() cancels a queued script before it publishes any synthetic traffic.
TEST_F(DetectAndAvoidTest, StopFakeTrafficCancelsPendingScript)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	navigator->run_fake_traffic(DetectAndAvoid::FakeTraffMode::kQueueFill);
	navigator->stop_fake_traffic();
	drain_mavlink_logs();

	navigator->check_traffic();

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();

	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New and Main:"), 0u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New "), 0u);
}

// A NaN traffic altitude is rejected before any DAA math; nothing is published or buffered.
TEST_F(DetectAndAvoidTest, RejectsNonFiniteTrafficAltitude)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f, flags);

	publish_transponder_report_and_check(tr);

	EXPECT_FALSE(_detect_and_avoid_sub.update());

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
}

// A report missing a required validity flag (here VALID_ALTITUDE) is dropped.
TEST_F(DetectAndAvoidTest, RejectsTrafficMissingRequiredFlags)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	// Same geometry as ownship (an immediate conflict if processed), but VALID_ALTITUDE is absent.
	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav, alt_uav, 30.f, 0.f, flags);

	publish_transponder_report_and_check(tr);

	EXPECT_FALSE(_detect_and_avoid_sub.update());
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);
}

// Finite coordinates are still unusable when the estimator marks either global position group invalid.
TEST_F(DetectAndAvoidTest, RejectsInvalidOwnshipGlobalPositionFlags)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;

	publish_land_status(false);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel);
	publish_global_pos(lat_uav, lon_uav, alt_uav, hrt_absolute_time(), false, true);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  alt_uav, 0.f, 0.f, flags);
	publish_transponder_report_and_check(tr);

	EXPECT_FALSE(_detect_and_avoid_sub.update());
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);

	// Drain the queued report after switching to an altitude-invalid pose as well.
	publish_global_pos(lat_uav, lon_uav, alt_uav, hrt_absolute_time(), true, false);
	sync_navigator_topics();
	navigator->check_traffic();
	EXPECT_FALSE(_detect_and_avoid_sub.update());
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);
}

// A report older than DAA_TRAFF_TOUT is dropped.
TEST_F(DetectAndAvoidTest, RejectsStaleTrafficReport)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// Shortest allowed timeout so a modest backdate is unambiguously stale.
	const int32_t traffic_timeout_s = 1;
	param_set(param_handle(px4::params::DAA_TRAFF_TOUT), &traffic_timeout_s);
	reload_daa_parameters();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav, alt_uav, 30.f, 0.f, flags);

	// Backdate well beyond the 1 s timeout. The harness has been running long enough (Navigator
	// construction waits on dataman) that hrt_absolute_time() is comfortably past the offset.
	const hrt_abstime backdate = 5_s;
	const hrt_abstime now = hrt_absolute_time();
	tr.timestamp = (now > backdate) ? (now - backdate) : 1;

	publish_transponder_report_and_check(tr);

	EXPECT_FALSE(_detect_and_avoid_sub.update());
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);

	param_reset_all();
}

// Source age (tslc) participates in input freshness even when the local publication is new.
TEST_F(DetectAndAvoidTest, RejectsSourceStaleTrafficReport)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const int32_t traffic_timeout_s = 1;

	param_set(param_handle(px4::params::DAA_TRAFF_TOUT), &traffic_timeout_s);
	reload_daa_parameters();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  alt_uav, 0.f, 0.f, flags);
	tr.tslc = 2;

	publish_transponder_report_and_check(tr);

	EXPECT_FALSE(_detect_and_avoid_sub.update());
	EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);
}

// Ownship reports (ICAO, callsign or UAS ID) are filtered out; foreign traffic with the same
// geometry creates a conflict. Identity comes from parameters and the board GUID.
TEST_F(DetectAndAvoidTest, SelfDetection)
{
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;

	const auto refresh_uav_pose = [&]() {
		set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
		sync_navigator_topics();
	};

	const auto expect_report_filtered = [&](const transponder_report_s & tr) {
		refresh_uav_pose();
		drain_detect_and_avoid_topic();
		publish_transponder_report_and_check(tr);

		EXPECT_FALSE(_detect_and_avoid_sub.update());
		EXPECT_EQ(navigator->get_detect_and_avoid()->get_most_urgent_conflict().conflict_level, kDaaConflictLvlNone);
	};

	const auto expect_report_in_conflict = [&](const transponder_report_s & tr, const uint64_t expected_id) {
		refresh_uav_pose();
		drain_detect_and_avoid_topic();
		publish_transponder_report_and_check(tr);

		EXPECT_TRUE(_detect_and_avoid_sub.update());
		const conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
		EXPECT_GT(conflict.conflict_level, kDaaConflictLvlNone);
		EXPECT_EQ(conflict.encoded_id.id, expected_id);
	};

	// Each parameter stores four callsign characters in display order (most-significant byte first).
	char callsign[kCallsignLength] = "PX4 TEST";
	const int32_t own_adsb_callsign1 = 0x50583420; // "PX4 "
	const int32_t own_adsb_callsign2 = 0x54455354; // "TEST"
	param_set(param_handle(px4::params::ADSB_CALLSIGN_1), &own_adsb_callsign1);
	param_set(param_handle(px4::params::ADSB_CALLSIGN_2), &own_adsb_callsign2);
	reload_daa_parameters();

	// ICAO zeroed so the callsign identifies it: ownship callsign dropped, foreign callsign conflicts
	transponder_report_s tr = create_transponder_report(0, callsign, lat_uav, lon_uav, alt_uav, 30.f, 0.f,
				  flags | transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN);
	expect_report_filtered(tr);

	char foreign_callsign[kCallsignLength];
	memcpy(foreign_callsign, callsign, sizeof(foreign_callsign));
	foreign_callsign[0] = (callsign[0] == 'A') ? 'B' : 'A';
	set_report_callsign(tr, foreign_callsign);
	expect_report_in_conflict(tr, DaaEncodedId::callsign_to_uint64(foreign_callsign));

	// primary and secondary ICAO as ownship identity: both dropped, foreign ICAO conflicts
	const uint32_t own_icao = 6593425;
	const uint32_t own_icao_2 = 3318901;
	param_set(param_handle(px4::params::ADSB_ICAO_ID), &own_icao);
	param_set(param_handle(px4::params::ADSB_ICAO_ID_2), &own_icao_2);
	reload_daa_parameters(); // also clears the previous foreign conflict

	expect_report_filtered(create_transponder_report(own_icao, "DDF0A1", lat_uav, lon_uav, alt_uav, 30.f, 0.f, flags));
	expect_report_filtered(create_transponder_report(own_icao_2, "DDF0A1", lat_uav, lon_uav, alt_uav, 30.f, 0.f, flags));

	const uint32_t foreign_icao = 14545057;
	expect_report_in_conflict(create_transponder_report(foreign_icao, "DDF0A1", lat_uav, lon_uav, alt_uav, 30.f, 0.f,
				  flags), foreign_icao);

	// UAS-ID only (no ICAO, no valid callsign)
	reload_daa_parameters(); // clear the previous foreign conflict

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid {};
	ASSERT_EQ(board_get_px4_guid(px4_guid), PX4_GUID_BYTE_LENGTH);

	// boards with a UUID drop traffic broadcasting the local UAS ID
	transponder_report_s tr_own_uas = create_transponder_report(0, "", lat_uav, lon_uav, alt_uav, 30.f, 0.f, flags);
	memcpy(tr_own_uas.uas_id, px4_guid, sizeof(px4_guid));
	expect_report_filtered(tr_own_uas);
#endif // !BOARD_HAS_NO_UUID

	// foreign UAS ID conflicts (also on boards without a UUID)
	transponder_report_s tr_foreign_uas = create_transponder_report(0, "", lat_uav, lon_uav, alt_uav, 30.f, 0.f, flags);

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH; ++i) {
		tr_foreign_uas.uas_id[i] = 0xe0 + i;
	}

	expect_report_in_conflict(tr_foreign_uas, DaaEncodedId::last_uas_id_bytes_to_uint64(tr_foreign_uas.uas_id));
}
