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

// WHY: Activation and deactivation gate all DetectAndAvoid processing and must fail closed on bad configuration.
// WHAT: Verify successful startup with default params, then confirm the disabled parameter set leaves the module inactive.
TEST_F(DetectAndAvoidTest, OnActivation)
{
	// GIVEN: DetectAndAvoid starts with the default valid parameter set.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	// WHEN: The module is explicitly deactivated.
	navigator->get_detect_and_avoid()->on_inactivation();

	// THEN: It reports itself inactive.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());

	// WHEN: The feature is disabled through parameters before activation.
	const int daa_en = 0;
	param_t param_daa_en = param_handle(px4::params::DAA_EN);
	param_set(param_daa_en, &daa_en);

	navigator->get_detect_and_avoid()->on_activation();

	// THEN: Activation is refused while the feature is disabled.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();
}

// WHY: ICAO identifiers are vehicle-specific, so the default configuration must not suppress unrelated traffic.
// WHAT: Read the default primary and secondary ICAO parameters and verify both are disabled by default.
TEST_F(DetectAndAvoidTest, PrimaryAndSecondaryIcaoDefaultsDisabled)
{
	int32_t primary_icao = 0;
	ASSERT_EQ(param_get(param_handle(px4::params::ADSB_ICAO_ID), &primary_icao), 0);
	EXPECT_EQ(primary_icao, -1);

	int32_t secondary_icao = 0;
	ASSERT_EQ(param_get(param_handle(px4::params::ADSB_ICAO_ID_2), &secondary_icao), 0);
	EXPECT_EQ(secondary_icao, -1);
}

// WHY: Operators need a way to cancel a queued fake-traffic script before it publishes more synthetic traffic.
// WHAT: Arm a fake-traffic mode, stop it before the next navigator cycle, and verify no synthetic conflicts are emitted.
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

// WHY: Invalid traffic altitude should be rejected before any DAA math runs to avoid propagating NaNs through conflict evaluation.
// WHAT: Publish a report with a NaN altitude and verify that no detect_and_avoid sample is published and no conflict is buffered.
TEST_F(DetectAndAvoidTest, RejectsNonFiniteTrafficAltitude)
{
	// GIVEN: Ownship is initialized and the detect_and_avoid topic is drained.
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

	// WHEN: Traffic is published with a NaN altitude.
	publish_transponder_report_and_check(tr);

	// THEN: No output is published and no conflict is buffered.
	EXPECT_FALSE(_detect_and_avoid_sub.update());

	conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
}

// WHY: Ownship reports must never enter the avoidance buffer regardless of whether identity comes from ICAO, callsign, or UAS ID.
// WHAT: Configure each self identifier format and verify is_self_detection() recognizes matching traffic.
TEST_F(DetectAndAvoidTest, SelfDetection)
{
	// GIVEN: DetectAndAvoid is active with default parameters.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	char callsign[kCallsignLength];
	transponder_report_s tr{};

	// WHEN: A callsign is configured as ownship identity (split into the two ADSB_CALLSIGN_* params).
	generate_random_callsign(callsign, 8);
	const uint64_t callsign_64 = DaaEncodedId::callsign_to_uint64(callsign);
	const uint32_t own_adsb_callsign1 = static_cast<uint32_t>(callsign_64 & 0xFFFFFFFF);
	const uint32_t own_adsb_callsign2 = static_cast<uint32_t>((callsign_64 >> 32) & 0xFFFFFFFF);
	param_set(param_handle(px4::params::ADSB_CALLSIGN_1), &own_adsb_callsign1);
	param_set(param_handle(px4::params::ADSB_CALLSIGN_2), &own_adsb_callsign2);
	reload_daa_parameters();

	tr.icao_address = 0; // Invalid ICAO so the callsign identifies the report.
	set_report_callsign(tr, callsign);
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

	// THEN: Traffic broadcasting the ownship callsign is treated as self.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(DaaEncodedId::from_report(tr)));

	// WHEN: The primary ICAO address matches ownship.
	const uint32_t own_icao = 6593425;
	tr.icao_address = own_icao;

	// THEN: With the default disabled primary ICAO parameter, real traffic is not treated as ownship.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_self_detection(DaaEncodedId::from_report(tr)));

	param_set(param_handle(px4::params::ADSB_ICAO_ID), &own_icao);
	reload_daa_parameters();

	// THEN: Once configured, matching primary ICAO traffic is treated as self.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(DaaEncodedId::from_report(tr)));

	// WHEN: The secondary ICAO address matches ownship.
	const uint32_t own_icao_2 = 3318901;
	param_set(param_handle(px4::params::ADSB_ICAO_ID_2), &own_icao_2);
	reload_daa_parameters();
	tr.icao_address = own_icao_2;

	// THEN: Matching secondary ICAO traffic is also treated as self.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(DaaEncodedId::from_report(tr)));

	// WHEN: The traffic report carries only a UAS ID.
	transponder_report_s tr_uas_id{};

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid {};
	board_get_px4_guid(px4_guid);
	memcpy(tr_uas_id.uas_id, px4_guid, sizeof(px4_guid));
	const DaaEncodedId own_uas_id = DaaEncodedId::from_report(tr_uas_id);
	ASSERT_NE(own_uas_id.id, 0u);

	// THEN: On boards with a UUID, the local UAS ID is recognized as self.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(own_uas_id));
#else

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH; ++i) {
		tr_uas_id.uas_id[i] = 0xe0 + i;
	}

	const DaaEncodedId foreign_uas_id = DaaEncodedId::from_report(tr_uas_id);
	ASSERT_NE(foreign_uas_id.id, 0u);

	// THEN: Without a board UUID, an arbitrary UAS ID is not considered self.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_self_detection(foreign_uas_id));
#endif
}
