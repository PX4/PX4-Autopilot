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

struct action_priority_test_case_s {
	const char *name;
	DaaAction action;
	DetectAndAvoidTest::expected_outcome_s expected_outcome;
};

class DetectAndAvoidActionPriorityTest : public DetectAndAvoidTest, public ::testing::WithParamInterface<action_priority_test_case_s>
{
};

std::string action_priority_test_case_name(const ::testing::TestParamInfo<action_priority_test_case_s> &info)
{
	return info.param.name;
}

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

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

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

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
}

// WHY: Ownship reports must never enter the avoidance buffer regardless of whether identity comes from ICAO, callsign, or UAS ID.
// WHAT: Configure each self identifier format and verify DetectAndAvoid recognizes and suppresses matching traffic.
TEST_F(DetectAndAvoidTest, SelfDetection)
{
	/*
		Method used to set call sign parameters:
			1. Start with callsign
			2. Convert to uint64:
				callsign_to_uint64(const char callsign[kCallsignLength]);
			3. Convert to two uint32
				const uint32_t lower = static_cast<uint32_t>(unique_id.id & 0xFFFFFFFF);
				const uint32_t upper = static_cast<uint32_t>((unique_id.id >> 32) & 0xFFFFFFFF);

		Method used to set ICAO parameters:
			uint32_t icao_uint = static_cast<uint32_t>(std::stoul(icao_str, nullptr, 16));

	*/

	// GIVEN: DetectAndAvoid is active with default parameters.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	// GIVEN: Random ADS-B callsigns will be configured as ownship identifiers.
	char callsign[9];
	transponder_report_s tr{};
	unique_id_s unique_id;

	// WHEN: Callsign identifiers are written into the self-identification parameters.
	for (int i = 0; i < 30; ++i) {
		generate_random_callsign(callsign, 8);
		const uint64_t callsign_64 = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);
		const uint32_t own_adsb_callsign1 = static_cast<uint32_t>(callsign_64 & 0xFFFFFFFF);
		const uint32_t own_adsb_callsign2 = static_cast<uint32_t>((callsign_64 >> 32) & 0xFFFFFFFF);
		param_set(param_handle(px4::params::ADSB_CALLSIGN_1), &own_adsb_callsign1);
		param_set(param_handle(px4::params::ADSB_CALLSIGN_2), &own_adsb_callsign2);

		navigator->get_detect_and_avoid()->updateParams();

		tr.icao_address = 0; // Invalid ICAO
		strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
		tr.callsign[sizeof(tr.callsign) - 1] = 0;
		tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

		navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

		// THEN: Matching callsigns are recognized as self-detections.
		EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
		EXPECT_TRUE(unique_id.id == callsign_64);
		EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));
	}

	// WHEN: The primary ICAO address is configured as ownship.
	const uint32_t own_icao = 6593425;
	tr.icao_address = own_icao;
	navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

	// THEN: With the default disabled primary ICAO parameter, real traffic is not treated as ownship.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));

	param_set(param_handle(px4::params::ADSB_ICAO_ID), &own_icao);
	navigator->get_detect_and_avoid()->updateParams();

	tr.icao_address = own_icao;
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

	navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

	// THEN: Matching primary ICAO traffic is treated as self-detection.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	EXPECT_TRUE(unique_id.id == own_icao);
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));

	// WHEN: The secondary ICAO address is configured as ownship.
	const uint32_t own_icao_2 = 3318901;
	param_set(param_handle(px4::params::ADSB_ICAO_ID_2), &own_icao_2);
	navigator->get_detect_and_avoid()->updateParams();

	tr.icao_address = own_icao_2;
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

	navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

	// THEN: Matching secondary ICAO traffic is also treated as self-detection.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	EXPECT_TRUE(unique_id.id == own_icao_2);
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));

	// WHEN: The traffic report carries only a UAS ID.
	transponder_report_s tr_uas_id{};
	tr_uas_id.icao_address = 0;
	memset(tr_uas_id.callsign, 0, sizeof(tr_uas_id.callsign));

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid {};
	board_get_px4_guid(px4_guid);
	memcpy(tr_uas_id.uas_id, px4_guid, sizeof(px4_guid));
	EXPECT_TRUE(navigator->get_detect_and_avoid()->get_unique_id(tr_uas_id, unique_id));

	// THEN: On boards with a UUID, the local UAS ID is recognized as self.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID);
	EXPECT_TRUE(unique_id.id == navigator->get_detect_and_avoid()->last_uas_id_bytes_to_uint64(tr_uas_id.uas_id));
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));
#else

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH; ++i) {
		tr_uas_id.uas_id[i] = 0xe0 + i;
	}

	EXPECT_TRUE(navigator->get_detect_and_avoid()->get_unique_id(tr_uas_id, unique_id));

	// THEN: Without a board UUID, an arbitrary UAS ID is not considered self.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID);
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));
#endif

	// WHEN: Traffic provides no usable identity fields.
	transponder_report_s tr_no_id{};
	tr_no_id.icao_address = 0;
	strncpy(&tr_no_id.callsign[0], callsign, sizeof(tr_no_id.callsign) - 1);
	tr_no_id.callsign[sizeof(tr.callsign) - 1] = 0;
	tr_no_id.flags = 0;

	// THEN: No unique identifier can be extracted.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->get_unique_id(tr_no_id, unique_id));
}

// WHY: Callsign packing and unpacking must round-trip cleanly so buffer keys and operator-facing messages stay consistent.
// WHAT: Convert random callsigns to uint64 and back, then verify the recovered string produces the same encoded value and reject unterminated input.
TEST_F(DetectAndAvoidTest, ReversibleCallsign)
{
	/*
		Ensure conversion is reversible
			Callsign:
				- uint = callsign_to_uint64
				- recovered = convert_uint64_callsign_to_str(uint)
				- back_to_uint = callsign_to_uint64(recovered)

				--> uint == back_to_uint
	*/

	// GIVEN: DetectAndAvoid is active and the callsign helpers are available.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	char callsign[9];

	// WHEN: Random null-terminated callsigns are converted to uint64 and back.
	for (int i = 0; i < 30; ++i) {
		generate_random_callsign(callsign, 8);
		const uint64_t callsign_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);
		char recovered[9];
		navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(callsign_uint, recovered);
		const uint64_t final_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(recovered);

		// THEN: The encoding round-trips without changing the packed value.
		EXPECT_TRUE(callsign_uint == final_uint);

		// Only debug print once
		if (i == 0) {

			PX4_DEBUG("TEST_F: UniqueID, init: %s (int %lu), recovered: %s (int %lu)", callsign, callsign_uint, recovered,
				  final_uint);

			char part1_recovered[9];
			char part2_recovered[9];
			uint32_t lower = static_cast<uint32_t>(callsign_uint & 0xFFFFFFFF);
			uint32_t upper = static_cast<uint32_t>((callsign_uint >> 32) & 0xFFFFFFFF);
			navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(uint64_t(lower), part1_recovered);
			navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(uint64_t(upper), part2_recovered);
			PX4_DEBUG("TEST_F: full: %s =? %s---%s == PART1---PART2", recovered, part1_recovered, part2_recovered);
		}
	}

	// WHEN: A callsign is not null terminated.
	char callsign_non_null[9];
	generate_random_callsign(callsign_non_null, 8);
	callsign_non_null[8] = 1;

	// THEN: The helper rejects the malformed input.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->callsign_to_uint64(callsign_non_null) == 0);
}

// WHY: Callsigns are stored as bytes, so high-bit characters must only affect their own byte slot and never sign-extend across the uint64_t key.
// WHAT: Encode a callsign containing bytes with the MSB set and verify the packed value preserves only those byte values.
TEST_F(DetectAndAvoidTest, CallsignPackingDoesNotSignExtend)
{
	const char callsign[kCallsignLength] {static_cast<char>(0x80), static_cast<char>(0xFF), '\0'};
	const uint64_t callsign_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);

	EXPECT_EQ(callsign_uint, static_cast<uint64_t>(0xFF80u));
}

// WHY: MAVLink fixed-length string fields copy every byte, so unused callsign bytes must be deterministic after decoding.
// WHAT: Decode a short callsign into a pre-filled buffer and verify all bytes after the terminator are cleared.
TEST_F(DetectAndAvoidTest, CallsignDecodeClearsUnusedBytes)
{
	const char callsign[kCallsignLength] {'A', 'B', '\0'};
	const uint64_t callsign_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);
	char recovered[kCallsignLength];
	memset(recovered, 0x7F, sizeof(recovered));

	navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(callsign_uint, recovered);

	EXPECT_STREQ(recovered, "AB");

	for (int i = 2; i < kCallsignLength; ++i) {
		EXPECT_EQ(recovered[i], '\0');
	}
}

// WHY: ICAO identifiers are 24-bit values, so operator-facing formatting must stay fixed-width and avoid platform-specific truncation.
// WHAT: Format representative ICAO values and verify the output is uppercase, zero-padded, and derived from the low 24 bits.
TEST_F(DetectAndAvoidTest, FormatsIcaoAsFixedWidthHex)
{
	// GIVEN: A buffer sized for a 6-character ICAO address plus the null terminator.
	char icao_buffer[kIcaoLength] {};

	// WHEN: A short ICAO value is formatted.
	DetectAndAvoid::convert_icao_uint32_to_hex_str(0xABCu, icao_buffer, sizeof(icao_buffer));

	// THEN: The helper zero-pads and uppercases the address.
	EXPECT_STREQ(icao_buffer, "000ABC");

	// WHEN: High bits are present in the input value.
	DetectAndAvoid::convert_icao_uint32_to_hex_str(0x12ABCDEFu, icao_buffer, sizeof(icao_buffer));

	// THEN: Only the ICAO address bits are rendered.
	EXPECT_STREQ(icao_buffer, "ABCDEF");
}

// WHY: Reduced UAS-ID packing must preserve the exact GUID tail bytes used as the unique identifier.
// WHAT: Round-trip random and zero-valued GUID tails through the uint64 conversion helpers and compare every byte.
TEST_F(DetectAndAvoidTest, ReversibleUasId)
{
	/*
		Ensure conversion is reversible
			UAS_ID:
				- uint = last_uas_id_bytes_to_uint64(uas_id);
				- recovered = uint64_to_last_uas_id_bytes(uint);

				--> last_uas_id_bytes == recovered_bytes
	*/

	// GIVEN: DetectAndAvoid is active and reduced UAS-ID packing is available.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	uint8_t uas_id[PX4_GUID_BYTE_LENGTH];

	// WHEN: Random UAS IDs are packed into uint64 and unpacked again.
	for (int i = 0; i < 30; ++i) {
		generate_random_UAS_ID(uas_id);
		const uint64_t uas_id_uint = navigator->get_detect_and_avoid()->last_uas_id_bytes_to_uint64(
						     uas_id); // Converts last kIdEncodingNbBytes bytes
		uint8_t recovered[kIdEncodingNbBytes]; // Gets the last kIdEncodingNbBytes back
		navigator->get_detect_and_avoid()->uint64_to_last_uas_id_bytes(uas_id_uint, recovered);

		// THEN: The packed representation preserves the trailing GUID bytes exactly.
		for (int k = 0; k < kIdEncodingNbBytes; ++k) {
			EXPECT_TRUE(uas_id[PX4_GUID_BYTE_LENGTH - kIdEncodingNbBytes + k] == recovered[k]);

			if (i == 0) {
				PX4_DEBUG(" Byte %d/%d: uas == %d == %d == recovered", PX4_GUID_BYTE_LENGTH - kIdEncodingNbBytes + k + 1,
					  PX4_GUID_BYTE_LENGTH, uas_id[PX4_GUID_BYTE_LENGTH - kIdEncodingNbBytes + k], recovered[k]);
			}

		}
	}

	// WHEN: The UAS ID is all zeros.
	uint8_t zero_uas_id[PX4_GUID_BYTE_LENGTH];

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH; i++) {
		zero_uas_id[i] = 0;
	}

	const uint64_t zero_uas_id_uint = navigator->get_detect_and_avoid()->last_uas_id_bytes_to_uint64(
			zero_uas_id); // Converts last kIdEncodingNbBytes bytes

	ASSERT_TRUE(zero_uas_id_uint == 0);

	// WHEN: The zero value is unpacked again.
	uint8_t zero_recovered[kIdEncodingNbBytes];
	navigator->get_detect_and_avoid()->uint64_to_last_uas_id_bytes(zero_uas_id_uint, zero_recovered);

	// THEN: The unpacked trailing bytes remain zeroed.
	for (int k = 0; k < kIdEncodingNbBytes; ++k) {
		EXPECT_TRUE(0 == zero_recovered[k]);
	}
}

// WHY: Automatic DAA actions must only escalate when they are stronger than the current navigator state permits.
// WHAT: Evaluate each requested action across representative navigation states and verify the expected escalation decisions.
TEST_P(DetectAndAvoidActionPriorityTest, ActionPriorities)
{
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	const action_priority_test_case_s &test_case = GetParam();
	check_eval_conflict_escalation_action(navigator.get(), test_case.expected_outcome, test_case.action);
}

INSTANTIATE_TEST_SUITE_P(
	AllActions,
	DetectAndAvoidActionPriorityTest,
	::testing::Values(
		action_priority_test_case_s{"Disabled", DaaAction::kDisabled, {false, false, false, false}},
		action_priority_test_case_s{"WarnOnly", DaaAction::kWarnOnly, {false, false, false, false}},
		action_priority_test_case_s{"Hold", DaaAction::kPositionHoldMode, {true, false, false, false}},
		action_priority_test_case_s{"ReturnMode", DaaAction::kReturnMode, {true, true, false, false}},
		action_priority_test_case_s{"LandMode", DaaAction::kLandMode, {true, true, true, false}},
		action_priority_test_case_s{"Terminate", DaaAction::kTerminate, {true, true, true, true}}),
	action_priority_test_case_name);
