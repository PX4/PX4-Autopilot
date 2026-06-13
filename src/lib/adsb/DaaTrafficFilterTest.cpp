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
 * @file DaaTrafficFilterTest.cpp
 * @brief Unit tests for the DAA traffic input filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include <lib/adsb/DaaTrafficFilter.h>

namespace
{

static constexpr hrt_abstime kNow{100'000'000}; // 100 s
static constexpr hrt_abstime kTimeoutUs{10'000'000}; // 10 s

transponder_report_s valid_report()
{
	transponder_report_s report{};
	report.timestamp = kNow;
	report.icao_address = 0xDDF0A1;
	report.lat = 46.52;
	report.lon = 6.52;
	report.altitude = 400.f;
	report.heading = 0.f;
	report.hor_velocity = 30.f;
	report.ver_velocity = 0.f;
	report.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
		       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
		       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
		       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	return report;
}

} // namespace

// WHY: Invalid traffic geometry must be rejected before any DAA math runs to avoid propagating NaNs.
// WHAT: Verify a fully valid report passes and each non-finite coordinate field is rejected.
TEST(DaaTrafficFilterTest, TransponderDataValidRejectsNonFiniteFields)
{
	EXPECT_TRUE(DaaTrafficFilter::transponder_data_valid(valid_report(), kNow, kTimeoutUs));

	transponder_report_s report = valid_report();
	report.lat = std::numeric_limits<double>::quiet_NaN();
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report = valid_report();
	report.lon = std::numeric_limits<double>::quiet_NaN();
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report = valid_report();
	report.altitude = std::numeric_limits<float>::quiet_NaN();
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));
}

// WHY: A report is only usable when the transponder is validated for the fields the built standard consumes.
// WHAT: Drop each required validity flag and verify the report is rejected (heading/velocity only outside F3442).
TEST(DaaTrafficFilterTest, TransponderDataValidRequiresFlags)
{
	transponder_report_s report = valid_report();
	report.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report = valid_report();
	report.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report = valid_report();
	report.flags &= ~(transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY);

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	// F3442 does not consume the reported heading or horizontal velocity.
	EXPECT_TRUE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));
#else
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	// Crosstrack mode depends on a finite traffic heading.
	report = valid_report();
	report.heading = std::numeric_limits<float>::quiet_NaN();
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));
#endif // CONFIG_NAVIGATOR_ADSB_F3442
}

// WHY: Stale traffic must not (re)enter the pipeline, and an unset timestamp must fail.
// WHAT: Check timestamp across the timeout boundary at a fixed current time.
TEST(DaaTrafficFilterTest, TransponderDataValidRejectsStaleTimestamps)
{
	transponder_report_s report = valid_report();
	report.timestamp = 0;
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report.timestamp = kNow - kTimeoutUs - 1;
	EXPECT_FALSE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report.timestamp = kNow - kTimeoutUs;
	EXPECT_TRUE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));

	report.timestamp = kNow;
	EXPECT_TRUE(DaaTrafficFilter::transponder_data_valid(report, kNow, kTimeoutUs));
}

// WHY: ICAO identifiers are vehicle-specific; unset (negative) parameters must never discard real traffic.
// WHAT: Check primary and secondary ICAO matching against set and unset ownship identifiers.
TEST(DaaTrafficFilterTest, SelfDetectionIcao)
{
	const DaaEncodedId traffic_id{0x123456, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO};

	// THEN: With both ICAO identifiers unset (negative), nothing matches.
	daa_ownship_ids_s ownship_ids{};
	EXPECT_FALSE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: The primary ICAO matches.
	ownship_ids.icao = 0x123456;
	EXPECT_TRUE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: Only the secondary ICAO matches.
	ownship_ids = {};
	ownship_ids.icao_2 = 0x123456;
	EXPECT_TRUE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: Configured but different identifiers do not skip traffic.
	ownship_ids.icao = 0x654321;
	ownship_ids.icao_2 = 0x654322;
	EXPECT_FALSE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));
}

// WHY: Callsign identity is split across two parameters and the packed comparison must match exactly.
// WHAT: Compare a packed traffic callsign against matching, unset, and different ownship callsigns.
TEST(DaaTrafficFilterTest, SelfDetectionCallsign)
{
	const char callsign[kCallsignLength] = "TST1234";
	const uint64_t packed_callsign = DaaEncodedId::callsign_to_uint64(callsign);
	ASSERT_NE(packed_callsign, 0u);

	const DaaEncodedId traffic_id{packed_callsign, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN};

	// THEN: An unset ownship callsign does not match.
	daa_ownship_ids_s ownship_ids{};
	EXPECT_FALSE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: The matching packed callsign is recognized as self.
	ownship_ids.callsign = packed_callsign;
	EXPECT_TRUE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: A single-bit difference is foreign traffic.
	ownship_ids.callsign = packed_callsign ^ 1u;
	EXPECT_FALSE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));
}

// WHY: Without a board UUID there is no own UAS identity, so UAS traffic must never be treated as self.
// WHAT: Compare a packed traffic UAS-ID against the ownship key with the valid flag set and cleared.
TEST(DaaTrafficFilterTest, SelfDetectionUasId)
{
	uint8_t uas_id_bytes[kUasIdByteLength];

	for (int i = 0; i < kUasIdByteLength; ++i) {
		uas_id_bytes[i] = 0xE0 + i;
	}

	const uint64_t packed_uas_id = DaaEncodedId::last_uas_id_bytes_to_uint64(uas_id_bytes);
	ASSERT_NE(packed_uas_id, 0u);

	const DaaEncodedId traffic_id{packed_uas_id, detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID};

	// THEN: A matching key is ignored while the ownship UAS-ID is marked unavailable.
	daa_ownship_ids_s ownship_ids{};
	ownship_ids.uas_id = packed_uas_id;
	EXPECT_FALSE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: Once available, the matching key is recognized as self.
	ownship_ids.uas_id_valid = true;
	EXPECT_TRUE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));

	// WHEN/THEN: A different key is incoming traffic.
	ownship_ids.uas_id = packed_uas_id ^ 1u;
	EXPECT_FALSE(DaaTrafficFilter::is_self_detection(traffic_id, ownship_ids));
}

// WHY: The identification must reject reports without a usable identity and ownship reports,
// and hand a decoded identifier to the tracker for everything else.
// WHAT: Identify an empty report, an incoming ICAO report, and the same report with a matching ownship ICAO.
TEST(DaaTrafficFilterTest, IdentifyTrafficReport)
{
	daa_ownship_ids_s ownship_ids{};
	DaaEncodedId encoded_id{};

	// WHEN/THEN: A report without ICAO, callsign or UAS-ID carries no usable identifier.
	transponder_report_s report{};
	EXPECT_FALSE(DaaTrafficFilter::identify_traffic_report(report, ownship_ids, encoded_id));

	// WHEN/THEN: Am incoming ICAO report is identified and decoded.
	report.icao_address = 0xABCDEF;
	EXPECT_TRUE(DaaTrafficFilter::identify_traffic_report(report, ownship_ids, encoded_id));
	EXPECT_EQ(encoded_id.id, 0xABCDEFu);
	EXPECT_EQ(encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);

	// WHEN/THEN: The same report is rejected once the ICAO is configured as ownship.
	ownship_ids.icao = 0xABCDEF;
	EXPECT_FALSE(DaaTrafficFilter::identify_traffic_report(report, ownship_ids, encoded_id));
}
