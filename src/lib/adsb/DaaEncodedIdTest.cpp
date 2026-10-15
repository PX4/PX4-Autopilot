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
 * @file DaaEncodedIdTest.cpp
 * @brief Unit tests for the DaaEncodedId.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <cstring>

#include <lib/adsb/DaaEncodedId.h>

namespace
{

void set_report_callsign(transponder_report_s &report, const char *callsign)
{
	strncpy(report.callsign, callsign, sizeof(report.callsign) - 1);
	report.callsign[sizeof(report.callsign) - 1] = '\0';
}

} // namespace

// Callsign packs to a 64-bit key and back; the round-trip re-encodes identically, unterminated rejected.
TEST(DaaEncodedIdTest, CallsignRoundTrip)
{
	const char *callsigns[] {
		"A",
		"PX4TEST1",
		"ABCD1234",
		"ZZ999999",
	};

	for (const char *callsign : callsigns) {
		const uint64_t key = DaaEncodedId::callsign_to_uint64(callsign);

		char recovered[kCallsignLength];
		DaaEncodedId::convert_uint64_callsign_to_str(key, recovered);

		EXPECT_EQ(key, DaaEncodedId::callsign_to_uint64(recovered));
	}

	// unterminated input -> 0
	char callsign_non_null[kCallsignLength];
	memset(callsign_non_null, 'A', sizeof(callsign_non_null));
	EXPECT_EQ(DaaEncodedId::callsign_to_uint64(callsign_non_null), 0u);
}

// High-bit callsign bytes stay in their own byte slot; no sign extension across the key.
TEST(DaaEncodedIdTest, CallsignPackingDoesNotSignExtend)
{
	const char callsign[kCallsignLength] {static_cast<char>(0x80), static_cast<char>(0xFF), '\0'};

	EXPECT_EQ(DaaEncodedId::callsign_to_uint64(callsign), static_cast<uint64_t>(0xFF80u));
}

TEST(DaaEncodedIdTest, CallsignParamsUseDocumentedCharacterOrder)
{
	// Documented parameter values: "PX4 " = 0x50583420, "TEST" = 0x54455354.
	const uint64_t encoded = DaaEncodedId::callsign_params_to_uint64(0x50583420, 0x54455354);
	EXPECT_EQ(encoded, 0x5453455420345850ULL);

	char decoded[kCallsignLength] {};
	DaaEncodedId::convert_uint64_callsign_to_str(encoded, decoded);
	EXPECT_STREQ(decoded, "PX4 TEST");

	// An embedded terminator ends a shorter callsign just as it does in a traffic report.
	EXPECT_EQ(DaaEncodedId::callsign_params_to_uint64(0x50583400, 0x54455354),
		  DaaEncodedId::callsign_to_uint64("PX4"));
	EXPECT_EQ(DaaEncodedId::callsign_params_to_uint64(0, 0), 0u);
}

// to_string() renders each encoding, plus an explicit placeholder for an unknown one.
TEST(DaaEncodedIdTest, ToStringRendersEachEncoding)
{
	char buffer[kUtmGuidMsgLength];

	// ICAO -> fixed-width uppercase hex of the low 24 bits.
	DaaEncodedId{0xABCu, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO}.to_string(buffer, sizeof(buffer));
	EXPECT_STREQ(buffer, "000ABC");

	// Callsign -> the decoded callsign string.
	const uint64_t callsign_key = DaaEncodedId::callsign_to_uint64("AB");
	DaaEncodedId{callsign_key, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN}.to_string(buffer, sizeof(buffer));
	EXPECT_STREQ(buffer, "AB");

	const char callsign_with_controls[kCallsignLength] {'A', '\n', static_cast<char>(0x7f), 'B', '\0'};
	DaaEncodedId{DaaEncodedId::callsign_to_uint64(callsign_with_controls),
		     detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN}.to_string(buffer, sizeof(buffer));
	EXPECT_STREQ(buffer, "A??B");

	// UAS ID -> reduced lowercase hex of the packed tail bytes (little-endian).
	DaaEncodedId{0x0102030405u, detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID}.to_string(buffer, sizeof(buffer));
	EXPECT_STREQ(buffer, "0504030201");

	// Unknown encoding -> explicit placeholder.
	char unknown_buffer[32];
	DaaEncodedId{42u, 0xFF}.to_string(unknown_buffer, sizeof(unknown_buffer));
	EXPECT_STREQ(unknown_buffer, "Unknown ID.");
}

TEST(DaaEncodedIdTest, ToStringRespectsBufferSize)
{
	struct guarded_buffer_s {
		char output[4];
		char guard;
	};

	guarded_buffer_s callsign_buffer{{}, 'X'};
	const uint64_t callsign_key = DaaEncodedId::callsign_to_uint64("ABCDEFGH");
	DaaEncodedId{callsign_key, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN}.to_string(
		callsign_buffer.output, sizeof(callsign_buffer.output));
	EXPECT_STREQ(callsign_buffer.output, "ABC");
	EXPECT_EQ(callsign_buffer.guard, 'X');

	guarded_buffer_s uas_id_buffer{{}, 'X'};
	DaaEncodedId{0x0102030405u, detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID}.to_string(
		uas_id_buffer.output, sizeof(uas_id_buffer.output));
	EXPECT_STREQ(uas_id_buffer.output, "050");
	EXPECT_EQ(uas_id_buffer.guard, 'X');

	char one_byte_buffer{'X'};
	DaaEncodedId{callsign_key, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN}.to_string(&one_byte_buffer, 1);
	EXPECT_EQ(one_byte_buffer, '\0');
}

// Equality keys on both id and encoding.
TEST(DaaEncodedIdTest, EqualityComparesIdAndEncoding)
{
	const DaaEncodedId a{42u, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO};

	// Same id and encoding: equal.
	EXPECT_TRUE(a == (DaaEncodedId{42u, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO}));
	EXPECT_FALSE(a != (DaaEncodedId{42u, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO}));

	// Different id: not equal.
	EXPECT_FALSE(a == (DaaEncodedId{43u, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO}));

	// Same numeric id, different encoding: not equal (keeps colliding values in separate buffer slots).
	EXPECT_FALSE(a == (DaaEncodedId{42u, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN}));
	EXPECT_TRUE(a != (DaaEncodedId{42u, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN}));
}

// from_report() selects ICAO > callsign > UAS-ID, or id=0 when nothing usable is present.
TEST(DaaEncodedIdTest, FromReportSelectsEncodingByPriority)
{
	// ICAO present: selected, and it wins even when a valid callsign is also present.
	transponder_report_s report{};
	report.icao_address = 0x4F74EE;
	report.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;
	set_report_callsign(report, "ABC");

	DaaEncodedId id = DaaEncodedId::from_report(report);
	EXPECT_EQ(id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	EXPECT_EQ(id.id, 0x4F74EEu);

	// No ICAO, valid callsign flag: callsign selected.
	report.icao_address = 0;
	id = DaaEncodedId::from_report(report);
	EXPECT_EQ(id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
	EXPECT_EQ(id.id, DaaEncodedId::callsign_to_uint64("ABC"));

	// An out-of-range ICAO address is ignored in favor of the next valid identifier.
	report.icao_address = kMaxIcaoAddress + 1u;
	id = DaaEncodedId::from_report(report);
	EXPECT_EQ(id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
	EXPECT_EQ(id.id, DaaEncodedId::callsign_to_uint64("ABC"));

	// No ICAO, no callsign flag, but a non-zero UAS ID: UAS-ID selected.
	transponder_report_s uas_report{};
	uas_report.uas_id[kUasIdByteLength - 1] = 0xAB;
	id = DaaEncodedId::from_report(uas_report);
	EXPECT_EQ(id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID);
	EXPECT_EQ(id.id, DaaEncodedId::last_uas_id_bytes_to_uint64(uas_report.uas_id));

	// Nothing usable: default id.
	transponder_report_s empty_report{};
	EXPECT_EQ(DaaEncodedId::from_report(empty_report).id, 0u);
}

// ICAO self-detection matches the primary or secondary ownship ICAO; unset (negative) never matches.
TEST(DaaEncodedIdTest, SelfDetectionIcao)
{
	const DaaEncodedId traffic_id{0x123456, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO};

	daa_ownship_ids_s ownship_ids{};
	EXPECT_FALSE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids.icao = 0x123456;
	EXPECT_TRUE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids = {};
	ownship_ids.icao_2 = 0x123456;
	EXPECT_TRUE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids.icao = 0x654321;
	ownship_ids.icao_2 = 0x654322;
	EXPECT_FALSE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));
}
