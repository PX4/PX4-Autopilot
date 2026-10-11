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

// Decoding a short callsign clears every byte after the terminator (MAVLink copies the whole field).
TEST(DaaEncodedIdTest, CallsignDecodeClearsUnusedBytes)
{
	const char callsign[kCallsignLength] {'A', 'B', '\0'};
	const uint64_t key = DaaEncodedId::callsign_to_uint64(callsign);

	char recovered[kCallsignLength];
	memset(recovered, 0x7F, sizeof(recovered));
	DaaEncodedId::convert_uint64_callsign_to_str(key, recovered);

	EXPECT_STREQ(recovered, "AB");

	for (int i = 2; i < kCallsignLength; ++i) {
		EXPECT_EQ(recovered[i], '\0');
	}
}

// ICAO formats as fixed-width uppercase hex of the low 24 bits; a too-small buffer is left untouched.
TEST(DaaEncodedIdTest, FormatsIcaoAsFixedWidthHex)
{
	char icao_buffer[kIcaoLength] {};

	// zero-padded + uppercased
	DaaEncodedId::convert_icao_uint32_to_hex_str(0xABCu, icao_buffer, sizeof(icao_buffer));
	EXPECT_STREQ(icao_buffer, "000ABC");

	// only the low 24 bits
	DaaEncodedId::convert_icao_uint32_to_hex_str(0x12ABCDEFu, icao_buffer, sizeof(icao_buffer));
	EXPECT_STREQ(icao_buffer, "ABCDEF");

	// too-small buffer untouched
	char small_buffer[kIcaoLength - 1];
	memset(small_buffer, 'X', sizeof(small_buffer));
	DaaEncodedId::convert_icao_uint32_to_hex_str(0xABCu, small_buffer, sizeof(small_buffer));
	EXPECT_EQ(small_buffer[0], 'X');
}

// Packed UAS-ID key preserves the exact GUID tail bytes (little-endian); all-zero packs to 0.
TEST(DaaEncodedIdTest, UasIdPackingPreservesTailBytes)
{
	uint8_t uas_id[kUasIdByteLength];

	for (int i = 0; i < 4; ++i) {
		for (int b = 0; b < kUasIdByteLength; ++b) {
			uas_id[b] = static_cast<uint8_t>(0x10 * i + b);
		}

		const uint64_t key = DaaEncodedId::last_uas_id_bytes_to_uint64(uas_id);

		// key byte k == GUID tail byte k
		for (int k = 0; k < kIdEncodingNbBytes; ++k) {
			const uint8_t key_byte = static_cast<uint8_t>((key >>(k * 8)) & 0xFF);
			EXPECT_EQ(uas_id[kUasIdByteLength - kIdEncodingNbBytes + k], key_byte);
		}
	}

	uint8_t zero_uas_id[kUasIdByteLength] {};
	EXPECT_EQ(DaaEncodedId::last_uas_id_bytes_to_uint64(zero_uas_id), 0u);
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

// Callsign self-detection requires an exact packed match.
TEST(DaaEncodedIdTest, SelfDetectionCallsign)
{
	const char callsign[kCallsignLength] = "TST1234";
	const uint64_t packed_callsign = DaaEncodedId::callsign_to_uint64(callsign);
	ASSERT_NE(packed_callsign, 0u);

	const DaaEncodedId traffic_id{packed_callsign, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN};

	daa_ownship_ids_s ownship_ids{};
	EXPECT_FALSE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids.callsign = packed_callsign;
	EXPECT_TRUE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids.callsign = packed_callsign ^ 1u;
	EXPECT_FALSE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));
}

// UAS-ID self-detection only when the board UUID is valid and the packed key matches.
TEST(DaaEncodedIdTest, SelfDetectionUasId)
{
	uint8_t uas_id_bytes[kUasIdByteLength];

	for (int i = 0; i < kUasIdByteLength; ++i) {
		uas_id_bytes[i] = 0xE0 + i;
	}

	const uint64_t packed_uas_id = DaaEncodedId::last_uas_id_bytes_to_uint64(uas_id_bytes);
	ASSERT_NE(packed_uas_id, 0u);

	const DaaEncodedId traffic_id{packed_uas_id, detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID};

	daa_ownship_ids_s ownship_ids{};
	ownship_ids.uas_id = packed_uas_id;
	EXPECT_FALSE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids.uas_id_valid = true;
	EXPECT_TRUE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));

	ownship_ids.uas_id = packed_uas_id ^ 1u;
	EXPECT_FALSE(DaaEncodedId::is_self_detection(traffic_id, ownship_ids));
}

// identify_traffic_report() rejects reports with no identity and ownship reports, decodes the rest.
TEST(DaaEncodedIdTest, IdentifyTrafficReport)
{
	daa_ownship_ids_s ownship_ids{};

	transponder_report_s report{};
	EXPECT_EQ(DaaEncodedId::identify_traffic_report(report, ownship_ids).id, 0u);

	report.icao_address = 0xABCDEF;
	const DaaEncodedId encoded_id = DaaEncodedId::identify_traffic_report(report, ownship_ids);
	EXPECT_EQ(encoded_id.id, 0xABCDEFu);
	EXPECT_EQ(encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);

	ownship_ids.icao = 0xABCDEF;
	EXPECT_EQ(DaaEncodedId::identify_traffic_report(report, ownship_ids).id, 0u);
}
