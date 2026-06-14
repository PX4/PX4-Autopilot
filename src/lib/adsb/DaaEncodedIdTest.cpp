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

// WHY: Callsign packing and unpacking must round-trip cleanly so buffer keys and operator-facing messages stay consistent.
// WHAT: Convert representative callsigns to the 64-bit key and back, verify the recovered string re-encodes identically, and reject unterminated input.
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

		// THEN: Re-encoding the recovered string yields the same key.
		EXPECT_EQ(key, DaaEncodedId::callsign_to_uint64(recovered));
	}

	// WHEN/THEN: A callsign that is not null-terminated is rejected.
	char callsign_non_null[kCallsignLength];
	memset(callsign_non_null, 'A', sizeof(callsign_non_null));
	EXPECT_EQ(DaaEncodedId::callsign_to_uint64(callsign_non_null), 0u);
}

// WHY: Callsigns are stored as bytes, so high-bit characters must only affect their own byte slot and never sign-extend across the 64-bit key.
// WHAT: Encode a callsign containing bytes with the MSB set and verify the key preserves only those byte values.
TEST(DaaEncodedIdTest, CallsignPackingDoesNotSignExtend)
{
	const char callsign[kCallsignLength] {static_cast<char>(0x80), static_cast<char>(0xFF), '\0'};

	EXPECT_EQ(DaaEncodedId::callsign_to_uint64(callsign), static_cast<uint64_t>(0xFF80u));
}

// WHY: MAVLink fixed-length string fields copy every byte, so unused callsign bytes must be deterministic after decoding.
// WHAT: Decode a short callsign into a pre-filled buffer and verify all bytes after the terminator are cleared.
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

// WHY: ICAO identifiers are 24-bit values, so operator-facing formatting must stay fixed-width and avoid platform-specific truncation.
// WHAT: Format representative ICAO values and verify the output is uppercase, zero-padded, derived from the low 24 bits, and that a too-small buffer is left untouched.
TEST(DaaEncodedIdTest, FormatsIcaoAsFixedWidthHex)
{
	char icao_buffer[kIcaoLength] {};

	// WHEN: A short ICAO value is formatted. THEN: It is zero-padded and uppercased.
	DaaEncodedId::convert_icao_uint32_to_hex_str(0xABCu, icao_buffer, sizeof(icao_buffer));
	EXPECT_STREQ(icao_buffer, "000ABC");

	// WHEN: High bits are present. THEN: Only the low 24 ICAO bits are rendered.
	DaaEncodedId::convert_icao_uint32_to_hex_str(0x12ABCDEFu, icao_buffer, sizeof(icao_buffer));
	EXPECT_STREQ(icao_buffer, "ABCDEF");

	// WHEN: The buffer is too small. THEN: It is left untouched.
	char small_buffer[kIcaoLength - 1];
	memset(small_buffer, 'X', sizeof(small_buffer));
	DaaEncodedId::convert_icao_uint32_to_hex_str(0xABCu, small_buffer, sizeof(small_buffer));
	EXPECT_EQ(small_buffer[0], 'X');
}

// WHY: The reduced UAS-ID key must preserve the exact GUID tail bytes used as the unique identifier.
// WHAT: Pack random and zero-valued GUID tails and verify each key byte matches the source tail byte.
TEST(DaaEncodedIdTest, UasIdPackingPreservesTailBytes)
{
	uint8_t uas_id[kUasIdByteLength];

	for (int i = 0; i < 4; ++i) {
		for (int b = 0; b < kUasIdByteLength; ++b) {
			uas_id[b] = static_cast<uint8_t>(0x10 * i + b);
		}

		const uint64_t key = DaaEncodedId::last_uas_id_bytes_to_uint64(uas_id);

		// THEN: Key byte k (little-endian) equals the k-th byte of the GUID tail.
		for (int k = 0; k < kIdEncodingNbBytes; ++k) {
			const uint8_t key_byte = static_cast<uint8_t>((key >>(k * 8)) & 0xFF);
			EXPECT_EQ(uas_id[kUasIdByteLength - kIdEncodingNbBytes + k], key_byte);
		}
	}

	// WHEN/THEN: An all-zero UAS ID packs to a zero key.
	uint8_t zero_uas_id[kUasIdByteLength] {};
	EXPECT_EQ(DaaEncodedId::last_uas_id_bytes_to_uint64(zero_uas_id), 0u);
}

// WHY: Messages for operators use to_string().
// WHAT: Convert one id per encoding plus an unknown encoding and verify the formatted string.
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

	// UAS ID -> reduced lowercase hex of the packed tail bytes (little-endian).
	DaaEncodedId{0x0102030405u, detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID}.to_string(buffer, sizeof(buffer));
	EXPECT_STREQ(buffer, "0504030201");

	// Unknown encoding -> explicit placeholder.
	char unknown_buffer[32];
	DaaEncodedId{42u, 0xFF}.to_string(unknown_buffer, sizeof(unknown_buffer));
	EXPECT_STREQ(unknown_buffer, "Unknown ID.");
}

// WHY: The traffic buffer keys conflicts on {id, encoding}, so equality must consider both fields.
// WHAT: Compare ids that differ only by value or only by encoding.
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

// WHY: A report can carry several identifiers; the buffer must key on a single, deterministic choice.
// WHAT: Verify from_report() selects ICAO > callsign > UAS-ID and returns id=0 when nothing usable is present.
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
