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
 * @file DaaEncodedId.h
 * @brief Traffic report identification helpers and encoded traffic IDs.
 *
 * A transponder report can identify an aircraft three different ways: an ICAO
 * address, an ADS-B callsign, or a UAS / UTM GUID. The DAA traffic buffer keys
 * conflicts on a single 64-bit value, so each of those representations is packed
 * into a uint64_t and the encoding that produced it.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <uORB/topics/detect_and_avoid.h>	// UNIQUE_ID_ENCODING_*
#include <uORB/topics/transponder_report.h>	// from_report()

// Number of trailing identifier bytes packed into the 64-bit key. Max 8 (fits a uint64_t).
static constexpr uint8_t kIdEncodingNbBytes{8};
// Chars used to display a UAS id: 10 hex + null terminator. Max: 17 = 16 + null.
static constexpr uint8_t kUtmGuidMsgLength{11};
// ADS-B callsign string length: 8 chars + null terminator.
static constexpr uint8_t kCallsignLength{9};
// ICAO address string length: 6 hex chars + null terminator.
static constexpr uint8_t kIcaoLength{7};
static constexpr uint32_t kMaxIcaoAddress{0xFFFFFFu};
// Length of a UAS ID (UTM GUID) byte array.
// Similar to PX4_GUID_BYTE_LENGTH and transponder_report_s::uas_id;
static constexpr uint8_t kUasIdByteLength{18};

// The ownship identifiers used to reject self-detections.
struct daa_ownship_ids_s {
	int32_t icao{-1};         // ADSB_ICAO_ID convention: negative = unset
	int32_t icao_2{-1};       // ADSB_ICAO_ID_2 convention: negative = unset
	uint64_t callsign{0};     // packed with DaaEncodedId::callsign_to_uint64
	uint64_t uas_id{0};       // packed with DaaEncodedId::last_uas_id_bytes_to_uint64
	bool uas_id_valid{false}; // false when the board UUID is unavailable
};

/**
 * @brief A traffic identifier reduced to a 64-bit key plus its source encoding.
 *
 * @c encoding is one of detect_and_avoid_s::UNIQUE_ID_ENCODING_* and tells the
 * codecs and to_string() how to interpret @c id.
 */
struct DaaEncodedId {
	uint64_t id{0};
	uint8_t encoding{detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO};

	bool operator==(const DaaEncodedId &other) const { return encoding == other.encoding && id == other.id; }
	bool operator!=(const DaaEncodedId &other) const { return !(*this == other); }

	/**
	 * @brief Pick the best available identifier from a transponder report.
	 *
	 * Priority is a valid 24-bit ICAO address, ADS-B callsign, then UAS-ID.
	 * Returns the default value when the report has no usable identifier.
	 */
	static DaaEncodedId from_report(const transponder_report_s &report);

	/**
	 * @brief Identify a transponder traffic report.
	 *
	 * Returns the extracted identifier, or id=0 (the default-constructed value) when the report
	 * carries no usable identifier or matches ownship (self-detection).
	 */
	static DaaEncodedId identify_traffic_report(const transponder_report_s &report, const daa_ownship_ids_s &ownship_ids);

	// True if the report's identifier matches ownship (ICAO, callsign or UAS-ID).
	static bool is_self_detection(const DaaEncodedId &encoded_id, const daa_ownship_ids_s &ownship_ids);

	// Render to a null-terminated string per its encoding.
	void to_string(char *buffer, size_t buffer_size) const;

	// Pack a callsign into a 64-bit key. Returns 0 if the input is not null-terminated.
	static uint64_t callsign_to_uint64(const char callsign[kCallsignLength]);

	/**
	 * @brief Convert ADSB_CALLSIGN_1/2 parameter values to the packed callsign key.
	 *
	 * Each parameter stores four characters in display order (most-significant byte
	 * first), while callsign_to_uint64() stores the first character in the least-significant
	 * byte. This conversion keeps ownship parameter IDs in the same representation as
	 * callsigns received in transponder_report.
	 */
	static uint64_t callsign_params_to_uint64(int32_t callsign_part1, int32_t callsign_part2);

	// Inverse of callsign_to_uint64.
	static void convert_uint64_callsign_to_str(uint64_t value, char callsign[kCallsignLength]);

	// Pack the trailing bytes of a UAS-ID (UTM GUID) into a 64-bit key.
	static uint64_t last_uas_id_bytes_to_uint64(const uint8_t uas_id[kUasIdByteLength]);

	// Render a packed UAS-ID key as a reduced lowercase hex string.
	static void convert_uas_id_uint64_to_str(uint64_t uas_id_int, char uas_id_char_arr[kUtmGuidMsgLength]);

	// Print the low 24 bits of an ICAO address as a 6-digit uppercase hex string.
	static void convert_icao_uint32_to_hex_str(uint64_t value, char *buffer, size_t buffer_size);
};
