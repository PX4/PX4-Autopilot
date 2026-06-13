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
 * @file DaaTrafficFilter.h
 *
 * Input gateway for DAA traffic reports: validates raw transponder data and
 * extracts a usable traffic identity, rejecting ownship self-detections.
 *
 * Ownship identifiers and the current time are passed in as plain values so
 * the filter stays free of parameter, board and clock dependencies.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h> // Provides CONFIG_NAVIGATOR_ADSB_F3442
#include <uORB/topics/transponder_report.h>

#include "DaaEncodedId.h"

// The ownship identifiers used to reject self-detections.
struct daa_ownship_ids_s {
	int32_t icao{-1};         // ADSB_ICAO_ID convention: negative = unset
	int32_t icao_2{-1};       // ADSB_ICAO_ID_2 convention: negative = unset
	uint64_t callsign{0};     // packed with DaaEncodedId::callsign_to_uint64
	uint64_t uas_id{0};       // packed with DaaEncodedId::last_uas_id_bytes_to_uint64
	bool uas_id_valid{false}; // false when the board UUID is unavailable
};

class DaaTrafficFilter
{
public:
	/** @brief True if the report has finite coords/altitude, the required flags, and a recent timestamp. */
	static bool transponder_data_valid(const transponder_report_s &report, const hrt_abstime now,
					   const hrt_abstime timeout_us);

	/** @brief Extract a usable traffic identifier and reject ownship reports. */
	static bool identify_traffic_report(const transponder_report_s &report, const daa_ownship_ids_s &ownship_ids,
					    DaaEncodedId &encoded_id);

	/** @brief True if the report's identifier matches ownship (ICAO, callsign or UAS-ID). */
	static bool is_self_detection(const DaaEncodedId &encoded_id, const daa_ownship_ids_s &ownship_ids);
};
