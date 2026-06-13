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
 * @file DaaTrafficFilter.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "DaaTrafficFilter.h"

#include <px4_platform_common/log.h>

bool DaaTrafficFilter::transponder_data_valid(const transponder_report_s &report, const hrt_abstime now,
		const hrt_abstime timeout_us)
{
	if (!PX4_ISFINITE(report.lat) || !PX4_ISFINITE(report.lon)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid lat/lon.");
		return false;
	}

	if (!PX4_ISFINITE(report.altitude)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid altitude.");
		return false;
	}

	uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
				  transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442
	required_flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	if ((report.flags & required_flags) != required_flags) {
		PX4_DEBUG("DAA: transponder data rejected, missing flags.");
		return false;
	}

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442

	if (!PX4_ISFINITE(report.heading)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid heading.");
		return false;
	}

#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	if (report.timestamp == 0 || (now - report.timestamp) > timeout_us) {
		PX4_DEBUG("DAA: transponder data rejected, too old.");
		return false;
	}

	return true;
}

bool DaaTrafficFilter::identify_traffic_report(const transponder_report_s &report,
		const daa_ownship_ids_s &ownship_ids, DaaEncodedId &encoded_id)
{
	encoded_id = DaaEncodedId::from_report(report);

	if (encoded_id.id == 0) {
		PX4_DEBUG("DAA: No valid unique ID, skipping report");
		return false;
	}

	if (is_self_detection(encoded_id, ownship_ids)) {
		PX4_DEBUG("DAA: Self detection, skipping report.");
		return false;
	}

#if defined(DEBUG_BUILD)
	char encoded_id_str[kUtmGuidMsgLength];
	encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));
	PX4_DEBUG("DAA: unique ID: %s (int:%lu)", encoded_id_str, encoded_id.id);
#endif

	return true;
}

bool DaaTrafficFilter::is_self_detection(const DaaEncodedId &encoded_id, const daa_ownship_ids_s &ownship_ids)
{
	switch (encoded_id.encoding) {
	case detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO: {
			if (ownship_ids.icao >= 0
			    && static_cast<uint32_t>(encoded_id.id) == static_cast<uint32_t>(ownship_ids.icao)) {
				PX4_DEBUG("DAA: Received own main ICAO.");
				return true;
			}

			if (ownship_ids.icao_2 >= 0
			    && static_cast<uint32_t>(encoded_id.id) == static_cast<uint32_t>(ownship_ids.icao_2)) {
				PX4_DEBUG("DAA: Received own secondary ICAO.");
				return true;
			}

			break;
		}

	case detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN: {
			if (encoded_id.id == ownship_ids.callsign) {
				PX4_DEBUG("DAA: Received own Callsign.");
				return true;
			}

			break;
		}

	case detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID: {
			if (ownship_ids.uas_id_valid && encoded_id.id == ownship_ids.uas_id) {
				PX4_DEBUG("DAA: Received own UAS ID.");
				return true;
			}

			break;
		}

	default:
		break;
	}

	return false;
}
