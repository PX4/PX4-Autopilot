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

#pragma once

#include <cstdint>

// Convert a DroneCAN Fix/Fix2 timestamp to UTC [us]. Zero denotes unknown or invalid time.
template<typename FixType>
uint64_t gnssTimeToUtc(const FixType &msg)
{
	const uint64_t timestamp_usec = msg.gnss_timestamp.usec;
	int32_t offset_seconds = 0;

	switch (msg.gnss_time_standard) {
	case FixType::GNSS_TIME_STANDARD_UTC:
		return timestamp_usec;

	case FixType::GNSS_TIME_STANDARD_GPS:
		// Fix2 DSDL: UTC = GPS - num_leap_seconds + 9 [s].
		offset_seconds = static_cast<int32_t>(msg.num_leap_seconds) - 9;
		break;

	case FixType::GNSS_TIME_STANDARD_TAI:
		// Fix2 DSDL: UTC = TAI - num_leap_seconds - 10 [s].
		offset_seconds = static_cast<int32_t>(msg.num_leap_seconds) + 10;
		break;

	default:
		return 0;
	}

	if (timestamp_usec == 0 || msg.num_leap_seconds == FixType::NUM_LEAP_SECONDS_UNKNOWN) {
		return 0;
	}

	const int64_t offset_usec = static_cast<int64_t>(offset_seconds) * 1'000'000;

	if (offset_usec >= 0) {
		const uint64_t correction_usec = static_cast<uint64_t>(offset_usec);
		return timestamp_usec >= correction_usec ? timestamp_usec - correction_usec : 0;
	}

	const uint64_t correction_usec = static_cast<uint64_t>(-offset_usec);
	return timestamp_usec <= UINT64_MAX - correction_usec ? timestamp_usec + correction_usec : 0;
}
