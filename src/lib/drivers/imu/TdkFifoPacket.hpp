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
 * @file TdkFifoPacket.hpp
 * @brief Shared 16/20-byte headered FIFO packet decoder for TDK banked and IREG IMUs.
 */

#pragma once

#include "ByteCursor.hpp"

namespace imu
{

constexpr size_t  kTdkHiresPacketSize = 20;
constexpr uint8_t kTdkHiresHeader     = 0x78; // Accel, gyro, high resolution, ODR timestamp.

struct TdkFifoSample {
	int16_t  accel[3]    {};     ///< Raw signed accelerometer axes [LSB].
	int16_t  gyro[3]     {};     ///< Raw signed gyroscope axes [LSB].
	int16_t  temperature {};     ///< Raw packet temperature [LSB].
	uint16_t timestamp   {};     ///< Raw packet FIFO timestamp [ticks].
	bool     accel_valid { true };
	bool     gyro_valid  { true };
};

inline bool decodeTdkFifoPacket(
	const uint8_t *packet,
	size_t size,
	bool little_endian,
	bool require_timestamp_header,
	bool require_nonzero_timestamp,
	bool has_invalid_sentinel,
	TdkFifoSample &sample)
{
	if (!packet || (size != 16 && size != kTdkHiresPacketSize)) {
		return false;
	}

	const bool    high_resolution = size == kTdkHiresPacketSize;
	const uint8_t expected        = high_resolution ? kTdkHiresHeader : 0x68;
	const uint8_t mask            = (little_endian && (require_timestamp_header || high_resolution)) ? 0xfc
					: ((require_timestamp_header || high_resolution) ? 0xff : 0xe0);

	if ((packet[0] & mask) != (expected & mask)) {
		return false;
	}

	const size_t msb = little_endian ? 1 : 0;
	const size_t lsb = little_endian ? 0 : 1;
	const auto word = [msb, lsb](const uint8_t *data) {
		return static_cast<int16_t>((static_cast<uint16_t>(data[msb]) << 8) | data[lsb]);
	};

	bool accel_valid = true;
	bool gyro_valid  = true;

	for (size_t axis = 0; axis < 3; ++axis) {
		const int16_t accel = word(packet + 1 + 2 * axis);
		const int16_t gyro  = word(packet + 7 + 2 * axis);

		if (has_invalid_sentinel) {
			const uint8_t ext = high_resolution ? packet[17 + axis] : 0;

			// In 20-bit packets only 0x80000 (word == INT16_MIN && ext nibble == 0) is the invalid sentinel;
			// 0x80001..0x8000f are genuine negative full-scale saturation codes.
			if (accel == INT16_MIN && (ext & 0xf0) == 0) {
				accel_valid = false;
			}

			if (gyro == INT16_MIN && (ext & 0x0f) == 0) {
				gyro_valid = false;
			}
		}

		sample.accel[axis] = accel;
		sample.gyro[axis]  = gyro;
	}

	sample.accel_valid = accel_valid;
	sample.gyro_valid  = gyro_valid;
	sample.temperature = high_resolution ? word(packet + 13) : 0;
	sample.timestamp   = static_cast<uint16_t>(word(packet + (high_resolution ? 15 : 14)));

	return !require_nonzero_timestamp || sample.timestamp != 0;
}

} // namespace imu
