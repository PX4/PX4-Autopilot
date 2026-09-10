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

#include "../../common/ByteCursor.hpp"

namespace tdk_icm42x_fifo
{
constexpr size_t  kPacketSize = 20;
constexpr uint8_t kHeader     = 0x78; // Accel, gyro, high resolution, ODR timestamp.

struct Sample {
	int16_t  accel[3];
	int16_t  gyro[3];
	int16_t  temperature;
	uint16_t timestamp;
};

/** Validate and decode each axis once. Discard the output on failure. */
inline bool decodePacket(
	const uint8_t *packet,
	size_t size,
	bool little_endian,
	bool require_timestamp_header,
	bool external_clock,
	Sample &sample)
{
	if (!packet || (size != 16 && size != kPacketSize)) {
		return false;
	}

	// 0x68 describes the standard accel/gyro packet; 0xf3 ignores optional timestamp/FSYNC header bits.
	const bool    high_resolution = size == kPacketSize;
	const uint8_t expected        = high_resolution ? kHeader : 0x68;
	const uint8_t mask            = require_timestamp_header || high_resolution ? 0xff : 0xf3;

	if ((packet[0] & mask) != (expected & mask)) {
		return false;
	}

	// Resolve byte order once per packet, outside the axis loop.
	const size_t msb = little_endian ? 1 : 0;
	const size_t lsb = little_endian ? 0 : 1;
	const auto word = [msb, lsb](const uint8_t *data) {
		return static_cast<int16_t>((static_cast<uint16_t>(data[msb]) << 8) | data[lsb]);
	};

	for (size_t axis = 0; axis < 3; ++axis) {
		const int16_t accel = word(packet + 1 + 2 * axis);
		const int16_t gyro  = word(packet + 7 + 2 * axis);

		if (accel == INT16_MIN || gyro == INT16_MIN) {
			return false;
		}

		sample.accel[axis] = accel;
		sample.gyro[axis]  = gyro;
	}

	sample.temperature = high_resolution ? word(packet + 13) : 0;
	sample.timestamp   = static_cast<uint16_t>(word(packet + (high_resolution ? 15 : 14)));

	return !(high_resolution || external_clock) || sample.timestamp != 0;
}

inline bool validPacket(
	const uint8_t *packet,
	size_t size,
	bool little_endian,
	bool require_timestamp_header,
	bool external_clock)
{
	Sample sample;

	return decodePacket(packet, size, little_endian, require_timestamp_header, external_clock, sample);
}

inline bool validPacket(const uint8_t *packet, size_t size)
{
	return size == kPacketSize && validPacket(packet, size, false, true, false);
}

} // namespace tdk_icm42x_fifo
