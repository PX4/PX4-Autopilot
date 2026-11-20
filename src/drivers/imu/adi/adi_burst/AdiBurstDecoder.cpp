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

#include "AdiBurstDecoder.hpp"

#if defined(__PX4_NUTTX)
#include <nuttx/crc32.h>
#else
#include <crc32.h>
#endif // __PX4_NUTTX

namespace adi_burst_decoder
{

template<imu::ByteOrder order>
bool decodeCrc32(const uint8_t *data, size_t length, Sample &sample, uint16_t *diagnostic)
{
	constexpr uint16_t kBurstId      { 0xa5a5 };
	constexpr size_t   kPayloadBytes { 30 }; // SYS_E_FLAG through DATA_CNT: fifteen words.
	constexpr size_t   kSlowSize     { 40 };

	if (!data || (length != kSlowSize && length != kSizeCrc32)
	    || unsignedWord<order>(data + 2) != 0
	    || unsignedWord<order>(data + 4) != kBurstId) {
		return false;
	}

	// ADIS16497 Rev. D, Tables 10/11: a second BURST_ID is emitted above
	// 3.6 MHz. Clock dividers can select the slower layout even when 5 MHz
	// is requested. Follow the ID-to-status transition, not the requested rate.
	const size_t status_offset = unsignedWord<order>(data + 6) == kBurstId ? 8 : 6;

	if (status_offset + kPayloadBytes + 4 > length) {
		return false;
	}

	// The CRC consumes low byte first regardless of the controller's memory layout.
	// Use the platform's table-based CRC instead of another private 1 KiB table.
	uint8_t crc_bytes[kPayloadBytes];

	for (size_t i = 0; i < kPayloadBytes; ++i) {
		const size_t offset = order == imu::ByteOrder::kBigEndian ? (i ^ 1u) : i;

		crc_bytes[i] = data[status_offset + offset];
	}

	const uint32_t expected =
		static_cast<uint32_t>(
			imu::readWordPair32<order, imu::WordOrder::kLowFirst>(
				data + status_offset + kPayloadBytes));

	if ((crc32part(crc_bytes, sizeof(crc_bytes), UINT32_MAX) ^ UINT32_MAX) != expected) {
		return false;
	}

	const uint16_t status = unsignedWord<order>(data + status_offset);

	if (diagnostic) {
		*diagnostic = status;
	}

	if (status != 0) {
		return false;
	}

	Sample decoded {};

	decoded.temperature = static_cast<int16_t>(unsignedWord<order>(data + status_offset + 2));

	const uint8_t *axes = data + status_offset + 4;

	for (size_t i = 0; i < 3; ++i) {
		decoded.gyro[i]  = imu::readWordPair32<order, imu::WordOrder::kLowFirst>(axes + i * 4);
		decoded.accel[i] = imu::readWordPair32<order, imu::WordOrder::kLowFirst>(axes + (i + 3) * 4);
	}

	decoded.counter = unsignedWord<order>(data + status_offset + kPayloadBytes - 2);
	sample          = decoded;

	return true;
}

template bool decodeCrc32<imu::ByteOrder::kBigEndian>(const uint8_t *, size_t, Sample &, uint16_t *);
template bool decodeCrc32<imu::ByteOrder::kLittleEndian>(const uint8_t *, size_t, Sample &, uint16_t *);

} // namespace adi_burst_decoder
