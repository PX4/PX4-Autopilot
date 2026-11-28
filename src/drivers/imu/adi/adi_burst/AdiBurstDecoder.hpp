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

namespace adi_burst_decoder
{
enum class Format : uint8_t {
	kBurst16,
	kBurst32,
	kTimestamp32,
	kCrc32,
};

constexpr size_t kCommandBytes    { 2 };
constexpr size_t kWordBytes       { 2 };
constexpr size_t kAxisCount       { 6 };
constexpr size_t kSize16          { 22 };
constexpr size_t kSize32          { 34 };
constexpr size_t kSizeTimestamp32 { 36 };
constexpr size_t kSizeCrc32       { 42 }; // Command plus the longest ADIS16497 response.

constexpr size_t size(Format format)
{
	return format == Format::kBurst16     ? kSize16
	       : format == Format::kBurst32     ? kSize32
	       : format == Format::kTimestamp32 ? kSizeTimestamp32
	       : format == Format::kCrc32       ? kSizeCrc32
	       : 0;
}

/** Validated raw single-sample channels; conversion and timestamps remain driver policy. */
struct Sample {
	int32_t  gyro[3]         {}; ///< Signed native gyro counts, including low words in wide formats.
	int32_t  accel[3]        {}; ///< Signed native accel counts, before coordinate conversion.
	int16_t  temperature     {}; ///< Signed sensor counts; the model owns scale and offset.
	uint16_t counter         {}; ///< Device sample counter, wrapping naturally.
	uint16_t timestamp_upper {}; ///< Optional ADIS1657x word; not used as a host timestamp.
};

template<imu::ByteOrder order = imu::ByteOrder::kBigEndian>
inline uint16_t unsignedWord(const uint8_t *data)
{
	return static_cast<uint16_t>(imu::readUnsigned<2, order>(data));
}

/** SPI words are big endian, but each 32-bit channel is sent low word first. */
template<imu::ByteOrder order = imu::ByteOrder::kBigEndian>
inline int32_t axis(const uint8_t *data, bool wide)
{
	if (!wide) {
		return static_cast<int16_t>(unsignedWord<order>(data));
	}

	return imu::readWordPair32<order, imu::WordOrder::kLowFirst>(data);
}

/**
 * @brief Decode the bounded, variable-prefix CRC32 response.
 * @param[in] data SPI response including command clocks, or null.
 * @param[in] length Received byte count, exactly 40 or 42.
 * @param[out] sample Validated channels; unchanged on failure.
 * @param[out] diagnostic Optional raw device status, written only after framing/CRC validation.
 * @return True only when framing, status and CRC all pass.
 * @tparam order Byte order in memory: wire bytes or native words from transferhword().
 */
template<imu::ByteOrder order = imu::ByteOrder::kBigEndian>
bool decodeCrc32(const uint8_t *data, size_t length, Sample &sample, uint16_t *diagnostic = nullptr);

/**
 * @brief Validate a complete byte-sum burst; CRC32 has a separately linked decoder.
 * @param[in] data SPI response including command clocks, or null.
 * @param[in] size Received byte count, not buffer capacity.
 * @param[in] format One of the byte-sum formats; CRC32 and unknown values are rejected.
 * @param[out] sample Validated channels; unchanged on failure.
 * @param[out] diagnostic Optional raw device status after checksum validation. ADIS1657x's
 * retained checksum excludes the status word; its status is not independently integrity-protected.
 * @return True only when length, diagnostic status and checksum all pass.
 * @tparam order Byte order in memory, independent of the SPI controller word width.
 */
template<imu::ByteOrder order = imu::ByteOrder::kBigEndian>
inline bool decode(const uint8_t *data, size_t size, Format format, Sample &sample, uint16_t *diagnostic = nullptr)
{
	if (!data
	    || size   <  kSize16
	    || format == Format::kCrc32
	    || size   != adi_burst_decoder::size(format)) {
		return false;
	}

	const size_t   checksum_offset = size - kWordBytes;
	const size_t   checksum_start  = format == Format::kTimestamp32 ? kCommandBytes + kWordBytes : kCommandBytes;
	const uint16_t checksum        = imu::byteSum16(data + checksum_start, checksum_offset - checksum_start);

	// An all-zero bus response also has a zero checksum; it is not a valid burst.
	if (checksum == 0
	    || checksum != unsignedWord<order>(data + checksum_offset)) {
		return false;
	}

	const uint16_t status = unsignedWord<order>(data + kCommandBytes);

	if (diagnostic) {
		*diagnostic = status;
	}

	if (status != 0) {
		return false;
	}

	Sample decoded {};
	const bool    wide       = format != Format::kBurst16;
	const size_t  axis_bytes = wide ? 2 * kWordBytes : kWordBytes;
	const uint8_t *axes      = data + kCommandBytes + kWordBytes;

	for (size_t i = 0; i < 3; ++i) {
		decoded.gyro[i]  = axis<order>(axes + i * axis_bytes, wide);
		decoded.accel[i] = axis<order>(axes + (i + 3) * axis_bytes, wide);
	}

	const uint8_t *tail = axes + kAxisCount * axis_bytes;

	decoded.temperature = static_cast<int16_t>(unsignedWord<order>(tail));
	decoded.counter     = unsignedWord<order>(tail + kWordBytes);

	if (format == Format::kTimestamp32) {
		decoded.timestamp_upper = unsignedWord<order>(tail + 2 * kWordBytes);
	}

	sample = decoded;

	return true;
}

} // namespace adi_burst_decoder
