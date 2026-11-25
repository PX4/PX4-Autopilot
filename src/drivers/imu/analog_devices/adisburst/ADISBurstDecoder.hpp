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
 * @file ADISBurstDecoder.hpp
 * @brief Byte-sum burst decoder for flat-register Analog Devices SPI IMUs.
 */

#pragma once

#include <lib/drivers/imu/ByteCursor.hpp>

namespace adis_burst_decoder
{

enum class Format : uint8_t {
	kBurst16,
	kBurst32,
	kTimestamp32,
};

constexpr size_t kCommandBytes    { 2 };
constexpr size_t kWordBytes       { 2 };
constexpr size_t kAxisCount       { 6 };
constexpr size_t kSize16          { 22 };
constexpr size_t kSize32          { 34 };
constexpr size_t kSizeTimestamp32 { 36 };

constexpr size_t size(Format format)
{
	return format == Format::kBurst16     ? kSize16
	       : format == Format::kBurst32     ? kSize32
	       : format == Format::kTimestamp32 ? kSizeTimestamp32
	       : 0;
}

/**
 * @brief Compute decimation filter group delay [us] for wide ADIS165xx burst modes.
 *
 * ADIS1650x uses (D + 1) / 2 * Ts when decimation D > 0; ADIS1657x uses D / 2 * Ts.
 */
constexpr uint32_t decimationDelayUs(Format format, uint16_t decimation, uint32_t sample_period_us = 500)
{
	if (decimation == 0 || format == Format::kBurst16) {
		return 0;
	}

	const uint32_t samples = format == Format::kBurst32
				 ? (static_cast<uint32_t>(decimation) + 1u)
				 : static_cast<uint32_t>(decimation);

	return samples * sample_period_us / 2u;
}

static_assert(decimationDelayUs(Format::kBurst16, 10) == 0, "16-bit burst must report zero decimation delay");
static_assert(decimationDelayUs(Format::kBurst32, 0) == 0, "ADIS1650x dec=0 must report zero decimation delay");
static_assert(decimationDelayUs(Format::kBurst32, 1) == 500, "ADIS1650x dec=1 must report 500 us");
static_assert(decimationDelayUs(Format::kTimestamp32, 0) == 0, "ADIS1657x dec=0 must report zero decimation delay");
static_assert(decimationDelayUs(Format::kTimestamp32, 199) == 49750, "ADIS1657x dec=199 must report 49750 us");

constexpr uint16_t kDiagOverrun             { 1u << 1 };
constexpr uint16_t kDiagSpiError            { 1u << 3 };
constexpr uint16_t kTransientDiagnosticMask { kDiagOverrun | kDiagSpiError };

/**
 * @brief Decide whether a device DIAG_STAT reading requires resetting the IMU.
 *
 * Non-transient fault bits trigger recovery immediately; transient SPI/overrun flags
 * tolerate isolated single-burst glitches and recover only when persistent.
 */
constexpr bool shouldRecoverOnDiagnostic(uint16_t diagnostic, uint8_t consecutive_transient_errors,
		uint8_t persistent_limit = 4)
{
	if (diagnostic == 0) {
		return false;
	}

	if ((diagnostic & ~kTransientDiagnosticMask) != 0) {
		return true;
	}

	return consecutive_transient_errors >= persistent_limit;
}

static_assert(!shouldRecoverOnDiagnostic(0, 10), "Zero diagnostic must not recover");
static_assert(shouldRecoverOnDiagnostic(1u << 5, 0), "Non-transient diagnostic must recover immediately");
static_assert(!shouldRecoverOnDiagnostic(kDiagOverrun, 1), "Single overrun must not trigger reset");
static_assert(!shouldRecoverOnDiagnostic(kTransientDiagnosticMask, 3), "Transient diagnostics below limit must not reset");
static_assert(shouldRecoverOnDiagnostic(kDiagOverrun, 4), "Persistent overrun at limit must trigger reset");

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
 * @brief Validate a complete byte-sum burst.
 * @param[in] data SPI response including command clocks, or null.
 * @param[in] length Received byte count, not buffer capacity.
 * @param[in] format Selected byte-sum burst layout.
 * @param[out] sample Validated channels; unchanged on failure.
 * @param[out] diagnostic Optional raw device status after checksum validation. ADIS1657x's
 * retained checksum excludes the status word; its status is not independently integrity-protected.
 * @return True only when length, diagnostic status and checksum all pass.
 * @tparam order Byte order in memory, independent of the SPI controller word width.
 */
template<imu::ByteOrder order = imu::ByteOrder::kBigEndian>
inline bool decode(const uint8_t *data, size_t length, Format format, Sample &sample, uint16_t *diagnostic = nullptr)
{
	const size_t expected_length = adis_burst_decoder::size(format);

	if (!data || expected_length < kSize16 || length != expected_length) {
		return false;
	}

	const size_t   checksum_offset   = length - kWordBytes;
	const size_t   checksum_start    = format == Format::kTimestamp32 ? kCommandBytes + kWordBytes : kCommandBytes;
	const uint16_t checksum          = imu::byteSum16(data + checksum_start, checksum_offset - checksum_start);
	const uint16_t received_checksum = unsignedWord<order>(data + checksum_offset);
	const uint16_t status            = unsignedWord<order>(data + kCommandBytes);

	// An all-zero bus response has status == 0 and checksum == 0; it is not a valid burst.
	// ADIS1657x (kTimestamp32) excludes DIAG_STAT from SPI_CHKSUM, so a faulted safe-mode frame
	// with status != 0 and zeroed sensor/counter outputs lawfully carries checksum == 0.
	const bool zero_bus = format == Format::kTimestamp32 ? (checksum == 0 && status == 0) : (checksum == 0);

	if (zero_bus || checksum != received_checksum) {
		return false;
	}

	if (diagnostic) {
		*diagnostic = status;
	}

	if (status != 0) {
		return false;
	}

	Sample decoded {};
	const bool     wide       = format != Format::kBurst16;
	const size_t   axis_bytes = wide ? 2 * kWordBytes : kWordBytes;
	const uint8_t *axes       = data + kCommandBytes + kWordBytes;

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

} // namespace adis_burst_decoder
