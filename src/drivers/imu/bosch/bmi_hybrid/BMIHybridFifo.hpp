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
 * @file BMIHybridFifo.hpp
 * @brief Fixed and tagged FIFO frame decoders for Bosch BMI055/085/088/270 SPI IMUs.
 */

#pragma once

#include <lib/drivers/imu/FifoFrames.hpp>

namespace bmi_hybrid_fifo
{
enum class Format : uint8_t {
	kFixed12Accel,
	kFixed16Gyro,
	kTaggedAccel,
	kTaggedImu,
};

using Frame = imu::FifoFrame;

constexpr size_t  kAxisBytes         { 6 };
constexpr uint8_t kEmptyFrameHeader  { 0x80 };
constexpr uint8_t kAccelHeader       { 0x84 };
constexpr uint8_t kGyroHeader        { 0x88 };
constexpr uint8_t kCombinedHeader    { 0x8c };
constexpr uint8_t kSkipHeader        { 0x40 };
constexpr uint8_t kSensorTimeHeader  { 0x44 };
constexpr uint8_t kInputConfigHeader { 0x48 };
constexpr uint8_t kSampleDropHeader  { 0x50 };

constexpr bool tagged(Format format)
{
	return format == Format::kTaggedAccel || format == Format::kTaggedImu;
}

constexpr uint8_t frameBytes(Format format)
{
	return format == Format::kTaggedImu     ? 13
	       : format == Format::kTaggedAccel ? 7
	       : 6;
}

using Sample = imu::FifoFrameSample;

/** Decode one fixed frame. Endianness and the 12-bit alignment are wire properties. */
template<unsigned shift>
inline void readAxes(const uint8_t *data, int16_t (&axes)[3])
{
	static_assert(
		shift == 0
		|| shift == 4,
		"Unsupported fixed sample alignment");

	imu::readAxes16<imu::ByteOrder::kLittleEndian, shift>(data, axes);
}

/**
 * @brief Bounded tagged parser. Only the channels named by the result may be used.
 * All payload lengths exclude the header. Control lengths are dialect-specific.
 * Unknown/truncated frames and in-batch sample drops must not be silently skipped.
 */
template<bool integrated>
inline Frame readTagged(imu::ByteCursor &cursor, Sample &sample, bool allow_truncated_tail = false)
{
	const uint8_t *header = cursor.take(1);

	if (!header) {
		return Frame::kEnd;
	}

	const Frame truncated = allow_truncated_tail ? Frame::kEnd : Frame::kInvalid;

	// Overread/empty frame is exact 0x80 0x00; 0x81..0x83 are invalid headers.
	if (*header == kEmptyFrameHeader) {
		const uint8_t *data = cursor.take(1);

		if (!data) {
			return truncated;
		}

		return data[0] == 0 ? Frame::kEnd : Frame::kInvalid;
	}

	// The low two header bits are interrupt tags, not part of the frame type.
	switch (*header & 0xfc) {
	case kAccelHeader: {
			const uint8_t *data = cursor.take(kAxisBytes);

			if (!data) {
				return truncated;
			}

			readAxes<0>(data, sample.accel);

			return Frame::kAccel;
		}

	case kGyroHeader: {
			if (!integrated) {
				return Frame::kInvalid;
			}

			const uint8_t *data = cursor.take(kAxisBytes);

			if (!data) {
				return truncated;
			}

			readAxes<0>(data, sample.gyro);

			return Frame::kGyro;
		}

	case kCombinedHeader: {
			if (!integrated) {
				return Frame::kInvalid;
			}

			const uint8_t *data = cursor.take(2 * kAxisBytes);

			if (!data) {
				return truncated;
			}

			// BMI270 sends gyro first, followed by accel.
			readAxes<0>(data, sample.gyro);
			readAxes<0>(data + kAxisBytes, sample.accel);

			return Frame::kBoth;
		}

	case kSkipHeader: { // Skip frame: one payload byte with skipped-frame count.
			const uint8_t *skipped = cursor.take(1);

			if (!skipped) {
				return truncated;
			}

			return skipped[0] == 0 ? Frame::kMetadata : Frame::kOverflow;
		}

	case kSensorTimeHeader: { // Sensor time: three payload bytes, not decoded as a sample.
			return cursor.take(3) ? Frame::kMetadata : truncated;
		}

	case kInputConfigHeader: { // Input-configuration frame: four payload bytes for BMI270, one for BMI08x accel.
			const uint8_t *data = cursor.take(integrated ? 4 : 1);

			if (!data) {
				return truncated;
			}

			if constexpr(integrated) {
				// BMI270 Byte 0 bits [3:0]: acc_conf_ch, acc_range_ch, gyr_conf_ch, gyr_range_ch.
				return (data[0] & 0x0f) ? Frame::kReconfigure : Frame::kMetadata;
			}

			return (data[0] & 0x03) ? Frame::kReconfigure : Frame::kMetadata;
		}

	case kSampleDropHeader: // Reserved opcode 4 on BMI270; in-batch dropped sample on BMI08x.
	default: {
			return Frame::kInvalid;
		}
	}
}

template<unsigned shift, typename Batch>
inline bool decodeFixed(const uint8_t *data, size_t bytes, bool reject_all_minimum, Batch &batch)
{
	const auto decode_sample = [reject_all_minimum](const uint8_t *packet, int16_t (&axes)[3]) {
		readAxes<shift>(packet, axes);

		return reject_all_minimum && axes[0] == INT16_MIN && axes[1] == INT16_MIN && axes[2] == INT16_MIN
		       ? imu::FifoSampleResult::kSkip : imu::FifoSampleResult::kAppend;
	};

	return imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ, true>(data, bytes, kAxisBytes, 0, 1, batch, decode_sample);
}

/** Sensor-specific temperature wire formats; false indicates an invalid sentinel. */
inline bool temperature08x(const uint8_t *data, float &temperature)
{
	const uint16_t raw = (uint16_t(data[0]) << 3) | (data[1] >> 5);

	if (raw == 1024) {
		return false;
	}

	const int16_t signed_raw = raw > 1023 ? int16_t(raw - 2048) : int16_t(raw);

	temperature = signed_raw * 0.125f + 23.f;

	return true;
}

inline bool temperature270(const uint8_t *data, float &temperature)
{
	const int16_t raw = imu::readLe16(data);

	if (raw == INT16_MIN) {
		return false;
	}

	temperature = raw / 512.f + 23.f;

	return true;
}

} // namespace bmi_hybrid_fifo
