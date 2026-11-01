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

#include "../../common/FifoFrames.hpp"

namespace bosch_direct_fifo
{
enum class Format : uint8_t {
	kFixed12Accel,
	kFixed16Gyro,
	kTaggedAccel,
	kTaggedImu,
};

using Frame = imu::FifoFrame;

constexpr size_t  kAxisBytes      { 6 };
constexpr uint8_t kAccelHeader    { 0x84 };
constexpr uint8_t kGyroHeader     { 0x88 };
constexpr uint8_t kCombinedHeader { 0x8c };

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

using Sample = imu::FifoFrameSample<>;

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
 * Unknown/truncated frames and sample gaps must not be silently skipped.
 */
template<bool integrated>
inline Frame readTagged(imu::ByteCursor &cursor, Sample &sample)
{
	const uint8_t *header = cursor.take(1);

	if (!header) {
		return Frame::kEnd;
	}

	// The low two header bits are interrupt tags, not part of the frame type.
	switch (*header & 0xfc) {
	case kAccelHeader: {
			const uint8_t *data = cursor.take(kAxisBytes);

			if (!data) {
				return Frame::kInvalid;
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
				return Frame::kInvalid;
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
				return Frame::kInvalid;
			}

			// BMI270 sends gyro first, followed by accel.
			readAxes<0>(data, sample.gyro);
			readAxes<0>(data + kAxisBytes, sample.accel);

			return Frame::kBoth;
		}

	case 0x44: { // Sensor time: three payload bytes, not decoded as a sample.
			return cursor.take(3) ? Frame::kMetadata : Frame::kInvalid;
		}

	case 0x48: { // Input-configuration frame: four payload bytes for BMI270, one for BMI08x accel.
			const uint8_t *data = cursor.take(integrated ? 4 : 1);

			if (!data) {
				return Frame::kInvalid;
			}

			// Enabling the sensor/FIFO may emit a zero change mask. Changes to
			// the running format/rate/range require restoring the configuration.
			// Check six change flags for an integrated endpoint, or the low two for accel only.
			return (data[0] & (integrated ? 0x3f : 0x03)) ? Frame::kReconfigure : Frame::kMetadata;
		}

	case 0x80: {
			const uint8_t *data = cursor.take(1);

			return data && data[0] == 0 ? Frame::kEnd : Frame::kInvalid;
		}

	case 0x40: // Skip frame: uniform dt no longer holds.
	case 0x50: // Dropped sample.
	default: {
			return Frame::kInvalid;
		}
	}
}

template<unsigned shift, typename Batch>
inline bool decodeFixed(const uint8_t *data, size_t bytes, bool reject_all_minimum, Batch &batch)
{
	const auto decode = [reject_all_minimum](const uint8_t *packet, int16_t (&axes)[3]) {
		readAxes<shift>(packet, axes);

		return reject_all_minimum && axes[0] == INT16_MIN && axes[1] == INT16_MIN && axes[2] == INT16_MIN
		       ? imu::FifoSampleResult::kInvalid : imu::FifoSampleResult::kAppend;
	};

	return imu::decodeFixedFifo<imu::FifoAxisMapping::kFlipYZ>(data, bytes, kAxisBytes, 0, 1, batch, decode);
}

/** Sensor-specific temperature wire formats; false leaves output unchanged. */
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

} // namespace bosch_direct_fifo
