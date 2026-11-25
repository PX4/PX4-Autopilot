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
 * @file FifoFrames.hpp
 * @brief Bounded framed FIFO traversal with independent native accel/gyro batches.
 */

#pragma once

#include "FifoBatch.hpp"

namespace imu
{

enum class FifoFrame : uint8_t {
	kAccel,
	kGyro,
	kBoth,
	kMetadata,
	kEnd,
	kOverflow,
	kInvalid,
	kReconfigure,
};

enum class FifoDecodeStatus : uint8_t {
	kComplete,
	kEmpty,
	kOverflow,
	kInvalid,
	kReconfigure,
};

enum class FifoChannels : uint8_t {
	kAccel,
	kGyro,
	kBoth,
};

struct FifoFrameSample {
	int16_t accel[3] {};
	int16_t gyro[3] {};
};

struct FifoDecodeResult {
	FifoDecodeStatus status         { FifoDecodeStatus::kEmpty };
	bool             has_metadata   { false };
	size_t           consumed_bytes { 0 };
};

template<FifoAxisMapping mapping,
	 FifoChannels channels = FifoChannels::kBoth,
	 FifoAxisMapping gyro_mapping = mapping,
	 typename AccelBatch,
	 typename GyroBatch,
	 typename Sample,
	 typename Decode>
[[nodiscard]] FifoDecodeResult decodeFifoFrames(
	const uint8_t *data,
	size_t bytes,
	AccelBatch &accel,
	GyroBatch &gyro,
	Sample &sample,
	Decode decode)
{
	static_assert(
		channels == FifoChannels::kAccel
		|| channels == FifoChannels::kGyro
		|| channels == FifoChannels::kBoth,
		"Invalid FIFO channel selection");
	static_assert(
		fifoBatchCapacity<AccelBatch>() > 0
		&& fifoBatchCapacity<GyroBatch>() > 0,
		"Invalid FIFO batches");

	FifoDecodeResult result;

	if ((!data && bytes != 0) || accel.samples != 0 || gyro.samples != 0) {
		result.status = FifoDecodeStatus::kInvalid;

		return result;
	}

	ByteCursor cursor(data, bytes);

	while (cursor.remaining()) {
		const size_t    before = cursor.remaining();
		const FifoFrame frame  = decode(cursor, sample);

		if (frame == FifoFrame::kReconfigure) {
			accel.samples         = 0;
			gyro.samples          = 0;
			result.has_metadata   = false;
			result.consumed_bytes = 0;
			result.status         = FifoDecodeStatus::kReconfigure;

			return result;
		}

		if (frame == FifoFrame::kOverflow) {
			accel.samples         = 0;
			gyro.samples          = 0;
			result.has_metadata   = false;
			result.consumed_bytes = 0;
			result.status         = FifoDecodeStatus::kOverflow;

			return result;
		}

		if (frame == FifoFrame::kEnd) {
			break;
		}

		if (frame == FifoFrame::kInvalid || cursor.remaining() >= before) {
			accel.samples         = 0;
			gyro.samples          = 0;
			result.has_metadata   = false;
			result.consumed_bytes = 0;
			result.status         = FifoDecodeStatus::kInvalid;

			return result;
		}

		if (frame == FifoFrame::kMetadata) {
			result.has_metadata   = true;
			result.consumed_bytes = bytes - cursor.remaining();
			continue;
		}

		bool valid = true;

		switch (frame) {
		case FifoFrame::kBoth: {
				if constexpr(channels != FifoChannels::kBoth) {
					valid = false;
					break;
				}

				if (accel.samples >= fifoBatchCapacity<AccelBatch>() || gyro.samples >= fifoBatchCapacity<GyroBatch>()) {
					valid = false;
					break;
				}

				valid = appendFifoSample<gyro_mapping>(gyro, sample.gyro)
					&& appendFifoSample<mapping>(accel, sample.accel);
				break;
			}

		case FifoFrame::kAccel: {
				if constexpr(channels == FifoChannels::kGyro) {
					valid = false;
					break;
				}

				valid = appendFifoSample<mapping>(accel, sample.accel);
				break;
			}

		case FifoFrame::kGyro: {
				if constexpr(channels == FifoChannels::kAccel) {
					valid = false;
					break;
				}

				valid = appendFifoSample<gyro_mapping>(gyro, sample.gyro);
				break;
			}

		default: {
				valid = false;
				break;
			}
		}

		if (!valid) {
			accel.samples         = 0;
			gyro.samples          = 0;
			result.has_metadata   = false;
			result.consumed_bytes = 0;
			result.status         = FifoDecodeStatus::kInvalid;

			return result;
		}

		result.consumed_bytes = bytes - cursor.remaining();
	}

	result.status = accel.samples || gyro.samples ? FifoDecodeStatus::kComplete : FifoDecodeStatus::kEmpty;

	return result;
}

} // namespace imu
