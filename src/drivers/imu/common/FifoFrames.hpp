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
 * @file
 * @brief Bounded framed FIFO traversal with independent native accel/gyro batches.
 */

#pragma once

#include "FifoBatch.hpp"

namespace imu
{
/** Family-decoded frame category, independent of its wire header encoding. */
enum class FifoFrame : uint8_t {
	kAccel,
	kGyro,
	kBoth,
	kMetadata,
	kEnd,
	kInvalid,
	kReconfigure,
};

/** Outcome after wire traversal, before any additional family validation or publication. */
enum class FifoDecodeStatus : uint8_t {
	kComplete,    ///< At least one sample accepted; traversal exhausted the input or reached an explicit end marker.
	kEmpty,       ///< No samples accepted; metadata alone or an end marker is allowed.
	kInvalid,     ///< Invalid arguments, wire data, channel, capacity or decoder progress; discard partial output.
	kReconfigure, ///< Decoder requested family recovery; discard partial output before reconfiguration.
};

/** Channels permitted by an endpoint, not its reserved scratch-buffer layout. */
enum class FifoChannels : uint8_t {
	kAccel,
	kGyro,
	kBoth,
};

/** Raw counts from the latest frame; only axes selected by its FifoFrame value are valid. */
template<typename AccelRaw = int16_t, typename GyroRaw = AccelRaw>
struct FifoFrameSample {
	static_assert(detail::RawTraits<AccelRaw>::kSupported, "Unsupported accel frame storage");
	static_assert(detail::RawTraits<GyroRaw>::kSupported, "Unsupported gyro frame storage");

	AccelRaw accel[3]; ///< Initialized only for an Accel or Both frame.
	GyroRaw  gyro[3]; ///< Initialized only for a Gyro or Both frame.
};

/** Traversal summary, not a publication or timestamp-validation result. */
struct FifoDecodeResult {
	FifoDecodeStatus status       { FifoDecodeStatus::kEmpty }; ///< Traversal result; publication validation is separate.
	bool             has_metadata { false }; ///< At least one metadata frame was seen before traversal stopped.
	bool             all_combined { true }; ///< No separate-channel frame seen; meaningful for sample batches only.
};

/**
 * @brief Traverse received frames and accumulate channels without publishing or resynchronizing.
 * @tparam mapping Sensor-frame axis conversion; see appendFifoSample().
 * @tparam channels Compile-time channel permission, not object layout or board registration.
 * @tparam gyro_mapping Optional independent gyro frame conversion; defaults to mapping.
 * @tparam AccelBatch Signed 16/32-bit accel batch with a uint8_t sample counter.
 * @tparam GyroBatch Signed 16/32-bit gyro batch with a uint8_t sample counter.
 * @tparam Sample FifoFrameSample-compatible raw axes with optional family-owned metadata.
 * @tparam Decode Callable returning FifoFrame from a ByteCursor and Sample reference.
 * @param[in] data Received bytes, non-overlapping with output; may be null only when bytes is zero.
 * @param[in] bytes Actual received length in bytes, not transfer-buffer capacity.
 * @param[in,out] accel Destination with samples == 0 on entry, even when the accel channel is disabled.
 * @param[in,out] gyro Destination with samples == 0 on entry, even when the gyro channel is disabled.
 * @param[in,out] sample Decoder scratch state; only channels returned by decode need valid axis values.
 * @param[in] decode Owns frame lengths, wire validity and metadata; must consume bytes for each accepted
 * frame, including an explicit End marker. Invalid and Reconfigure may return without consuming bytes.
 * @return Traversal status and frame summary; Complete still requires family-level validation before publication.
 * @pre With FlipYZValidated, decode must reject the storage minimum on each affected channel's Y/Z axes.
 * @note An explicit End stops traversal without inspecting trailing bytes. Unknown/truncated frames do not
 * trigger implicit resynchronization. On Invalid or Reconfigure, discard both partial batches; scratch and
 * captured decoder state are not rolled back. Empty has no samples to publish, but may contain useful metadata.
 * @note Accel and gyro may have different counts, timestamps and rates. This function does not publish either
 * channel and provides no atomic dual-uORB publication guarantee.
 */
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

	if ((!data && bytes != 0)
	    || accel.samples != 0
	    || gyro.samples != 0) {
		result.status = FifoDecodeStatus::kInvalid;

		return result;
	}

	ByteCursor cursor(data, bytes);

	while (cursor.remaining()) {
		const size_t    before = cursor.remaining();
		const FifoFrame frame  = decode(cursor, sample);

		if (frame == FifoFrame::kReconfigure) {
			result.status = FifoDecodeStatus::kReconfigure;

			return result;
		}

		// Every accepted frame must advance the cursor, including metadata and explicit end markers.
		if (frame == FifoFrame::kInvalid || cursor.remaining() >= before) {
			result.status = FifoDecodeStatus::kInvalid;

			return result;
		}

		if (frame == FifoFrame::kEnd) {
			break;
		}

		if (frame == FifoFrame::kMetadata) {
			result.has_metadata = true;
			continue;
		}

		bool valid = true;

		result.all_combined &= frame == FifoFrame::kBoth;

		switch (frame) {
		case FifoFrame::kBoth: {
				if constexpr(channels != FifoChannels::kBoth) {
					valid = false;
					break;
				}

				// Check both capacities before appending either channel.
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
			result.status = FifoDecodeStatus::kInvalid;

			return result;
		}
	}

	result.status = accel.samples || gyro.samples ? FifoDecodeStatus::kComplete : FifoDecodeStatus::kEmpty;

	return result;
}

} // namespace imu
