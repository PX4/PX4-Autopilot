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
 * @brief Bounded fixed-frame traversal with signed 16/32-bit batch storage.
 */

#pragma once

#include "ByteCursor.hpp"

namespace imu
{
/** Sensor-frame conversion only; board rotation remains in the PX4 publishers. */
enum class FifoAxisMapping : uint8_t {
	kIdentity,        ///< Preserve all axes.
	kFlipYZ,          ///< Negate Y/Z, saturating the storage minimum to its maximum.
	kFlipYZValidated, ///< Negate Y/Z after rejecting the storage minimum on these axes.
};

/** Result for one family-decoded fixed frame; Skip never appends stale axes. */
enum class FifoSampleResult : uint8_t {
	kAppend,
	kSkip,
	kInvalid,
};

namespace detail
{
/// @cond INTERNAL
// Deliberately undefined for pointers. RawTraits rejects unsigned and unsupported scalar types.
template<typename Array>
struct FifoAxisArray;

template<typename Raw, size_t size>
struct FifoAxisArray<Raw[size]> {
	using Value = Raw;
	static_assert(RawTraits<Raw>::kSupported, "FIFO axes require signed 16/32-bit storage");
	static constexpr size_t kCapacity { size };
};

template<typename Counter>
constexpr bool kIsFifoCounter { false };

template<>
constexpr bool kIsFifoCounter<uint8_t> { true };
/// @endcond

} // namespace detail

/**
 * @brief Optional driver-local batch storage, never a replacement uORB message.
 * @tparam Raw int16_t for the existing FIFO path, or int32_t for wide raw samples.
 * @tparam capacity Maximum valid samples (1..UINT8_MAX).
 * @note Only the first samples entries are initialized and may be read. No heap allocation
 * or timestamp/scaling policy is introduced. Existing uORB structs can be used directly instead.
 */
template<typename Raw, size_t capacity>
struct SampleBatch {
	static_assert(detail::RawTraits<Raw>::kSupported, "Unsupported sample storage");
	static_assert(capacity > 0, "Batch capacity must be nonzero");
	static_assert(capacity <= UINT8_MAX, "Batch capacity exceeds the sample counter range");

	uint8_t samples { 0 }; ///< Number of initialized entries in each axis.
	Raw x[capacity]; ///< Raw X counts; entries beyond samples are unspecified.
	Raw y[capacity]; ///< Raw Y counts; entries beyond samples are unspecified.
	Raw z[capacity]; ///< Raw Z counts; entries beyond samples are unspecified.
};

/** Scalar type of the batch's fixed X array; Y/Z compatibility is checked by fifoBatchCapacity(). */
template<typename Batch>
using BatchRaw = typename detail::FifoAxisArray<decltype(Batch::x)>::Value;

/**
 * @brief Read three adjacent signed words without an intermediate sample buffer.
 * @tparam order Wire byte order.
 * @tparam shift Arithmetic right shift of each signed word, in bits (0..15).
 * @param[in] data At least six readable bytes; bounds and invalid-value checks belong to the caller.
 * @param[out] axes X/Y/Z values in raw counts after shifting.
 */
template<ByteOrder order, unsigned shift = 0>
inline void readAxes16(const uint8_t *data, int16_t (&axes)[3])
{
	static_assert(shift < 16, "Invalid signed word alignment");
	static_assert(
		order == ByteOrder::kBigEndian
		|| order == ByteOrder::kLittleEndian,
		"Invalid byte order");

	// Keep the three loads explicit: -Os must not introduce a per-axis loop.
	axes[0] = (order == ByteOrder::kLittleEndian ? readLe16(data) : readBe16(data)) >> shift;
	axes[1] = (order == ByteOrder::kLittleEndian ? readLe16(data + 2) : readBe16(data + 2)) >> shift;
	axes[2] = (order == ByteOrder::kLittleEndian ? readLe16(data + 4) : readBe16(data + 4)) >> shift;
}

/**
 * @brief Obtain the number of samples that fit in each axis of a native FIFO batch.
 * @tparam Batch Type with fixed x/y/z arrays of the same int16_t or int32_t type and a uint8_t count.
 * @return Capacity in samples (1..UINT8_MAX); unsupported layouts fail compilation.
 */
template<typename Batch>
constexpr size_t fifoBatchCapacity()
{
	using X = detail::FifoAxisArray<decltype(Batch::x)>;
	using Y = detail::FifoAxisArray<decltype(Batch::y)>;
	using Z = detail::FifoAxisArray<decltype(Batch::z)>;

	constexpr size_t capacity = X::kCapacity;

	static_assert(
		sizeof(typename X::Value) == sizeof(typename Y::Value)
		&& sizeof(typename X::Value) == sizeof(typename Z::Value),
		"FIFO axis types must match");
	static_assert(
		capacity == detail::FifoAxisArray<decltype(Batch::y)>::kCapacity
		&& capacity == detail::FifoAxisArray<decltype(Batch::z)>::kCapacity,
		"FIFO axis capacities must match");
	static_assert(detail::kIsFifoCounter<decltype(Batch::samples)>, "FIFO sample count must be uint8_t");
	static_assert(
		capacity > 0
		&& capacity <= UINT8_MAX,
		"Invalid FIFO capacity for uint8_t sample count");

	return capacity;
}

/**
 * @brief Append one raw sample directly to a native PX4-shaped batch.
 * @tparam mapping Sensor-frame axis conversion, applied once during append.
 * @tparam Batch Fixed signed 16/32-bit axis arrays with a uint8_t sample counter.
 * @param[in,out] batch Destination; existing samples are retained and the count is incremented on success.
 * @param[in] axes Raw X/Y/Z counts of exactly the destination scalar type; no implicit narrowing is allowed.
 * @pre FlipYZValidated requires Y/Z to have been checked against the storage minimum.
 * @return True on append; false if full or wide validated negation would overflow, with batch unchanged.
 * @note Board rotation, scaling and publication remain in the native PX4 publishers.
 */
template<FifoAxisMapping mapping, typename Batch>
inline bool appendFifoSample(Batch &batch, const BatchRaw<Batch> (&axes)[3])
{
	static_assert(
		mapping == FifoAxisMapping::kIdentity
		|| mapping == FifoAxisMapping::kFlipYZ
		|| mapping == FifoAxisMapping::kFlipYZValidated,
		"Invalid FIFO axis mapping");

	uint8_t &samples = batch.samples;

	if (samples >= fifoBatchCapacity<Batch>()) {
		return false;
	}

	if constexpr(mapping == FifoAxisMapping::kFlipYZValidated && sizeof(BatchRaw<Batch>) == sizeof(int32_t)) {
		// Wide storage must never negate INT32_MIN, even if a future decoder
		// violates the validated-mapping contract. Keep the legacy int16 path unchanged.
		if (axes[1] == INT32_MIN || axes[2] == INT32_MIN) {
			return false;
		}
	}

	const uint8_t index = samples++;

	batch.x[index] = axes[0];

	if constexpr(mapping == FifoAxisMapping::kFlipYZ) {
		batch.y[index] = negateSaturated(axes[1]);
		batch.z[index] = negateSaturated(axes[2]);

	} else if constexpr(mapping == FifoAxisMapping::kFlipYZValidated) {
		batch.y[index] = -axes[1];
		batch.z[index] = -axes[2];

	} else {
		batch.y[index] = axes[1];
		batch.z[index] = axes[2];
	}

	return true;
}

/**
 * @brief Append selected fixed-stride samples using a family-owned wire decoder.
 * @tparam mapping Sensor-frame axis conversion; see appendFifoSample().
 * @tparam may_skip Whether the decoder may return FifoSampleResult::kSkip for repeated samples.
 * @tparam min_frame_bytes Decoder's minimum readable frame length; zero selects six bytes for legacy int16 storage.
 * Wide storage requires an explicit wire minimum. Packed decoders must describe the wire, not sizeof their decoded axes.
 * @tparam Batch Signed 16/32-bit batch with a uint8_t sample counter.
 * @tparam Decode Inlineable callable returning Append, Skip or Invalid for one frame and an output axis array.
 * @param[in] data Received bytes; must be non-null and must not overlap batch storage.
 * @param[in] bytes Received length in bytes, nonzero and divisible by stride.
 * @param[in] stride Complete wire frame length in bytes, at least min_frame_bytes; independent of Raw storage width.
 * @param[in] first Zero-based first frame to decode; may equal bytes/stride to select no frames.
 * @param[in] step Frame-index increment, nonzero; unselected frames are not decoded or validated.
 * @param[in,out] batch Destination; supports appending to an existing batch within its capacity.
 * @param[in] decode Receives a frame pointer and axes in raw counts; must not read beyond one stride.
 * @return True if all selected frames were accepted, even when no new samples were appended.
 * @pre With FlipYZValidated, decode must reject the storage minimum on Y/Z before returning Append.
 * @note Check batch.samples before publishing. With may_skip, every frame may be skipped.
 * @note Failure may leave partial output and advanced decoder state. Discard the batch and apply the
 * family recovery policy; neither output nor captured decoder state is rolled back. Reset and timestamps remain local.
 */
template<FifoAxisMapping mapping,
	 bool may_skip = false,
	 size_t min_frame_bytes = 0,
	 typename Batch,
	 typename Decode>
[[nodiscard]] bool decodeFixedFifo(
	const uint8_t *data,
	size_t bytes,
	size_t stride,
	size_t first,
	size_t step,
	Batch &batch,
	Decode decode)
{
	static_assert(
		min_frame_bytes > 0
		|| sizeof(BatchRaw<Batch>) == sizeof(int16_t),
		"Wide fixed decoders require an explicit wire minimum");

	constexpr size_t minimum_frame_bytes = min_frame_bytes ? min_frame_bytes : 6;

	if (!data
	    || !bytes
	    || stride < minimum_frame_bytes
	    || !step
	    || bytes % stride != 0
	    || batch.samples > fifoBatchCapacity<Batch>()) {
		return false;
	}

	const size_t frames = bytes / stride;

	if (first > frames) {
		return false;
	}

	if constexpr(!may_skip) {
		// With no skipped frames, reject an oversized batch before invoking the family decoder.
		const size_t selected = first == frames ? 0 : 1 + (frames - 1 - first) / step;

		if (selected > fifoBatchCapacity<Batch>() - batch.samples) {
			return false;
		}
	}

	for (size_t index = first; index < frames;) {
		// Stop before invoking a decoder once the output is full.
		if (batch.samples >= fifoBatchCapacity<Batch>()) {
			return false;
		}

		BatchRaw<Batch> axes[3];
		const FifoSampleResult result = decode(data + index * stride, axes);

		if (result != FifoSampleResult::kAppend && !(may_skip && result == FifoSampleResult::kSkip)) {
			return false;
		}

		if (result == FifoSampleResult::kAppend && !appendFifoSample<mapping>(batch, axes)) {
			return false;
		}

		if (step >= frames - index) {
			break;
		}

		index += step;
	}

	return true;
}

} // namespace imu
