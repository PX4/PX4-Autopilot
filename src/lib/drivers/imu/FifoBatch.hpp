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
 * @file FifoBatch.hpp
 * @brief Bounded fixed-frame traversal with signed 16-bit batch storage.
 */

#pragma once

#include "ByteCursor.hpp"

#include <cmath>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/atomic.h>

namespace imu
{

constexpr bool validFifoSampleDtUs(float sample_dt_us)
{
	return sample_dt_us > 0.f && sample_dt_us <= 1e9f;
}

inline uint64_t roundDurationUs(float duration_us)
{
	return duration_us > 0.f ? static_cast<uint64_t>(llroundf(duration_us)) : 0u;
}

/**
 * @brief Back-calculate a batch's end timestamp when polling leaves newer frames in FIFO.
 *
 * When reading only the oldest read_samples out of available_samples without a
 * sample-aligned DRDY interrupt timestamp, timestamp_sample corresponds to the
 * newest available frame and must be shifted backward by (available - read) periods.
 */
inline hrt_abstime adjustFifoBacklogTimestamp(
	hrt_abstime timestamp_sample,
	bool using_drdy_timestamp,
	uint16_t available_samples,
	uint16_t read_samples,
	float sample_dt_us)
{
	if (using_drdy_timestamp || available_samples <= read_samples || !validFifoSampleDtUs(sample_dt_us)) {
		return timestamp_sample;
	}

	const uint64_t delta_us = roundDurationUs(static_cast<float>(available_samples - read_samples) * sample_dt_us);

	return timestamp_sample > delta_us ? (timestamp_sample - delta_us) : 0;
}

/** @brief Resolved FIFO batch end timestamp and anchor consumption decision. */
struct DrdyBatchTimestamp {
	hrt_abstime timestamp_sample { 0 };
	bool        using_drdy       { false };
	bool        consume_anchor   { false };
};

/**
 * @brief Align a latched batch-boundary DRDY timestamp across FIFO backlog drain passes.
 *
 * When DRDY fires on every sample and latches every N samples, a multi-interrupt backlog
 * leaves the newest latched anchor near the tail of the FIFO. Mapping that anchor to its
 * 1-indexed FIFO position allows each drained batch to compute its exact end timestamp
 * without shifting N+1 jitter frames or reusing stale/frozen anchors.
 */
inline DrdyBatchTimestamp resolveDrdyBatchTimestamp(
	hrt_abstime now,
	hrt_abstime latched_timestamp,
	uint32_t    pulse_count,
	uint32_t    latched_pulse,
	uint32_t    consumed_pulse,
	bool        has_consumed_pulse,
	uint16_t    available_samples,
	uint16_t    read_samples,
	float       sample_dt_us,
	uint32_t    max_scheduling_delay_us)
{
	if (latched_timestamp == 0
	    || (has_consumed_pulse && latched_pulse == consumed_pulse)
	    || now < latched_timestamp
	    || available_samples == 0
	    || read_samples == 0
	    || read_samples > available_samples
	    || !validFifoSampleDtUs(sample_dt_us)) {
		return DrdyBatchTimestamp {
			adjustFifoBacklogTimestamp(now, false, available_samples, read_samples, sample_dt_us),
			false,
			false
		};
	}

	const uint32_t after_latched = pulse_count - latched_pulse;

	if (after_latched >= available_samples) {
		return DrdyBatchTimestamp {
			adjustFifoBacklogTimestamp(now, false, available_samples, read_samples, sample_dt_us),
			false,
			false
		};
	}

	const uint64_t expected_tail_us = roundDurationUs(static_cast<float>(after_latched) * sample_dt_us);

	if ((now - latched_timestamp) >= (expected_tail_us + max_scheduling_delay_us)) {
		return DrdyBatchTimestamp {
			adjustFifoBacklogTimestamp(now, false, available_samples, read_samples, sample_dt_us),
			false,
			false
		};
	}

	const int32_t latched_sample = static_cast<int32_t>(available_samples) - static_cast<int32_t>(after_latched);
	const int32_t sample_offset  = static_cast<int32_t>(read_samples) - latched_sample;

	hrt_abstime resolved_timestamp = latched_timestamp;

	if (sample_offset > 0) {
		const uint64_t advance_us = roundDurationUs(static_cast<float>(sample_offset) * sample_dt_us);
		resolved_timestamp = (now - latched_timestamp) >= advance_us ? (latched_timestamp + advance_us) : now;

	} else if (sample_offset < 0) {
		const uint64_t back_us = roundDurationUs(static_cast<float>(-sample_offset) * sample_dt_us);
		resolved_timestamp = latched_timestamp > back_us ? (latched_timestamp - back_us) : 0;
	}

	const bool consume_anchor = read_samples >= static_cast<uint16_t>(latched_sample);

	return DrdyBatchTimestamp { resolved_timestamp, true, consume_anchor };
}

/**
 * @brief Lock-free SPSC (ISR -> work queue) seqlock anchor for per-sample DRDY timestamps.
 *
 * ISR calls onInterrupt() on every DRDY pulse; the work queue takes snapshot() before
 * reading the hardware FIFO count and calls resolve() to align timestamps and track
 * consumed anchors strictly in work-queue state without overwriting ISR updates.
 */
class DrdySampleAnchor
{
public:
	struct Snapshot {
		hrt_abstime latched_timestamp { 0 };
		uint32_t    pulse_count       { 0 };
		uint32_t    latched_pulse     { 0 };
		bool        valid             { false };
	};

	bool onInterrupt(hrt_abstime timestamp, uint16_t batch_samples)
	{
		if (timestamp == 0 || batch_samples == 0) {
			return false;
		}

		const uint32_t epoch = _reset_epoch.load();
		const uint32_t seq   = _seq.load();
		_seq.store(seq + 1u);

		uint32_t pulse = _pulse_count.load();

		if (epoch != _isr_epoch) {
			_isr_epoch      = epoch;
			_batch_progress = 0;
			pulse           = 0;
			_latched_timestamp.store(0);
			_latched_pulse.store(0);
		}

		++pulse;
		_pulse_count.store(pulse);

		bool latched = false;

		if (++_batch_progress >= batch_samples) {
			_batch_progress = 0;
			_latched_pulse.store(pulse);
			_latched_timestamp.store(timestamp);
			latched = true;
		}

		_latched_epoch.store(epoch);
		_seq.store(seq + 2u);

		return latched;
	}

	Snapshot snapshot() const
	{
		const uint32_t target_epoch = _reset_epoch.load();

		for (unsigned attempt = 0; attempt < 4; ++attempt) {
			const uint32_t seq1 = _seq.load();

			if (seq1 & 1u) {
				continue;
			}

			const uint32_t    pulse_count       = _pulse_count.load();
			const uint32_t    latched_pulse     = _latched_pulse.load();
			const uint32_t    latched_epoch     = _latched_epoch.load();
			const hrt_abstime latched_timestamp = _latched_timestamp.load();
			const uint32_t    seq2              = _seq.load();

			if (seq1 == seq2) {
				if (latched_epoch != target_epoch || latched_timestamp == 0) {
					return Snapshot {};
				}

				return Snapshot { latched_timestamp, pulse_count, latched_pulse, true };
			}
		}

		return Snapshot {};
	}

	DrdyBatchTimestamp resolve(
		const Snapshot &snap,
		hrt_abstime     now,
		uint16_t        available_samples,
		uint16_t        read_samples,
		float           sample_dt_us,
		uint32_t        max_scheduling_delay_us)
	{
		const DrdyBatchTimestamp resolved = resolveDrdyBatchTimestamp(
				now,
				snap.valid ? snap.latched_timestamp : 0,
				snap.pulse_count,
				snap.latched_pulse,
				_consumed_pulse,
				_has_consumed_pulse,
				available_samples,
				read_samples,
				sample_dt_us,
				max_scheduling_delay_us);

		if (resolved.using_drdy && resolved.consume_anchor) {
			_consumed_pulse     = snap.latched_pulse;
			_has_consumed_pulse = true;
		}

		return resolved;
	}

	void reset()
	{
		_consumed_pulse     = 0;
		_has_consumed_pulse = false;
		_reset_epoch.fetch_add(1u);
	}

private:
	// ISR-written, work-queue-read state (scalar atomics <= 8 bytes for lock-free POSIX guarantees).
	px4::atomic<uint32_t>    _seq               { 0 };
	px4::atomic<uint32_t>    _pulse_count       { 0 };
	px4::atomic<uint32_t>    _latched_pulse     { 0 };
	px4::atomic<uint32_t>    _latched_epoch     { 0 };
	px4::atomic<hrt_abstime> _latched_timestamp { 0 };

	// ISR-only state.
	uint32_t _isr_epoch      { 0 };
	uint32_t _batch_progress { 0 };

	// Work-queue-only state (epoch signals ISR to discard pre-reset counters/anchors).
	px4::atomic<uint32_t> _reset_epoch        { 0 };
	uint32_t              _consumed_pulse     { 0 };
	bool                  _has_consumed_pulse { false };
};

enum class FifoAxisMapping : uint8_t {
	kIdentity,
	kFlipYZ,
	kFlipYZValidated,
};

enum class FifoSampleResult : uint8_t {
	kAppend,
	kSkip,
	kInvalid,
};

namespace detail
{

template<unsigned shift>
constexpr int16_t shiftArithmeticRight16(int16_t value)
{
	static_assert(shift < 16, "Invalid signed word alignment");

	if constexpr(shift == 0) {
		return value;

	} else {
		return value >= 0
		       ? static_cast<int16_t>(static_cast<uint16_t>(value) >> shift)
		       : static_cast<int16_t>(-1 - static_cast<int16_t>(static_cast<uint16_t>(-1 - value) >> shift));
	}
}

} // namespace detail

template<size_t capacity>
struct SampleBatch {
	static_assert(capacity > 0 && capacity <= UINT8_MAX, "Invalid batch capacity");

	uint8_t samples { 0 };
	int16_t x[capacity] {};
	int16_t y[capacity] {};
	int16_t z[capacity] {};
};

template<ByteOrder order, unsigned shift = 0>
inline void readAxes16(const uint8_t *data, int16_t (&axes)[3])
{
	static_assert(shift < 16, "Invalid signed word alignment");
	static_assert(order == ByteOrder::kBigEndian || order == ByteOrder::kLittleEndian, "Invalid byte order");

	const auto read_word = [](const uint8_t *bytes) {
		const int16_t raw = order == ByteOrder::kLittleEndian ? readLe16(bytes) : readBe16(bytes);
		return detail::shiftArithmeticRight16<shift>(raw);
	};

	axes[0] = read_word(data);
	axes[1] = read_word(data + 2);
	axes[2] = read_word(data + 4);
}

template<typename Batch>
constexpr size_t fifoBatchCapacity()
{
	static_assert(
		sizeof(Batch::x[0]) == sizeof(int16_t)
		&& sizeof(Batch::x) == sizeof(Batch::y)
		&& sizeof(Batch::x) == sizeof(Batch::z),
		"FIFO batch axes must be matching int16_t arrays");
	static_assert(sizeof(Batch::samples) == sizeof(uint8_t), "FIFO sample count must be uint8_t");

	constexpr size_t capacity = sizeof(Batch::x) / sizeof(int16_t);
	static_assert(capacity > 0 && capacity <= UINT8_MAX, "Invalid FIFO capacity for uint8_t sample count");

	return capacity;
}

namespace detail
{

template<FifoAxisMapping mapping, typename Batch>
inline bool storeFifoSample(Batch &batch, uint8_t index, const int16_t (&axes)[3])
{
	static_assert(
		mapping == FifoAxisMapping::kIdentity
		|| mapping == FifoAxisMapping::kFlipYZ
		|| mapping == FifoAxisMapping::kFlipYZValidated,
		"Invalid FIFO axis mapping");

	if constexpr(mapping == FifoAxisMapping::kFlipYZValidated) {
		if (axes[1] == INT16_MIN || axes[2] == INT16_MIN) {
			return false;
		}
	}

	batch.x[index] = axes[0];

	if constexpr(mapping == FifoAxisMapping::kFlipYZ) {
		batch.y[index] = negateSaturated(axes[1]);
		batch.z[index] = negateSaturated(axes[2]);

	} else if constexpr(mapping == FifoAxisMapping::kFlipYZValidated) {
		batch.y[index] = static_cast<int16_t>(-axes[1]);
		batch.z[index] = static_cast<int16_t>(-axes[2]);

	} else {
		batch.y[index] = axes[1];
		batch.z[index] = axes[2];
	}

	return true;
}

template<FifoAxisMapping mapping, bool may_skip, typename Batch, typename Decode>
inline bool decodeFixedSamples(
	const uint8_t *data,
	size_t frames,
	size_t stride,
	size_t first,
	size_t step,
	Batch &batch,
	Decode decode)
{
	if (!step || first > frames || batch.samples > fifoBatchCapacity<Batch>()) {
		return false;
	}

	if constexpr(!may_skip) {
		const size_t selected = first == frames ? 0 : 1 + (frames - 1 - first) / step;

		if (selected > fifoBatchCapacity<Batch>() - batch.samples) {
			return false;
		}
	}

	uint8_t next_sample = batch.samples;

	for (size_t index = first; index < frames;) {
		if constexpr(may_skip) {
			if (next_sample >= fifoBatchCapacity<Batch>()) {
				return false;
			}
		}

		int16_t axes[3] {};
		const FifoSampleResult result = decode(data + index * stride, axes);

		if (result != FifoSampleResult::kAppend && !(may_skip && result == FifoSampleResult::kSkip)) {
			return false;
		}

		if (result == FifoSampleResult::kAppend) {
			if (!detail::storeFifoSample<mapping>(batch, next_sample, axes)) {
				return false;
			}

			++next_sample;
		}

		if (step >= frames - index) {
			break;
		}

		index += step;
	}

	batch.samples = next_sample;

	return true;
}

} // namespace detail

template<FifoAxisMapping mapping, typename Batch>
inline bool appendFifoSample(Batch &batch, const int16_t (&axes)[3])
{
	if (batch.samples >= fifoBatchCapacity<Batch>() || !detail::storeFifoSample<mapping>(batch, batch.samples, axes)) {
		return false;
	}

	++batch.samples;

	return true;
}

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
	constexpr size_t minimum_frame_bytes = min_frame_bytes ? min_frame_bytes : 6;

	if (!data || !bytes || stride < minimum_frame_bytes || bytes % stride != 0) {
		return false;
	}

	const size_t frames = bytes / stride;

	return detail::decodeFixedSamples<mapping, may_skip>(data, frames, stride, first, step, batch, decode);
}

} // namespace imu
