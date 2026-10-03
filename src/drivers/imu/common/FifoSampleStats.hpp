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
 * @brief Bounded raw-count statistics for family-specific FIFO validation.
 */

#pragma once

#include <cstdint>

#include "ByteCursor.hpp"

namespace imu
{
/**
 * @brief Bounded integer statistics for family-owned sample validation.
 *
 * @tparam Raw int16_t or int32_t raw counts; accumulation uses int32_t or int64_t respectively.
 * The extrema have the largest distance from the mean, so checking them
 * replaces a second traversal of every sample. Sensor limits remain local.
 */
template<typename Raw>
class SampleStats
{
	using Traits      = detail::RawTraits<Raw>;
	using Accumulator = typename Traits::Accumulator;

	static_assert(Traits::kSupported, "Raw statistics require signed 16/32-bit storage");

public:
	/** @return True before any sample has been accepted. */
	bool empty() const { return _count == 0; }

	/**
	 * @brief Accumulate one signed raw sample without overflowing the integer sum.
	 * @param[in] sample Raw count, before conversion to physical units.
	 * @return True if accepted; false at UINT16_MAX samples, leaving the statistics unchanged.
	 */
	bool add(Raw sample)
	{
		// UINT16_MAX raw samples fit in the matching wider accumulator.
		if (_count == UINT16_MAX) {
			return false;
		}

		_sum += sample;
		_minimum = sample < _minimum ? sample : _minimum;
		_maximum = sample > _maximum ? sample : _maximum;
		++_count;

		return true;
	}

	/** Reject implicit narrowing, unsigned input or floating-point input. */
	template<typename Other>
	bool add(Other) = delete;

	/**
	 * @brief Check whether every sample is within the allowed distance of the arithmetic mean.
	 * @param[in] maximum_deviation Inclusive nonnegative deviation limit in raw counts; NaN is rejected.
	 * @param[out] mean Floating-point mean in raw counts; unchanged on failure. Wide raw integers
	 * are retained internally, but this explicitly floating-point result can be rounded.
	 * @return True for a nonempty set within the limit; false otherwise.
	 */
	bool meanWithin(float maximum_deviation, float &mean) const
	{
		if (_count == 0 || !(maximum_deviation >= 0.f)) {
			return false;
		}

		const float average = static_cast<float>(_sum) / _count;

		if constexpr(sizeof(Raw) == sizeof(int16_t)) {
			// Preserve the established int16 temperature path and its generated arithmetic.
			if (average - _minimum > maximum_deviation || _maximum - average > maximum_deviation) {
				return false;
			}

		} else {
			// Compare distances before rounding the wide mean to float. Every integer
			// below is bounded to fewer than 49 bits and is exactly representable in double.
			const int64_t lower = _sum - static_cast<int64_t>(_minimum) * _count;
			const int64_t upper = static_cast<int64_t>(_maximum) * _count - _sum;
			const double  limit = static_cast<double>(maximum_deviation) * _count;

			if (static_cast<double>(lower) > limit || static_cast<double>(upper) > limit) {
				return false;
			}
		}

		mean = average;

		return true;
	}

private:
	Accumulator _sum     { 0 };
	uint16_t    _count   { 0 };
	Raw         _minimum { sizeof(Raw) == sizeof(int16_t) ? INT16_MAX : INT32_MAX };
	Raw         _maximum { sizeof(Raw) == sizeof(int16_t) ? INT16_MIN : INT32_MIN };
};

/** Existing FIFO temperature callers retain their int16 storage and arithmetic. */
using FifoSampleStats = SampleStats<int16_t>;

} // namespace imu
