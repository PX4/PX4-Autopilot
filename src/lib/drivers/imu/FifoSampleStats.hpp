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
 * @file FifoSampleStats.hpp
 * @brief Bounded raw-count statistics for family-specific FIFO validation.
 */

#pragma once

#include <cstdint>

namespace imu
{

class FifoSampleStats
{
public:
	bool empty() const { return _count == 0; }

	bool add(int16_t sample)
	{
		if (_count == UINT16_MAX) {
			return false;
		}

		_sum += sample;
		_minimum = sample < _minimum ? sample : _minimum;
		_maximum = sample > _maximum ? sample : _maximum;
		++_count;

		return true;
	}

	bool meanWithin(float maximum_deviation, float &mean) const
	{
		if (_count == 0 || !(maximum_deviation >= 0.f)) {
			return false;
		}

		if (_minimum == _maximum) {
			mean = static_cast<float>(_minimum);

			return true;
		}

		const double average   = static_cast<double>(_sum) / _count;
		const double tolerance = static_cast<double>(maximum_deviation);

		if (average - _minimum > tolerance || _maximum - average > tolerance) {
			return false;
		}

		mean = static_cast<float>(average);

		return true;
	}

private:
	int32_t  _sum     { 0 };
	uint16_t _count   { 0 };
	int16_t  _minimum { INT16_MAX };
	int16_t  _maximum { INT16_MIN };
};

} // namespace imu
