/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

/*
 * @file MedianFilter.hpp
 *
 * @brief Implementation of a median filter with C qsort.
 *
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <px4_platform_common/defines.h>

// Minimal float detection without headers
template<typename U> struct px4_is_floating_point { static constexpr bool value = false; };
template<> struct px4_is_floating_point<float> { static constexpr bool value = true; };
template<> struct px4_is_floating_point<double> { static constexpr bool value = true; };
template<> struct px4_is_floating_point<long double> { static constexpr bool value = true; };

namespace math
{

template<typename T, int WINDOW = 3>
class MedianFilter
{
public:
	static_assert(WINDOW >= 3, "MedianFilter window size must be >= 3");
	static_assert(WINDOW % 2, "MedianFilter window size must be odd"); // odd

	MedianFilter() = default;

	void insert(const T &sample)
	{
		_head = (_head + 1) % WINDOW;
		_buffer[_head] = sample;
	}

	T median()
	{
		T sorted[WINDOW];
		memcpy(sorted, _buffer, sizeof(_buffer));
		qsort(&sorted, WINDOW, sizeof(T), cmp);

		return sorted[WINDOW / 2];
	}

	T apply(const T &sample)
	{
		insert(sample);
		return median();
	}

private:

	static int cmp(const void *a, const void *b)
	{
		const T va = *(const T *)a;
		const T vb = *(const T *)b;

		if constexpr(px4_is_floating_point<T>::value) {
			if (!PX4_ISFINITE(va) && !PX4_ISFINITE(vb)) { return  0; }

			else if (!PX4_ISFINITE(va))                 { return  1; }

			else if (!PX4_ISFINITE(vb))                 { return -1; }
		}

		if (va < vb) { return -1; }

		if (va > vb) { return  1; }

		return 0;
	}

	T _buffer[WINDOW] {};
	uint8_t _head{0};
};

} // namespace math
