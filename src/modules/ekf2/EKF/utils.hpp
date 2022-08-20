/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include <matrix/math.hpp>

#ifndef EKF_UTILS_HPP
#define EKF_UTILS_HPP

// Use Kahan summation algorithm to get the sum of "sum_previous" and "input".
// This function relies on the caller to be responsible for keeping a copy of
// "accumulator" and passing this value at the next iteration.
// Ref: https://en.wikipedia.org/wiki/Kahan_summation_algorithm
inline float kahanSummation(float sum_previous, float input, float &accumulator)
{
	const float y = input - accumulator;
	const float t = sum_previous + y;
	accumulator = (t - sum_previous) - y;
	return t;
}

namespace ecl
{
inline float powf(float x, int exp)
{
	float ret;

	if (exp > 0) {
		ret = x;

		for (int count = 1; count < exp; count++) {
			ret *= x;
		}

		return ret;

	} else if (exp < 0) {
		return 1.0f / ecl::powf(x, -exp);
	}

	return 1.0f;
}

} // namespace ecl

#endif // EKF_UTILS_HPP
