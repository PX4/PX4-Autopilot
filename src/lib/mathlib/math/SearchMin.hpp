/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file SearchMin.hpp
 *
 * - Binary Search (ToDo)
 * - Golden Section Search
 */

#pragma once

namespace math
{
static constexpr double GOLDEN_RATIO = 1.6180339887; //(sqrt(5)+1)/2

// Type-safe abs
template<typename _Tp>
_Tp abs_t(_Tp val)
{
	return ((val > (_Tp)0) ? val : -val);
}

// golden section search to find extremeum for function with minimum
template<typename _Tp>
inline const _Tp goldensection(const _Tp &arg1, const _Tp &arg2, _Tp(*fun)(_Tp), const _Tp &tol)
{
	_Tp a = arg1;
	_Tp b = arg2;
	_Tp c = b - (b - a) / GOLDEN_RATIO;
	_Tp d = a + (b - a) / GOLDEN_RATIO;

	while (abs_t(c - d) > tol) {

		if (fun(c) < fun(d)) {
			b = d;

		} else {
			a = c;
		}

		c = b - (b - a) / GOLDEN_RATIO;
		d = a + (b - a) / GOLDEN_RATIO;

	}

	return ((b + a) / (_Tp)2);
}
}
