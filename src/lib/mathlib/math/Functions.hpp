/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file Functions.hpp
 *
 * collection of rather simple mathematical functions that get used over and over again
 */

#pragma once

#include <platforms/px4_defines.h>

namespace math
{

// Type-safe signum function
template<typename T>
int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

/*
 * So called exponential curve function implementation.
 * It is essentially a linear combination between a linear and a cubic function.
 * It's used in the range [-1,1]
 */
template<typename _Tp>
inline const _Tp expo(const _Tp &value, const _Tp &e)
{
	_Tp x = constrain(value, (_Tp) - 1, (_Tp)1);
	return (1 - e) * x + e * x * x * x;
}

template<typename _Tp>
inline const _Tp deadzone(const _Tp &value, const _Tp &dz)
{
	_Tp x = constrain(value, (_Tp) - 1, (_Tp)1);
	_Tp dzc = constrain(dz, (_Tp) - 1, (_Tp)1);
	// Rescale the input such that we get a piecewise linear function that will be continuous with applied deadzone
	_Tp out = (x - sign(x) * dzc) / (1 - dzc);
	// apply the deadzone (values zero around the middle)
	return out * (fabsf(x) > dzc);
}

template<typename _Tp>
inline const _Tp expo_deadzone(const _Tp &value, const _Tp &e, const _Tp &dz)
{
	return expo(deadzone(value, dz), e);
}


/*
 * Constant, linear, constant function with the two corner points as parameters
 * y_high          -------
 *                /
 *               /
 *              /
 * y_low -------
 *         x_low   x_high
 */
template<typename _Tp>
inline const _Tp gradual(const _Tp &value, const _Tp &x_low, const _Tp &x_high, const _Tp &y_low, const _Tp &y_high)
{
	if (value < x_low) {
		return y_low;

	} else if (value > x_high) {
		return y_high;

	} else {
		/* linear function between the two points */
		float a = (y_high - y_low) / (x_high - x_low);
		float b = y_low - a * x_low;
		return  a * value + b;
	}
}

}
