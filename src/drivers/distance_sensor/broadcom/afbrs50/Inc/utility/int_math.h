/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Provides algorithms applied to integer values.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef INT_MATH
#define INT_MATH
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @addtogroup  argus_misc
 * @{
 *****************************************************************************/

#include <stdint.h>
#include <assert.h>

/*! Enables the integer square root function. */
#ifndef INT_SQRT
#define INT_SQRT 0
#endif

/*!***************************************************************************
 * @brief   Integer Base-2 Logarithm.
 *
 * @details Calculates the base-2 logarithm for unsigned integer values. The
 *          result is the integer equivalent of floor(log2(x)).
 *
 * @param   x Input parameter.
 * @return  The floor of the base-2 logarithm.
 *****************************************************************************/
inline uint32_t log2i(uint32_t x)
{
	assert(x != 0);
#if 1
	return (uint32_t)(31 - __builtin_clz(x));
#else
#define S(k) if (x >= (1 << k)) { i += k; x >>= k; }
	int i = 0; S(16); S(8); S(4); S(2); S(1); return i;
#undef S
#endif
}

/*!***************************************************************************
 * @brief   Integer Base-2 Logarithm with rounded result.
 *
 * @details Calculates the base-2 logarithm for unsigned integer values and
 *          returns the rounded result. The result is the integer equivalent
 *          of round(log2(x)).
 *
 *          It is finding the nearest power-of-two value s.t. |x - 2^n| becomes
 *          minimum for all n.
 *
 * @param   x Input parameter.
 * @return  The rounded value of the base-2 logarithm.
 *****************************************************************************/
inline uint32_t log2_round(uint32_t x)
{
	assert(x != 0);
#if 0
	const uint32_t y = x;
	const uint32_t i = 0;

	while (y >>= 1) { i++; }

#else
	const uint32_t i = log2i(x);
#endif
	return (i + ((x >> (i - 1U)) == 3U));
}

/*!***************************************************************************
 * @brief   Finding the nearest power-of-two value.
 *
 * @details Implemented s.t. |x - 2^n| becomes minimum for all n.
 *          Special case 0: returns 0;
 *          Maximum input: 3037000499; higher number result in overflow! (returns 0)
 *
 * @param   x Input parameter.
 * @return  Nearest power-of-two number, i.e. 2^n.
 *****************************************************************************/
inline uint32_t binary_round(uint32_t x)
{
	assert(x != 0);
	const uint32_t shift = log2_round(x);
	return (shift > 31U) ? 0 : 1U << shift;
}

/*!***************************************************************************
 * @brief   Counting bits set in a 32-bit unsigned integer.
 *
 * @details @see http://graphics.stanford.edu/~seander/bithacks.html
 *
 * @param   x Input parameter.
 * @return  Number of bits set in input value.
 *****************************************************************************/
inline uint32_t popcount(uint32_t x)
{
	// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
	x = x - ((x >> 1) & 0x55555555);
	x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
	return (((x + (x >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

/*!***************************************************************************
 * @brief   Determining if an integer is a power of 2
 *
 * @details @see http://graphics.stanford.edu/~seander/bithacks.html
 *
 * @param   x Input parameter.
 * @return  True if integer is power of 2.
 *****************************************************************************/
inline uint32_t ispowoftwo(uint32_t x)
{
	return x && !(x & (x - 1));
}

/*!***************************************************************************
 * @brief   Calculates the absolute value.
 *
 * @param   x Input parameter.
 * @return  The absolute value of x.
 *****************************************************************************/
inline uint32_t absval(int32_t x)
{
	// Note: special case of INT32_MIN must be handled correctly:
	return x < 0 ? ((~(uint32_t)(x)) + 1) : (uint32_t)x;

	/* alternative with equal performance:*/
//  int32_t y = x >> 31;
//  return (x ^ y) - y;
	/* wrong implementation:
	 * does not correctly return abs(INT32_MIN) on 32-bit platform */
//  return x < 0 ? (uint32_t)(-x) : (uint32_t)x;
}

/*!***************************************************************************
 * @brief   Calculates the floor division by a factor of 2: floor(x / 2^n).
 *
 * @param   x Input parameter.
 * @param   n The shift value, maximum is 31.
 * @return  The floor division by 2^n result.
 *****************************************************************************/
inline uint32_t floor2(uint32_t x, uint_fast8_t n)
{
	assert(n < 32);
	return x >> n;
}

/*!***************************************************************************
 * @brief   Calculates the ceildiv division by a factor of 2: ceildiv(x / 2^n).
 *
 * @param   x Input parameter.
 * @param   n The shift value, maximum is 31.
 * @return  The ceildiv division by 2^n result.
 *****************************************************************************/
inline uint32_t ceiling2(uint32_t x, uint_fast8_t n)
{
	assert(n < 32);
	return x ? (1 + ((x - 1) >> n)) : 0;
}

/*!***************************************************************************
 * @brief   Calculates the ceildiv division: ceildiv(x / y).
 *
 * @param   x Numerator
 * @param   y Denominator
 * @return  The result of the ceildiv division ceildiv(x / y).
 *****************************************************************************/
inline uint32_t ceildiv(uint32_t x, uint32_t y)
{
	assert(y != 0);
	return x ? (1 + ((x - 1) / y)) : 0;
}

/*!***************************************************************************
 * @brief   Calculates the maximum of two values.
 *
 * @param   a Input parameter.
 * @param   b Input parameter.
 * @return  The maximum value of the input parameters.
 *****************************************************************************/
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/*!***************************************************************************
 * @brief   Calculates the minimum of two values.
 *
 * @param   a Input parameter.
 * @param   b Input parameter.
 * @return  The minimum value of the input parameters.
 *****************************************************************************/
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/*!***************************************************************************
 * @brief   Clamps a value between a minimum and maximum boundary.
 *
 * @details Clamps the values such that the condition min <= x <= max is true.
 *
 * @note    The condition \p min <= \p max must hold!!!
 *
 * @param   x The input parameter to be clamped.
 * @param   min The minimum or lower boundary.
 * @param   max The maximum or upper boundary.
 * @return  The clamped value of the input parameter within [min,max].
 *****************************************************************************/
#define CLAMP(x, min, max) (MIN(MAX((x), (min)), (max)))

#if INT_SQRT
/*!***************************************************************************
 * @brief   Calculates the integer square root of x.
 *
 * @details The integer square root is defined as:
 *          isqrt(x) = (int)sqrt(x)
 *
 *          @see https://en.wikipedia.org/wiki/Integer_square_root
 *          @see https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
 *
 * @param   x Input parameter.
 * @return      isqrt(x)
 *****************************************************************************/
inline uint32_t isqrt(uint32_t v)
{
	unsigned t, q, b, r;
	r = v;           // r = v - x²
	b = 0x40000000;  // a²
	q = 0;           // 2ax

	while (b > 0) {
		t = q + b;   // t = 2ax + a²
		q >>= 1;     // if a' = a/2, then q' = q/2

		if (r >= t) { // if (v - x²) >= 2ax + a²
			r -= t;  // r' = (v - x²) - (2ax + a²)
			q += b;  // if x' = (x + a) then ax' = ax + a², thus q' = q' + b
		}

		b >>= 2;     // if a' = a/2, then b' = b / 4
	}

	return q;
}
#endif // INT_SQRT

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* INT_MATH */
