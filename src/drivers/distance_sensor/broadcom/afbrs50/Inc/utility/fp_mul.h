/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Provides definitions and basic macros for fixed point data types.
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

#ifndef FP_MUL_H
#define FP_MUL_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @addtogroup  argus_fp
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include "utility/fp_rnd.h"

/*!***************************************************************************
 * Set to use hardware division (Cortex-M3/4) over software division (Cortex-M0/1).
 *****************************************************************************/
#ifndef USE_64BIT_MUL
#define USE_64BIT_MUL 0
#endif

#if !USE_64BIT_MUL
/*!***************************************************************************
 * @brief   Long multiplication of two unsigned 32-bit into an 64-bit value on
 *          32-bit architecture.
 *
 * @details w (two words) gets the product of u and v (one word each).
 *          w[0] is the most significant word of the result, w[1] the least.
 *          (The words are in big-endian order).
 *          It is Knuth's Algorithm M from [Knu2] section 4.3.1.
 * *
 * @see     http://www.hackersdelight.org/hdcodetxt/muldwu.c.txt
 *
 * @param   w The result (u * v) value given as two unsigned 32-bit numbers:
 *              w[0] is the most significant word of the result, w[1] the least.
 *              (The words are in big-endian order).
 * @param   u Left hand side of the multiplication.
 * @param   v Right hand side of the multiplication.
 *****************************************************************************/
inline void muldwu(uint32_t w[], uint32_t u, uint32_t v)
{
	const uint32_t u0 = u >> 16U;
	const uint32_t u1 = u & 0xFFFFU;
	const uint32_t v0 = v >> 16U;
	const uint32_t v1 = v & 0xFFFFU;

	uint32_t t = u1 * v1;
	const uint32_t w3 = t & 0xFFFFU;
	uint32_t k = t >> 16U;

	t = u0 * v1 + k;
	const uint32_t w2 = t & 0xFFFFU;
	const uint32_t w1 = t >> 16U;

	t = u1 * v0 + w2;
	k = t >> 16U;

	w[0] = u0 * v0 + w1 + k;
	w[1] = (t << 16U) + w3;
}
#endif

/*!***************************************************************************
 * @brief   64-bit implementation of an unsigned multiplication with fixed point format.
 *
 * @details Algorithm to evaluate a*b, where a and b are arbitrary fixed point
 *          number of 32-bit width. The multiplication is done in 64-bit and
 *          the result is shifted down by the passed shift parameter in order
 *          to return a 32-bit value.
 *          The shift is executed with correct rounding.
 *
 *          Note that the result must fit into the 32-bit value. An assertion
 *          error occurs otherwise (or undefined behavior of no assert available).
 *
 * @param   u The left parameter in UQx1.y1 format
 * @param   v The right parameter in UQx2.y2 format
 * @param   shift The final right shift (rounding) value.
 * @return  Result = (a*b)>>shift in UQx.(y1+y2-shift) format.
 *****************************************************************************/
inline uint32_t fp_mulu(uint32_t u, uint32_t v, uint_fast8_t shift)
{
	assert(shift <= 32);
#if USE_64BIT_MUL
	const uint64_t w = (uint64_t)u * (uint64_t)v;
	return (uint32_t)((w >> shift) + ((w >> (shift - 1)) & 1U));
#else
	uint32_t tmp[2] = { 0 };
	muldwu(tmp, u, v);

	assert(shift ? tmp[0] <= (UINT32_MAX >> (32 - shift)) : tmp[0] == 0);

	if (32 - shift) {
		return ((tmp[0] << (32 - shift)) + fp_rndu(tmp[1], shift));

	} else {
		return tmp[1] > (UINT32_MAX >> 1) ? tmp[0] + 1 : tmp[0];
	}

#endif
}

/*!***************************************************************************
 * @brief   64-bit implementation of a signed multiplication with fixed point format.
 *
 * @details Algorithm to evaluate a*b, where a and b are arbitrary fixed point
 *          number of 32-bit width. The multiplication is done in 64-bit and
 *          the result is shifted down by the passed shift parameter in order
 *          to return a 32-bit value.
 *          The shift is executed with correct rounding.
 *
 *          Note that the result must fit into the 32-bit value. An assertion
 *          error occurs otherwise (or undefined behavior of no assert available).
 *
 * @param   u The left parameter in Qx1.y1 format
 * @param   v The right parameter in Qx2.y2 format
 * @param   shift The final right shift (rounding) value.
 * @return  Result = (a*b)>>shift in Qx.(y1+y2-shift) format.
 *****************************************************************************/
inline int32_t fp_muls(int32_t u, int32_t v, uint_fast8_t shift)
{
	int_fast8_t sign = 1;

	uint32_t u2, v2;

	if (u < 0) { u2 = (uint32_t) - u; sign = -sign; } else { u2 = (uint32_t)u; }

	if (v < 0) { v2 = (uint32_t) - v; sign = -sign; } else { v2 = (uint32_t)v; }

	const uint32_t res = fp_mulu(u2, v2, shift);

	assert(sign > 0 ? res <= 0x7FFFFFFFU : res <= 0x80000000U);

	return sign > 0 ? (int32_t)res : -(int32_t)res;
}


/*!***************************************************************************
 * @brief   48-bit implementation of a unsigned multiplication with fixed point format.
 *
 * @details Algorithm to evaluate a*b, where a and b are arbitrary fixed point
 *          numbers with 32-bit unsigned and 16-bit unsigned format respectively.
 *          The multiplication is done in two 16x16-bit operations and the
 *          result is shifted down by the passed shift parameter in order to
 *          return a 32-bit value.
 *
 *          Note that the result must fit into the 32-bit value. An assertion
 *          error occurs otherwise (or undefined behavior of no assert available).
 *
 * @param   u The left parameter in Qx1.y1 format
 * @param   v The right parameter in Qx2.y2 format
 * @param   shift The final right shift (rounding) value.
 * @return  Result = (a*b)>>shift in Qx.(y1+y2-shift) format.
 *****************************************************************************/
inline uint32_t fp_mul_u32_u16(uint32_t u, uint16_t v, uint_fast8_t shift)
{
	assert(shift <= 48);

	if (shift > 16) {
		uint32_t msk = 0xFFFFU;
		uint32_t a = (u >> 16U) * v;
		uint32_t b = (msk & u) * v;
		return fp_rndu(a, shift - 16) + fp_rndu(b, shift);

	} else {
		uint32_t msk = ~(0xFFFFFFFFU << shift);
		uint32_t a = (u >> shift) * v;
		uint32_t b = fp_rndu((msk & u) * v, shift);
		return a + b;
	}
}

/*!***************************************************************************
 * @brief   48-bit implementation of an unsigned/signed multiplication with fixed point format.
 *
 * @details Algorithm to evaluate a*b, where a and b are arbitrary fixed point
 *          numbers with 32-bit signed and 16-bit unsigned format respectively.
 *          The multiplication is done in two 16x16-bit operations and the
 *          result is shifted down by the passed shift parameter in order to
 *          return a 32-bit value.
 *          The shift is executed with correct rounding.
 *
 *          Note that the result must fit into the 32-bit value. An assertion
 *          error occurs otherwise (or undefined behavior of no assert available).
 *
 * @param   u The left parameter in Qx1.y1 format
 * @param   v The right parameter in Qx2.y2 format
 * @param   shift The final right shift (rounding) value.
 * @return  Result = (a*b)>>shift in Qx.(y1+y2-shift) format.
 *****************************************************************************/
inline int32_t fp_mul_s32_u16(int32_t u, uint16_t v, uint_fast8_t shift)
{
	return u >= 0 ?
	       (int32_t)fp_mul_u32_u16((uint32_t)u, v, shift) :
	       -(int32_t)fp_mul_u32_u16((uint32_t) - u, v, shift);
}

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* FP_MUL_H */
