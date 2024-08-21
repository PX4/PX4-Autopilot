/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Provides averaging algorithms for fixed point data types.
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

#ifndef FP_EMA_H
#define FP_EMA_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @addtogroup  argus_fp
 * @{
 *****************************************************************************/

#include "fp_def.h"

#include "utility/fp_rnd.h"
#include "utility/fp_mul.h"

/*!***************************************************************************
 * @brief   Circular exponentially weighted moving average using UQ1.15 format.
 *
 * @details Evaluates the moving average (exponentially weighted) for circular
 *          data in UQ1.15 format.
 *          Circular data is that MAX_VALUE + 1 == MIN_VALUE. For example the
 *          usual phase information.
 *
 *          Problem: Due to circularity of phase values, i.e. 0+x and 2PI+x are
 *          the same, the usual EMA has issues with the wrap around effect.
 *          Especially for vectors with phase around 0 (or 2PI), two values
 *          like 0 + x and PI - y are averaged to something around PI instead
 *          of 0 which would be more correct.
 *
 *          Solution: Assume that phase jumps of more than PI are not allowed
 *          or possible. If a deviation of the new value to the smoothed signal
 *          occurs, it is clear that this stems from the wrap around effect and
 *          can be caught and correctly handled by the smoothing algorithm.
 *
 *          Caution: If a target comes immediately into the field of view, phase
 *          jumps of > PI are indeed possible and volitional. However, the
 *          averaging break there anyway since the smoothed signal approaches
 *          only with delay to the correct values. The error made here is, that
 *          the smoothed signal approaches from the opposite direction. However,
 *          is approaches even faster since it always takes the shortest
 *          direction.
 *
 * @param   mean The previous mean value in UQ1.15 format.
 * @param   x The current value to be added to the average UQ1.15 format.
 * @param   weight The EMA weight in UQ0.7 format.
 * @return  The new mean value in UQ1.15 format.
 *****************************************************************************/
inline uq1_15_t fp_ema15c(uq1_15_t mean, uq1_15_t x, uq0_8_t weight)
{
	if (weight == 0) { return x; }

	// Heeds the wrap around effect by casting dx to int16:
	const int16_t dx = (int16_t)(x - mean);
	const int32_t diff = weight * dx;
	return (uq1_15_t)fp_rnds((mean << 8U) + diff, 8U);
}

/*!***************************************************************************
 * @brief   Exponentially weighted moving average using the Q11.4 format.
 *
 * @details Evaluates the moving average (exponentially weighted) for data in
 *          Q11.4 format.
 *
 * @param   mean The previous mean value in Q11.4 format.
 * @param   x The current value to be added to the average Q11.4 format.
 * @param   weight The EMA weight in UQ0.7 format.
 * @return  The new mean value in Q11.4 format.
 *****************************************************************************/
inline q11_4_t fp_ema4(q11_4_t mean, q11_4_t x, uq0_8_t weight)
{
	if (weight == 0) { return x; }

	const int32_t dx = x - mean;
	const int32_t diff = weight * dx;
	return (q11_4_t)fp_rnds((mean << 8U) + diff, 8U);
}

/*!***************************************************************************
 * @brief   Exponentially weighted moving average using the Q7.8 format.
 *
 * @details Evaluates the moving average (exponentially weighted) for data in
 *          Q7.8 format.
 *
 * @param   mean The previous mean value in Q7.8 format.
 * @param   x The current value to be added to the average Q7.8 format.
 * @param   weight The EMA weight in UQ0.7 format.
 * @return  The new mean value in Q7.8 format.
 *****************************************************************************/
inline q7_8_t fp_ema8(q7_8_t mean, q7_8_t x, uq0_8_t weight)
{
	return (q7_8_t)fp_ema4(mean, x, weight);
}

/*!***************************************************************************
 * @brief   Exponentially weighted moving average using the Q15.16 format.
 *
 * @details Evaluates the moving average (exponentially weighted) for data in
 *          Q15.16 format.
 *
 * @param   mean The previous mean value in Q15.16 format.
 * @param   x The current value to be added to the average Q15.16 format.
 * @param   weight The EMA weight in UQ0.7 format.
 * @return  The new mean value in Q15.16 format.
 *****************************************************************************/
inline uint32_t uint_ema32(uint32_t mean, uint32_t x, uq0_8_t weight)
{
	if (weight == 0) { return x; }

	if (x > mean) {
		const uint32_t dx = x - mean;
		const uint32_t diff = fp_mulu(weight, dx, 8U);
		return mean + diff;

	} else {
		const uint32_t dx = mean - x;
		const uint32_t diff = fp_mulu(weight, dx, 8U);
		return mean - diff;
	}
}
/*!***************************************************************************
 * @brief   Exponentially weighted moving average using the Q15.16 format.
 *
 * @details Evaluates the moving average (exponentially weighted) for data in
 *          Q15.16 format.
 *
 * @param   mean The previous mean value in Q15.16 format.
 * @param   x The current value to be added to the average Q15.16 format.
 * @param   weight The EMA weight in UQ0.7 format.
 * @return  The new mean value in Q15.16 format.
 *****************************************************************************/
inline int32_t int_ema32(int32_t mean, int32_t x, uq0_8_t weight)
{
	if (weight == 0) { return x; }

	if (x > mean) {
		const uint32_t dx = x - mean;
		const uint32_t diff = fp_mulu(weight, dx, 8U);
		return mean + diff;

	} else {
		const uint32_t dx = mean - x;
		const uint32_t diff = fp_mulu(weight, dx, 8U);
		return mean - diff;
	}
}

/*!***************************************************************************
 * @brief   Exponentially weighted moving average using the Q15.16 format.
 *
 * @details Evaluates the moving average (exponentially weighted) for data in
 *          Q15.16 format.
 *
 * @param   mean The previous mean value in Q15.16 format.
 * @param   x The current value to be added to the average Q15.16 format.
 * @param   weight The EMA weight in UQ0.7 format.
 * @return  The new mean value in Q15.16 format.
 *****************************************************************************/
inline q15_16_t fp_ema16(q15_16_t mean, q15_16_t x, uq0_8_t weight)
{
	return (q15_16_t)int_ema32(mean, x, weight);
}

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* FP_EMA_H */
