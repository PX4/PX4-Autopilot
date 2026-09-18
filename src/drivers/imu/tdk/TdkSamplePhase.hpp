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
 * @brief TDK-specific selection of repeated accelerometer samples.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace tdk
{

/**
 * @brief Locate the phase of accel samples repeated twice in fixed-stride frames.
 * @param[in] data Frames with accel occupying their first six bytes; non-null.
 * @param[in] size Received byte count, divisible by stride and containing at least two frames.
 * @param[in] stride Frame size in bytes, at least six.
 * @param[out] first_sample First accel frame index (0 or 1); unchanged on failure.
 * @param[in] short_phase Phase for two/three frames: 0 for MPU9250, 1 for the other repeated-frame paths.
 * @return True when a phase can be selected; false for invalid input or inconsistent initial repetitions.
 * @note Four or more frames inspect the first four frames only.
 * MPU6000's eightfold repetition is intentionally not part of this format.
 */
inline bool repeatedAccelPhase(
	const uint8_t *data,
	size_t size,
	size_t stride,
	uint8_t &first_sample,
	uint8_t short_phase = 1)
{
	constexpr size_t kAccelBytes { 3 * sizeof(int16_t) };

	if (!data
	    || stride < kAccelBytes
	    || size / stride < 2
	    || size % stride != 0
	    || short_phase > 1) {
		return false;
	}

	uint8_t first = short_phase;

	if (size / stride >= 4) {
		if (memcmp(data, data + stride, kAccelBytes) == 0
		    && memcmp(data + 2 * stride, data + 3 * stride, kAccelBytes) == 0) {
			first = 1;

		} else if (memcmp(data + stride, data + 2 * stride, kAccelBytes) == 0) {
			first = 0;

		} else {
			return false;
		}
	}

	first_sample = first;

	return true;
}

} // namespace tdk
