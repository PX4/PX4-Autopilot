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
 * @file ImuSamplePhase.hpp
 * @brief Phase detection for twofold repeated accelerometer samples in fixed-stride FIFOs.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace imu
{

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
	    || size / stride == 0
	    || size % stride != 0
	    || short_phase > 1) {
		return false;
	}

	const size_t frames = size / stride;

	if (frames == 1) {
		first_sample = 1;

		return true;
	}

	uint8_t first = short_phase;

	if (frames == 3) {
		const bool pair01 = memcmp(data, data + stride, kAccelBytes) == 0;
		const bool pair12 = memcmp(data + stride, data + 2 * stride, kAccelBytes) == 0;

		if (pair01 && !pair12) {
			first = 1;

		} else if (pair12 && !pair01) {
			first = 0;

		} else if (!pair01 && !pair12) {
			return false;
		}

	} else if (frames >= 4) {
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

} // namespace imu
