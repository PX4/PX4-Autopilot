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
 * @file InvenSense_AAF.hpp
 *
 * Anti-alias filter presets for the ICM-4268x / IIM-4265x family, selected
 * with the drivers' -B <hz> start option. Coefficients from the datasheet
 * "anti-alias filter bandwidth" table; the UI filter code is the ODR/N
 * setting of GYRO_ACCEL_CONFIG0 at the drivers' 8 kHz ODR.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace InvenSense_AAF
{

struct Preset {
	uint16_t bandwidth_hz;
	uint8_t delt;        // *_AAF_DELT
	uint16_t deltsqr;    // *_AAF_DELTSQR
	uint8_t bitshift;    // *_AAF_BITSHIFT
	uint8_t ui_filt_bw;  // *_UI_FILT_BW code: 6 = ODR/20 (400 Hz), 7 = ODR/40 (200 Hz)
};

// The chip default is 585 Hz with a 1st-order UI filter at ODR/2; these are the
// alternatives for boards that decimate to 1 kHz or below.
inline constexpr Preset kPresets[] {
	{126, 3,  9, 12, 7},
	{258, 6, 36, 10, 7},
	{394, 9, 81,  9, 6},
};

inline const Preset *preset(uint32_t bandwidth_hz)
{
	for (const auto &p : kPresets) {
		if (p.bandwidth_hz == bandwidth_hz) {
			return &p;
		}
	}

	return nullptr;
}

} // namespace InvenSense_AAF
