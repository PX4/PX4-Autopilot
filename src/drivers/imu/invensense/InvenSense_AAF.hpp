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
 * Anti-alias filter coefficients shared by the ICM-4268x / IIM-4265x family
 * (datasheet table "GYRO/ACCEL anti-alias filter bandwidth") and the UI
 * filter bandwidth codes of GYRO_ACCEL_CONFIG0.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace InvenSense_AAF
{

struct Coefficients {
	uint8_t delt;        // GYRO_AAF_DELT / ACCEL_AAF_DELT
	uint16_t deltsqr;    // GYRO_AAF_DELTSQR / ACCEL_AAF_DELTSQR
	uint8_t bitshift;    // GYRO_AAF_BITSHIFT / ACCEL_AAF_BITSHIFT
	uint16_t bandwidth_hz;
};

inline constexpr Coefficients kTable[] {
	{ 1,    1, 15,   42}, { 2,    4, 13,   84}, { 3,    9, 12,  126}, { 4,   16, 11,  170},
	{ 5,   25, 10,  213}, { 6,   36, 10,  258}, { 7,   49,  9,  303}, { 8,   64,  9,  348},
	{ 9,   81,  9,  394}, {10,  100,  8,  441}, {11,  122,  8,  488}, {12,  144,  8,  536},
	{13,  170,  8,  585}, {14,  196,  7,  634}, {15,  224,  7,  684}, {16,  256,  7,  734},
	{17,  288,  7,  785}, {18,  324,  7,  837}, {19,  360,  6,  890}, {20,  400,  6,  943},
	{21,  440,  6,  997}, {22,  488,  6, 1051}, {23,  528,  6, 1107}, {24,  576,  6, 1163},
	{25,  624,  6, 1220}, {26,  680,  6, 1277}, {27,  736,  5, 1336}, {28,  784,  5, 1395},
	{29,  848,  5, 1454}, {30,  900,  5, 1515}, {31,  960,  5, 1577}, {32, 1024,  5, 1639},
	{33, 1088,  5, 1702}, {34, 1152,  5, 1766}, {35, 1232,  5, 1830}, {36, 1296,  5, 1896},
	{37, 1376,  4, 1962}, {38, 1440,  4, 2029}, {39, 1536,  4, 2097}, {40, 1600,  4, 2166},
	{41, 1696,  4, 2235}, {42, 1760,  4, 2306}, {43, 1856,  4, 2377}, {44, 1952,  4, 2449},
	{45, 2016,  3, 2522}, {46, 2112,  3, 2596}, {47, 2208,  3, 2671}, {48, 2304,  3, 2746},
	{49, 2400,  3, 2823}, {50, 2496,  3, 2900}, {51, 2592,  3, 2978}, {52, 2720,  3, 3057},
	{53, 2816,  3, 3137}, {54, 2944,  3, 3217}, {55, 3008,  3, 3299}, {56, 3136,  3, 3381},
	{57, 3264,  3, 3464}, {58, 3392,  3, 3548}, {59, 3456,  3, 3633}, {60, 3584,  3, 3718},
	{61, 3712,  3, 3805}, {62, 3840,  3, 3892}, {63, 3968,  3, 3979},
};

// Closest table row to the requested bandwidth.
inline const Coefficients &lookup(uint32_t bandwidth_hz)
{
	const Coefficients *best = &kTable[0];

	for (const auto &c : kTable) {
		const uint32_t d_best = (best->bandwidth_hz > bandwidth_hz) ? best->bandwidth_hz - bandwidth_hz : bandwidth_hz -
					best->bandwidth_hz;
		const uint32_t d = (c.bandwidth_hz > bandwidth_hz) ? c.bandwidth_hz - bandwidth_hz : bandwidth_hz - c.bandwidth_hz;

		if (d < d_best) {
			best = &c;
		}
	}

	return *best;
}

// UI filter bandwidth codes 0..7 (GYRO_ACCEL_CONFIG0 *_UI_FILT_BW) select ODR/N.
inline constexpr uint8_t kUiDivisors[] {2, 4, 5, 8, 10, 16, 20, 40};

// Code whose ODR/N is closest to the requested bandwidth.
inline uint8_t ui_filter_bw_code(uint32_t odr_hz, uint32_t bandwidth_hz)
{
	uint8_t best = 0;
	uint32_t d_best = UINT32_MAX;

	for (size_t i = 0; i < sizeof(kUiDivisors); i++) {
		const uint32_t bw = odr_hz / kUiDivisors[i];
		const uint32_t d = (bw > bandwidth_hz) ? bw - bandwidth_hz : bandwidth_hz - bw;

		if (d < d_best) {
			d_best = d;
			best = static_cast<uint8_t>(i);
		}
	}

	return best;
}

} // namespace InvenSense_AAF
