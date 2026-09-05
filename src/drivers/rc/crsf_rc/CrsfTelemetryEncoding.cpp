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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "CrsfTelemetryEncoding.hpp"

#include <cmath>

namespace
{

// Lower calibration point used to scale fallback RSSI into a percentage [dBm].
constexpr float kRssiMinimumDbm{-130.f};

// CRSF shifts GPS altitude so negative mean-sea-level heights fit the unsigned field [m].
constexpr int32_t kGpsAltitudeOffsetM{1000};

// Upper saturation limit for consumed capacity in the unsigned 24-bit CRSF field [mAh].
constexpr uint32_t kFuelMaximumMah{0xFFFFFF};

} // namespace

uint8_t crsfRssiDbmToPercent(float rssi_dbm, uint8_t maximum)
{
	if (std::isnan(rssi_dbm)) {
		return 0;
	}

	const float scaled_rssi_percent = (1.f - rssi_dbm / kRssiMinimumDbm) * maximum;

	if (scaled_rssi_percent <= 0.f) {
		return 0;

	} else if (scaled_rssi_percent >= maximum) {
		return maximum;
	}

	return static_cast<uint8_t>(scaled_rssi_percent);
}

uint16_t crsfGpsAltitudeToWire(double altitude_msl_m)
{
	if (std::isnan(altitude_msl_m)) {
		return 0;
	}

	if (altitude_msl_m <= -kGpsAltitudeOffsetM) {
		return 0;

	} else if (altitude_msl_m >= UINT16_MAX - kGpsAltitudeOffsetM) {
		return UINT16_MAX;
	}

	// Quantize before applying the offset to retain the established whole-meter encoding.
	return static_cast<uint16_t>(static_cast<int32_t>(altitude_msl_m) + kGpsAltitudeOffsetM);
}

uint32_t crsfFuelToWire(float fuel_mah)
{
	if (std::isnan(fuel_mah) || fuel_mah <= 0.f) {
		return 0;

	} else if (fuel_mah >= kFuelMaximumMah) {
		return kFuelMaximumMah;
	}

	return static_cast<uint32_t>(fuel_mah);
}

bool crsfWriteComplete(ssize_t result, size_t expected)
{
	return result >= 0 && static_cast<size_t>(result) == expected;
}
