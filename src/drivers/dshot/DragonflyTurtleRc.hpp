// SPDX-License-Identifier: BSD-3-Clause
// Validity and channel selection for the PX4 RC input pipeline.
#pragma once

#include <cmath>
#include <cstdint>

namespace dragonfly_turtle_rc
{
static constexpr int32_t min_channel = 1; // actual function assignments are checked separately
static constexpr int32_t max_channel = 18;
static constexpr int32_t default_channel = 9;
static constexpr uint64_t max_age_us = 300000;
static constexpr float normalized_channel_limit = 1.05f; // Tolerance around the calibrated [-1, 1] range.

inline bool healthy(uint64_t now, uint64_t timestamp, uint64_t last_valid,
		    bool signal_lost)
{
	// Require a fresh primary timestamp and a valid signal. Some RC publishers
	// leave timestamp_last_valid at zero; check its age when it is nonzero.
	// The caller separately validates the RC-source manual input.
	if (signal_lost || timestamp == 0 || now < timestamp
	    || now - timestamp >= max_age_us) {
		return false;
	}

	return last_valid == 0
	       || (now >= last_valid && now - last_valid < max_age_us);
}

inline float select(const float *values, uint8_t count, int32_t channel)
{
	if (!values || channel < min_channel || channel > max_channel || count < channel) {
		return NAN;
	}

	const float value = values[channel - 1];
	return std::isfinite(value) && std::fabs(value) <= normalized_channel_limit ? value : NAN;
}

} // namespace dragonfly_turtle_rc
