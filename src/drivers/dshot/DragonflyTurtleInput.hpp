// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <cstdint>
#include <cstddef>
namespace dragonfly_trtl
{
inline bool fresh(uint64_t now, uint64_t stamp, uint64_t limit)
{
	return stamp && now >= stamp && now - stamp < limit;
}
// RCUpdate may suppress unchanged stick samples for more than 300 ms. Keep
// publication freshness separate from sample age; raw RC health is checked
// independently by the caller. All bounds reject absent and future timestamps.
inline bool manualInputFresh(uint64_t now, uint64_t publication, uint64_t sample)
{
	static constexpr uint64_t publication_age_us = 300000;
	static constexpr uint64_t sample_age_us = 750000;
	return fresh(now, publication, publication_age_us) && fresh(now, sample, sample_age_us);
}
inline float vertical(const float q[4])
{
	const float n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

	if (!std::isfinite(n) || n < 0.9f || n > 1.1f) {
		return NAN;
	}

	return 1.f - 2.f * (q[1] * q[1] + q[2] * q[2]) / n;
}
// The caller supplies non-AUX RC_MAP_* parameter values. They use one-based
// channel numbers and zero for an unassigned function.
inline bool channelConflict(const int32_t *mapped_channels, size_t count, int32_t channel)
{
	if (channel < 1 || channel > 18 || !mapped_channels) {
		return true;
	}

	for (size_t i = 0; i < count; ++i) {
		if (mapped_channels[i] < 0 || mapped_channels[i] > 18 || mapped_channels[i] == channel) {
			return true;
		}
	}

	return false;
}
// RC_MAP_FLTM_BTN is a bitmask, unlike the scalar RC_MAP_* channel assignments.
inline bool modeButtonConflict(int32_t button_mask, int32_t channel)
{
	static constexpr unsigned channel_count = 18;
	static constexpr uint32_t valid_channels = (uint32_t{1} << channel_count) - 1;

	if (channel < 1 || channel > int32_t(channel_count) || button_mask < 0
	    || (uint32_t(button_mask) & ~valid_channels)) {
		return true;
	}

	return (uint32_t(button_mask) & (uint32_t{1} << (channel - 1))) != 0;
}

inline bool quadCorner(unsigned c, float x, float y, float ax, float ay, float az)
{
	if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
		return false;
	}

	const float norm = std::sqrt(ax * ax + ay * ay + az * az);

	if (!std::isfinite(norm) || norm < 0.000001f) {
		return false;
	}

	ax /= norm;
	ay /= norm;
	az /= norm;
	const bool front = c < 2, right = c == 1 || c == 3;
	static constexpr float position_tolerance_m = 0.0001f;
	return (front ? x > position_tolerance_m : x < -position_tolerance_m) && (right ? y > position_tolerance_m : y < -position_tolerance_m)
	       && std::fabs(ax) < 0.01f && std::fabs(ay) < 0.01f && az < -0.99f;
}
} // namespace dragonfly_trtl
