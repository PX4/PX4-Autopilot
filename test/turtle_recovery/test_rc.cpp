// SPDX-License-Identifier: BSD-3-Clause

// Regression for RC publishers that leave timestamp_last_valid at zero.
#include "DragonflyTurtleRc.hpp"
#include "DragonflyTurtleInput.hpp"
#include <cassert>
#include <cstdio>
#include <cstdint>
int main()
{
	using dragonfly_turtle_rc::healthy;
	const uint64_t now = 664602747;
	const uint64_t sample = 664598155; // Sample is 4.592 ms old.
	assert(healthy(now, sample, 0, false));
	assert(healthy(now, sample, sample, false));
	const uint64_t last_values[2] = {0, sample};

	for (uint64_t last : last_values) {
		assert(!healthy(now, sample, last, true));
		assert(!healthy(now, 0, last, false));
		assert(!healthy(now, now + 1, last, false));
		assert(!healthy(now, now - 300000, last, false));
		assert(!healthy(now, now - 300001, last, false));
		assert(healthy(now, now - 299999, last, false));
	}

	assert(!healthy(now, sample, now + 1, false));
	assert(!healthy(now, sample, now - 300000, false));
	assert(!healthy(now, sample, now - 300001, false));
	assert(healthy(now, sample, now - 299999, false));
	assert(!healthy(0, 0, 0, false));
	assert(!healthy(0, 1, 0, false));
	assert(!healthy(1000, UINT64_MAX, 0, false));
	const float channels[18] = {0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, .06599f, -1, -1, -1, -1, 0, 0};
	assert(dragonfly_turtle_rc::select(channels, 16, 9) < -.99f);
	assert(!std::isfinite(dragonfly_turtle_rc::select(channels, 8, 9)));
	assert(!std::isfinite(dragonfly_turtle_rc::select(nullptr, 16, 9)));
	// Driver supplies non-AUX RC_MAP_* channels, using one-based values.
	int32_t mapped_channels[6] = {1, 2, 3, 4, 5, 10};
	assert(!dragonfly_trtl::channelConflict(mapped_channels, 6, 9));
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 10));
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 5));
	mapped_channels[4] = 9;
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 9));

	for (auto &channel : mapped_channels) {
		channel = 0;
	}

	assert(!dragonfly_trtl::channelConflict(mapped_channels, 6, 9));
	assert(dragonfly_trtl::channelConflict(nullptr, 6, 9));
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 0));
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 19));
	mapped_channels[0] = -1;
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 9));
	mapped_channels[0] = 19;
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 9));
	mapped_channels[0] = 18;
	assert(dragonfly_trtl::channelConflict(mapped_channels, 6, 18));
	assert(!dragonfly_trtl::channelConflict(mapped_channels, 6, 17));
	// RC_MAP_FLTM_BTN stores a bitmask, not a single channel number.
	using dragonfly_trtl::modeButtonConflict;
	assert(modeButtonConflict(4, 3));
	assert(!modeButtonConflict(4, 4));
	const int32_t two_buttons = (1 << 2) | (1 << 16);
	assert(modeButtonConflict(two_buttons, 3));
	assert(modeButtonConflict(two_buttons, 17));
	assert(!modeButtonConflict(two_buttons, 4));

	for (int32_t channel = 1; channel <= 18; ++channel) {
		const int32_t selected = 1 << (channel - 1);
		assert(modeButtonConflict(selected, channel));
		assert(!modeButtonConflict(selected, channel == 18 ? 1 : channel + 1));
		assert(!modeButtonConflict(0, channel));
		assert(modeButtonConflict((1 << 18) - 1, channel));
		assert(modeButtonConflict(1 << 18, channel));
	}

	assert(modeButtonConflict(-1, 9));
	assert(modeButtonConflict(0, 0));
	assert(modeButtonConflict(0, 19));
	std::puts("PASS: RC timestamps, channel validity, function conflicts and flight-mode button masks");
}
