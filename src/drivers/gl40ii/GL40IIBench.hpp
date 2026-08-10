#pragma once

#include <cstdint>

namespace gl40ii
{

// Fifth-order minimum-jerk progress from zero to one.
float minimumJerk(float phase);

// Duration needed to keep the minimum-jerk peak rate at or below peak_rate.
float minimumJerkDuration(float distance, float peak_rate, float minimum_duration);

// Validate the complete relative move without relying on command clamping.
bool benchRangeValid(float start, float delta, float position_limit, float margin);

// Validate positive and negative moves about one captured start position.
bool benchBidirectionalRangeValid(float start, float amplitude, float position_limit, float margin);

// Select the reply-only drives that must receive disable/poll frames. Exclude
// the selected rotor only while it is receiving enable or position commands;
// poll every drive during an inter-motor disabled dwell.
uint8_t benchGuardMask(uint8_t active_mask, uint8_t selected_rotor, bool selected_commanded);

} // namespace gl40ii
