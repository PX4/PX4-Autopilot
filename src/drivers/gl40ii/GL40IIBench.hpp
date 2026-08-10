#pragma once

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

} // namespace gl40ii
