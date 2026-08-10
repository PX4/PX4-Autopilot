#include "GL40IIBench.hpp"

#include <cmath>

namespace gl40ii
{

float minimumJerk(float phase)
{
	if (phase <= 0.f) {
		return 0.f;
	}

	if (phase >= 1.f) {
		return 1.f;
	}

	const float phase2 = phase * phase;
	const float phase3 = phase2 * phase;
	return phase3 * (10.f - 15.f * phase + 6.f * phase2);
}

float minimumJerkDuration(float distance, float peak_rate, float minimum_duration)
{
	if (!std::isfinite(distance) || !std::isfinite(peak_rate) || peak_rate <= 0.f
	    || !std::isfinite(minimum_duration) || minimum_duration <= 0.f) {
		return NAN;
	}

	// The maximum derivative of 10u^3 - 15u^4 + 6u^5 is 1.875.
	const float rate_limited_duration = 1.875f * fabsf(distance) / peak_rate;
	return rate_limited_duration > minimum_duration ? rate_limited_duration : minimum_duration;
}

bool benchRangeValid(float start, float delta, float position_limit, float margin)
{
	if (!std::isfinite(start) || !std::isfinite(delta) || !std::isfinite(position_limit)
	    || !std::isfinite(margin) || position_limit <= 0.f || margin < 0.f || margin >= position_limit) {
		return false;
	}

	const float usable_limit = position_limit - margin;
	return fabsf(start) <= usable_limit && fabsf(start + delta) <= usable_limit;
}

bool benchBidirectionalRangeValid(float start, float amplitude, float position_limit, float margin)
{
	if (!std::isfinite(amplitude) || amplitude < 0.f) {
		return false;
	}

	return benchRangeValid(start, amplitude, position_limit, margin)
	       && benchRangeValid(start, -amplitude, position_limit, margin);
}

} // namespace gl40ii
