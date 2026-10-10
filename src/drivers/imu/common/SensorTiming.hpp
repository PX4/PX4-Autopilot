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
 * @file
 * @brief Counter progress and bounded retry timing, independent of transport and sample format.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace imu
{

namespace detail
{
/// @cond INTERNAL
// Explicit type contracts keep this mechanism usable without libstdc++ on NuttX.
template<typename Counter>
constexpr bool kIsSampleCounter { false };
template<>
constexpr bool kIsSampleCounter<uint8_t> { true };
template<>
constexpr bool kIsSampleCounter<uint16_t> { true };
template<>
constexpr bool kIsSampleCounter<uint32_t> { true };
template<>
constexpr bool kIsSampleCounter<uint64_t> { true };
/// @endcond
} // namespace detail

/** Track validated sample progress in host monotonic microseconds, not compensated sample time. */
template<typename Counter>
class SampleProgress
{
	static_assert(detail::kIsSampleCounter<Counter>, "Sample counter must be an unsigned integer");
public:
	/** Start a new acquisition epoch; every counter value, including zero, is initially valid. */
	void reset(uint64_t now)
	{
		_last_update = now;
		_initialized = false;
	}

	/** Call only after validation. Equality means duplicate; unsigned wrap is a valid change. */
	[[nodiscard]] bool observe(Counter counter, uint64_t now)
	{
		if (_initialized && counter == _counter) {
			return false;
		}

		_counter = counter;
		_last_update = now;
		_initialized = true;
		return true;
	}

	/** No side effects; repeated reads must not extend a stalled sensor's deadline. */
	bool expired(uint64_t now, uint64_t timeout) const { return now - _last_update >= timeout; }
	uint64_t lastUpdate() const { return _last_update; }

private:
	uint64_t _last_update {};
	Counter _counter {};
	bool _initialized {};
};

/** Select a bounded delay; callers own the policy values and attempt counter. */
template<typename Duration, size_t count>
constexpr Duration retryDelay(unsigned attempt, const Duration(&delays)[count])
{
	static_assert(count > 0, "Retry schedule must not be empty");
	return delays[attempt < count ? attempt : count - 1];
}

} // namespace imu
