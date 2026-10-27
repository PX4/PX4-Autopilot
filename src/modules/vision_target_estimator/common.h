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
 * @file common.h
 * @brief Definitions common to the position and orientation filters.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <cstdint>

#include <drivers/drv_hrt.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector3.hpp>
#include <uORB/topics/vte_aid_source1d.h>

namespace vision_target_estimator
{

using namespace time_literals;

static constexpr float kMicrosecondsToSeconds = 1e-6f;
static constexpr float kMicrosecondsToMilliseconds = 1e-3f;
static constexpr float kMinVariance = 1e-6f;
static constexpr float kMinObservationNoise = 1e-2f;
static constexpr float kMinNisThreshold = 0.1f;
static constexpr float kDefaultNisThreshold = 3.84f;
static constexpr float kGravityMps2 = 9.80665f;
static constexpr float kMinInnovationVariance = kMinVariance;

constexpr hrt_abstime kWarnThrottleIntervalUs = 20_s;
constexpr hrt_abstime kEstimatorUpdatePeriodUs = 20_ms;
constexpr hrt_abstime kOosmMinTimeUs = kEstimatorUpdatePeriodUs;
constexpr hrt_abstime kOosmMaxTimeUs = 500_ms;
constexpr int kOosmHistorySize = static_cast<int>(kOosmMaxTimeUs / kEstimatorUpdatePeriodUs);
// Observations kept per position axis for replays: 50 Hz vision and three 10 Hz GNSS streams need 40
// over the window, plus margin. Beyond that the oldest are dropped and the replay window shrinks.
constexpr int kOosmMaxStoredObs = 48;

// Fusion status values are defined as message constants in VteAidSource1d.msg / VteAidSource3d.msg
// (both messages declare the same STATUS_* values). Using vte_aid_source1d_s here keeps a single
// source of truth for the enum values shared between the filters and the logged uORB topics.
using FusionStatus = vte_aid_source1d_s;

using TimeSource = hrt_abstime(*)();

inline hrt_abstime defaultNowUs()
{
	return hrt_absolute_time();
}

class TimeSourceProvider
{
public:
	static hrt_abstime nowUs()
	{
		return timeSourceStorage()();
	}

#if defined(VTE_ENABLE_TEST_HOOKS)
	static void setTimeSourceForTest(TimeSource newTimeSource)
	{
		timeSourceStorage() = newTimeSource ? newTimeSource : defaultNowUs;
	}

	static void resetTimeSourceForTest()
	{
		timeSourceStorage() = defaultNowUs;
	}
#endif

private:
	static TimeSource &timeSourceStorage()
	{
		static TimeSource activeTimeSource = defaultNowUs;
		return activeTimeSource;
	}
};

inline hrt_abstime nowUs()
{
	return TimeSourceProvider::nowUs();
}

// Velocity aiding must not refresh position validity; relative position also backs EKF2 aiding.
enum class FusionSource : uint8_t { Primary, Relative, Untracked };

struct FusionTimes {
	uint64_t last_fused{0};
	uint64_t last_relative_fused{0};

	void record(FusionSource source, uint64_t capture_time)
	{
		if (source != FusionSource::Untracked && capture_time > last_fused) {
			last_fused = capture_time;
		}

		if (source == FusionSource::Relative && capture_time > last_relative_fused) {
			last_relative_fused = capture_time;
		}
	}
};

struct FusionResult {
	uint8_t status{FusionStatus::STATUS_IDLE};
	float innov{0.f};
	float innov_var{0.f};
	float test_ratio{-1.f}; // Normalized NIS (beta / threshold)
	uint8_t history_steps{0};
	uint8_t replayed_count{0}; // Stored observations gated again after this one
	uint8_t replay_revised_count{0}; // Those whose gate decision changed
	float replay_test_ratio_max{-1.f}; // Highest test ratio among those accepted before, -1 if none
};

static inline bool hasTimedOutAt(const hrt_abstime now, const hrt_abstime ts, const hrt_abstime timeout_us)
{
	// Zero timestamps are treated as invalid / timed out.
	if (ts == 0) {
		return true;
	}

	// Future timestamps are considered invalid -> timed out
	if (ts > now) {
		return true;
	}

	return (now - ts) >= timeout_us;
}

static inline bool hasTimedOut(const hrt_abstime ts, const hrt_abstime timeout_us)
{
	return hasTimedOutAt(nowUs(), ts, timeout_us);
}

// Clock-sync jitter can stamp an external sample slightly after now: treat it as current, like the
// OOSM fast path. Returns false for samples too far in the future to trust.
static inline bool alignFutureSampleTime(hrt_abstime &ts, const hrt_abstime now)
{
	if (ts > now) {
		if ((ts - now) >= kOosmMinTimeUs) {
			return false;
		}

		ts = now;
	}

	return true;
}

// Rate limit a warning to one per kWarnThrottleIntervalUs.
inline bool shouldEmitWarning(hrt_abstime &last_warn)
{
	const hrt_abstime now = nowUs();

	if ((last_warn == 0) || (now < last_warn) || ((now - last_warn) > kWarnThrottleIntervalUs)) {
		last_warn = now;
		return true;
	}

	return false;
}

constexpr inline int64_t signedTimeDiffUs(const hrt_abstime newer, const hrt_abstime older)
{
	return static_cast<int64_t>(newer) - static_cast<int64_t>(older);
}

static constexpr float sq(float var) { return var * var; }

union SensorFusionMaskU {
	struct {
		uint16_t use_target_gps_pos   : 1;  ///< bit 0
		uint16_t use_uav_gps_vel      : 1;  ///< bit 1
		uint16_t use_vision_pos       : 1;  ///< bit 2
		uint16_t use_mission_pos      : 1;  ///< bit 3
		uint16_t use_target_gps_vel   : 1;  ///< bit 4
		uint16_t use_home_pos         : 1;  ///< bit 5
		uint16_t reserved             : 10; ///< bits 6..15 (future use)
	} flags;

	uint16_t value;
};
static_assert(sizeof(SensorFusionMaskU) == 2, "Unexpected packing");

struct Vector3fStamped {
	hrt_abstime timestamp = 0;
	bool valid = false;
	matrix::Vector3f xyz{};
};

} // namespace vision_target_estimator
