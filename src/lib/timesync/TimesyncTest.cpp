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
 * Timesync convergence on a simulated remote clock (7 ppm drift, +-0.5 ms noise) with a step or ramp correction.
 */

#include <gtest/gtest.h>

#include "Timesync.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace
{

struct Scenario {
	double rate_hz{1.0};                 // timesync rate after the initial burst
	bool burst{true};                    // uXRCE-DDS: fast exchange until converged
	double disturbance_us{0.0};          // size of the remote clock correction
	double disturbance_start_s{180.0};
	double disturbance_duration_s{0.0};  // 0: step, > 0: ramp
	double duration_s{1800.0};
};

struct Result {
	double max_error_before_disturbance_us{0.0};
	double last_error_above_10ms_s{0.0}; // time after the disturbance start
	double last_error_above_1ms_s{0.0};
};

Result runScenario(const Scenario &scenario)
{
	static constexpr double OFFSET_US = -1000e6; // remote clock 1000 s ahead
	static constexpr double DRIFT_PPM = 7.0;
	static constexpr int64_t NOISE_US = 500;
	static constexpr double BURST_INTERVAL_US = 15e3;

	std::mt19937 rng(1);
	Timesync timesync;
	Result result{};
	double time_us = 1e6;

	for (uint32_t sample = 0; time_us < scenario.duration_s * 1e6; sample++) {
		const bool in_burst = scenario.burst && (sample < CONVERGENCE_WINDOW);
		time_us += in_burst ? BURST_INTERVAL_US : 1e6 / scenario.rate_hz;
		const double time_s = time_us * 1e-6;
		const double since_disturbance_s = time_s - scenario.disturbance_start_s;

		double disturbance_us = 0.0;

		if (since_disturbance_s >= 0.0) {
			const double progress = (scenario.disturbance_duration_s > 0.0)
						? std::min(1.0, since_disturbance_s / scenario.disturbance_duration_s) : 1.0;
			disturbance_us = progress * scenario.disturbance_us;
		}

		const double true_offset_us = OFFSET_US + DRIFT_PPM * time_s + disturbance_us;
		const int64_t noise_us = static_cast<int64_t>(rng() % (2 * NOISE_US + 1)) - NOISE_US;
		const int64_t observed_offset_us = llround(true_offset_us) + noise_us;

		// Zero round-trip time
		const uint64_t now_us = static_cast<uint64_t>(time_us);
		const int64_t remote_ns = (static_cast<int64_t>(now_us) - observed_offset_us) * 1000;
		timesync.update(now_us, remote_ns, static_cast<int64_t>(now_us) * 1000);

		if (!timesync.sync_converged()) {
			continue;
		}

		const double error_us = std::fabs(static_cast<double>(timesync.offset()) - true_offset_us);

		if (since_disturbance_s < 0.0) {
			if (time_s > 60.0) {
				result.max_error_before_disturbance_us = std::max(result.max_error_before_disturbance_us, error_us);
			}

		} else {
			if (error_us > 10000.0) {
				result.last_error_above_10ms_s = since_disturbance_s;
			}

			if (error_us > 1000.0) {
				result.last_error_above_1ms_s = since_disturbance_s;
			}
		}
	}

	return result;
}

void expectRecovery(const Result &result)
{
	EXPECT_LE(result.last_error_above_10ms_s, 30.0);
	EXPECT_LE(result.last_error_above_1ms_s, 300.0);
}

} // namespace

TEST(TimesyncTest, SteadyStateAccuracy)
{
	Scenario scenario{};
	scenario.disturbance_start_s = 1e9; // never
	const Result result = runScenario(scenario);
	EXPECT_LT(result.max_error_before_disturbance_us, 1500.0);
}

TEST(TimesyncTest, RecoversFromRemoteClockStepForward)
{
	Scenario scenario{};
	scenario.disturbance_us = 60e3;
	expectRecovery(runScenario(scenario));
}

TEST(TimesyncTest, RecoversFromRemoteClockStepBackward)
{
	Scenario scenario{};
	scenario.disturbance_us = -90e3;
	expectRecovery(runScenario(scenario));
}

TEST(TimesyncTest, RecoversFromRemoteClockRamp)
{
	Scenario scenario{};
	scenario.disturbance_us = 60e3;
	scenario.disturbance_duration_s = 60.0;
	expectRecovery(runScenario(scenario));
}

TEST(TimesyncTest, IndependentOfSampleRate)
{
	// MAVLink: constant 10 Hz, no burst
	Scenario scenario{};
	scenario.rate_hz = 10.0;
	scenario.burst = false;
	scenario.disturbance_us = 60e3;
	const Result result = runScenario(scenario);
	EXPECT_LT(result.max_error_before_disturbance_us, 1500.0);
	expectRecovery(result);
}
