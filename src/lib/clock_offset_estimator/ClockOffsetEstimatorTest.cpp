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

#include <gtest/gtest.h>

#include "clock_offset_estimator.hpp"

// The estimator recovers the FC HRT of a remote event from two observables per
// message: the remote send timestamp and the local HRT at receipt. The model
// used throughout these tests is
//
//     local_hrt = (remote_ts + kClockOffset) + transport
//                  \_______ send instant in HRT ______/   \__ CAN latency __/
//
// where kClockOffset is the (constant) epoch difference between FC HRT and the
// remote clock, and transport >= 0 is the one-way bus latency (jittery). The
// estimator tracks the *minimum* observed (local - remote), so toLocal() returns
// the send instant in HRT, biased late by only the minimum transport seen.

namespace
{
constexpr int64_t  kClockOffset = 4'000'000;  // HRT - remote clock [us]
constexpr uint64_t kStartRemote = 1'000'000;  // first remote send time [us]
constexpr uint64_t kPeriod      = 100'000;    // 10 Hz fix cadence [us]

// HRT at which a message sent at remote_us with the given transport arrives.
constexpr uint64_t arrival_hrt(uint64_t remote_us, uint64_t transport_us)
{
	return static_cast<uint64_t>(static_cast<int64_t>(remote_us) + kClockOffset) + transport_us;
}

// HRT of the send instant itself (what toLocal() should converge toward).
constexpr uint64_t send_hrt(uint64_t remote_us)
{
	return static_cast<uint64_t>(static_cast<int64_t>(remote_us) + kClockOffset);
}
} // namespace

TEST(ClockOffsetEstimator, ReturnsZeroWhenRemoteTimestampUnknown)
{
	ClockOffsetEstimator estimator;

	// A node that leaves its send timestamp unset (0) cannot be mapped.
	EXPECT_EQ(estimator.toLocal(0, 5'000'000), 0u);

	// The rejected sample must not corrupt state: a real sample still bootstraps.
	EXPECT_EQ(estimator.toLocal(kStartRemote, arrival_hrt(kStartRemote, 3'000)),
		  arrival_hrt(kStartRemote, 3'000));
}

TEST(ClockOffsetEstimator, FirstSampleMapsToReceiptTime)
{
	ClockOffsetEstimator estimator;

	// With a single observation there is no transport floor to subtract yet, so
	// the best the estimator can do is anchor the remote epoch to receipt time.
	const uint64_t local = arrival_hrt(kStartRemote, 12'000);
	EXPECT_EQ(estimator.toLocal(kStartRemote, local), local);
}

TEST(ClockOffsetEstimator, RecoversSendInstantUpToConstantTransport)
{
	ClockOffsetEstimator estimator;

	// With a constant transport the residual bias is exactly that transport from
	// the very first sample, and the mapping is stable across the whole stream.
	constexpr uint64_t kTransport = 5'000;

	for (unsigned k = 0; k < 5; ++k) {
		const uint64_t remote = kStartRemote + k * kPeriod;
		const uint64_t local  = arrival_hrt(remote, kTransport);

		EXPECT_EQ(estimator.toLocal(remote, local), send_hrt(remote) + kTransport);
	}
}

TEST(ClockOffsetEstimator, ConvergesToMinTransportWithinTwoCycles)
{
	ClockOffsetEstimator estimator;

	// Per-message transport: the first sample is a bad (late) one, so the
	// bootstrap over-estimates the floor. The true floor (3 ms) appears by the
	// third sample and must then survive the 40 ms jitter spike at index 3.
	constexpr uint64_t kTransport[]   = {50'000, 8'000, 3'000, 40'000, 3'500, 3'000};
	constexpr uint64_t kMinTransport  = 3'000;
	// Between floor observations the leaky-min drifts up at kMaxClockDriftPpm
	// (100 ppm); over a handful of 10 Hz cycles that is a few tens of us.
	constexpr uint64_t kLeakSlack     = 100;

	for (unsigned k = 0; k < 6; ++k) {
		const uint64_t remote = kStartRemote + k * kPeriod;
		const uint64_t local  = arrival_hrt(remote, kTransport[k]);

		const hrt_abstime mapped = estimator.toLocal(remote, local);

		// Causal at all times: at/after the nav-send, at/before receipt.
		EXPECT_GE(mapped, send_hrt(remote));
		EXPECT_LE(mapped, local);

		if (k == 0) {
			// Bootstrap: still 50 ms off the true send instant.
			EXPECT_EQ(mapped - send_hrt(remote), kTransport[0]);

		} else if (k >= 2) {
			// Converged (and held): the residual bias has collapsed from 50 ms
			// to the minimum observed transport, and the 40 ms spike at k == 3
			// does not pull the estimate back up.
			EXPECT_GE(mapped, send_hrt(remote) + kMinTransport);
			EXPECT_LE(mapped, send_hrt(remote) + kMinTransport + kLeakSlack);
		}
	}
}

TEST(ClockOffsetEstimator, ResetsOnRemoteClockDiscontinuity)
{
	ClockOffsetEstimator estimator;

	constexpr uint64_t kTransport = 3'000;

	// Converge on a steady stream.
	uint64_t remote = kStartRemote;

	for (unsigned k = 0; k < 4; ++k) {
		remote = kStartRemote + k * kPeriod;
		EXPECT_EQ(estimator.toLocal(remote, arrival_hrt(remote, kTransport)),
			  send_hrt(remote) + kTransport);
	}

	// The node clock steps back 1 s (> kResetThresholdUs) while HRT keeps
	// running: the next send timestamp is 1 s behind the established chain. The
	// resulting lag jump must re-bootstrap rather than be tracked as a new floor.
	const uint64_t hrt_now     = arrival_hrt(remote + kPeriod, kTransport); // continuous HRT
	const uint64_t remote_jump = (remote + kPeriod) - 1'000'000;            // node UTC steps back 1 s

	EXPECT_EQ(estimator.toLocal(remote_jump, hrt_now), hrt_now); // re-bootstrapped to receipt

	// After the step the FC-HRT-to-node offset is 1 s larger; the estimator
	// reconverges on the new baseline and stays causal.
	const uint64_t remote_next = remote_jump + kPeriod;
	const uint64_t hrt_next    = hrt_now + kPeriod;
	const hrt_abstime mapped   = estimator.toLocal(remote_next, hrt_next);

	EXPECT_LE(mapped, hrt_next);
	EXPECT_EQ(mapped, hrt_next); // constant transport again -> maps to receipt
}
