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

#include "DShotDirection.hpp"

namespace
{
constexpr uint16_t Motors = 0xf;
constexpr uint64_t Start = 100;
constexpr uint64_t Restore = Start + 2000000;
// From the first command frame to the last one of the final burst.
constexpr uint64_t BurstTime = DShotDirection::CommandBursts * (DShotDirection::CommandRepetitions - 1)
			       * DShotDirection::CommandInterval + (DShotDirection::CommandBursts - 1) * DShotDirection::BurstGap;

void arm(DShotDirection &direction, uint64_t start = Start)
{
	direction.observeArming(start - 2, false);
	direction.observeArming(start - 1, true);
}

bool beginReversal(DShotDirection &direction)
{
	arm(direction);
	return direction.requestReverse(Start, Start, Start - 1, 1, Motors);
}

// Sends the remaining bursts once the one in progress ended at 'now'. Returns the time of the last frame.
uint64_t finishBursts(DShotDirection &direction, uint64_t now, unsigned remaining)
{
	for (unsigned burst = 0; burst < remaining; ++burst) {
		EXPECT_EQ(direction.phase(), DShotDirection::Phase::Gap);
		EXPECT_TRUE(direction.pending());
		direction.update(now + DShotDirection::BurstGap - 1);
		EXPECT_FALSE(direction.commandDue(now + DShotDirection::BurstGap - 1));
		now += DShotDirection::BurstGap;
		direction.update(now);

		for (unsigned frame = 0; frame < DShotDirection::CommandRepetitions; ++frame) {
			if (frame > 0) {
				now += DShotDirection::CommandInterval;
			}

			EXPECT_TRUE(direction.commandDue(now));
			direction.commandAttempted(now, direction.mask());
		}
	}

	EXPECT_EQ(direction.phase(), DShotDirection::Phase::Settle);
	return now;
}

void submitBurst(DShotDirection &direction, uint64_t start)
{
	direction.update(start + DShotDirection::StopTime);

	for (unsigned frame = 0; frame < DShotDirection::CommandRepetitions; ++frame) {
		const uint64_t now = start + DShotDirection::StopTime + frame * DShotDirection::CommandInterval;
		ASSERT_TRUE(direction.commandDue(now));
		direction.commandAttempted(now, direction.mask());
	}

	const uint64_t first_burst_end = start + DShotDirection::StopTime + (DShotDirection::CommandRepetitions - 1)
					 * DShotDirection::CommandInterval;
	ASSERT_EQ(finishBursts(direction, first_burst_end, DShotDirection::CommandBursts - 1),
		  start + DShotDirection::StopTime + BurstTime);
}

void completeCommand(DShotDirection &direction, uint64_t start)
{
	submitBurst(direction, start);
	direction.update(start + DShotDirection::StopTime + BurstTime + DShotDirection::SettleTime);
	ASSERT_FALSE(direction.pending());
}
}

TEST(DShotDirection, StartupRequiresObservedDisarm)
{
	DShotDirection direction;
	direction.observeArming(0, false);
	direction.observeArming(Start - 1, true);
	EXPECT_FALSE(direction.requestReverse(Start, Start, Start - 1, 1, Motors));
	EXPECT_FALSE(direction.cancel(Start));
	direction.update(Start + DShotDirection::StopTime);
	EXPECT_FALSE(direction.active());
	EXPECT_FALSE(direction.commandDue(Start + DShotDirection::StopTime));
	EXPECT_EQ(direction.phase(), DShotDirection::Phase::Idle);
}

TEST(DShotDirection, StopAndCommandSpacing)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	direction.update(Start + DShotDirection::StopTime - 1);
	EXPECT_FALSE(direction.commandDue(Start + DShotDirection::StopTime - 1));
	direction.update(Start + DShotDirection::StopTime);
	ASSERT_TRUE(direction.commandDue(Start + DShotDirection::StopTime));
	direction.commandAttempted(Start + DShotDirection::StopTime, Motors);
	EXPECT_FALSE(direction.commandDue(Start + DShotDirection::StopTime + DShotDirection::CommandInterval - 1));
	EXPECT_TRUE(direction.commandDue(Start + DShotDirection::StopTime + DShotDirection::CommandInterval));
}

TEST(DShotDirection, SkippedAndPartialReverseAttemptsDoNotCountAsRepetitions)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	uint64_t now = Start + DShotDirection::StopTime;
	direction.update(now);
	ASSERT_TRUE(direction.commandDue(now));
	direction.commandAttempted(now, 0);
	EXPECT_EQ(direction.reversedMask(), 0u);
	EXPECT_FALSE(direction.commandDue(now + DShotDirection::CommandInterval - 1));

	now += DShotDirection::CommandInterval;
	ASSERT_TRUE(direction.commandDue(now));
	direction.commandAttempted(now, 0x3);
	EXPECT_EQ(direction.reversedMask(), Motors);
	EXPECT_FALSE(direction.commandDue(now + DShotDirection::CommandInterval - 1));

	for (unsigned frame = 0; frame < DShotDirection::CommandRepetitions; ++frame) {
		now += DShotDirection::CommandInterval;
		ASSERT_TRUE(direction.commandDue(now));
		direction.commandAttempted(now, Motors);
	}

	finishBursts(direction, now, DShotDirection::CommandBursts - 1);
}

TEST(DShotDirection, RestorationKeepsMaskThroughSettle)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	completeCommand(direction, Start);
	EXPECT_TRUE(direction.active());
	EXPECT_EQ(direction.reversedMask(), Motors);

	direction.observeArming(Restore, false);
	ASSERT_TRUE(direction.cancel(Restore));
	ASSERT_TRUE(direction.restoring());
	submitBurst(direction, Restore);
	const uint64_t settle_end = Restore + DShotDirection::StopTime + BurstTime + DShotDirection::SettleTime;
	direction.update(settle_end - 1);
	EXPECT_TRUE(direction.pending());
	EXPECT_EQ(direction.reversedMask(), Motors);
	direction.update(settle_end);
	EXPECT_FALSE(direction.active());
	EXPECT_EQ(direction.reversedMask(), 0u);
	EXPECT_EQ(direction.requestId(), 1u);
}

TEST(DShotDirection, IncompleteRestoreKeepsAllReversedMotorsPending)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	completeCommand(direction, Start);
	direction.observeArming(Restore, false);
	ASSERT_TRUE(direction.cancel(Restore));
	uint64_t now = Restore + DShotDirection::StopTime;
	direction.update(now);

	for (unsigned frame = 1; frame < DShotDirection::CommandRepetitions; ++frame) {
		ASSERT_TRUE(direction.commandDue(now));
		direction.commandAttempted(now, Motors);
		now += DShotDirection::CommandInterval;
	}

	direction.commandAttempted(now, 0);
	now += DShotDirection::CommandInterval;
	direction.commandAttempted(now, 0x3);
	now += DShotDirection::SettleTime;
	direction.update(now);
	EXPECT_EQ(direction.phase(), DShotDirection::Phase::Command);
	EXPECT_EQ(direction.reversedMask(), Motors);

	ASSERT_TRUE(direction.commandDue(now));
	direction.commandAttempted(now, Motors);
	now = finishBursts(direction, now, DShotDirection::CommandBursts - 1);
	direction.update(now + DShotDirection::SettleTime - 1);
	EXPECT_EQ(direction.reversedMask(), Motors);
	direction.update(now + DShotDirection::SettleTime);
	EXPECT_FALSE(direction.active());
	EXPECT_EQ(direction.reversedMask(), 0u);
}

TEST(DShotDirection, EveryBurstIsSentAfterStopFrames)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	uint64_t now = Start;
	unsigned frames = 0;
	unsigned bursts = 0;
	unsigned gaps = 0;
	auto phase = direction.phase();

	// Step at a fast output rate and count what the driver would send.
	while (direction.pending() && now < Start + 10000000) {
		direction.update(now);

		if (direction.phase() != phase) {
			phase = direction.phase();
			bursts += phase == DShotDirection::Phase::Command;
		}

		if (direction.commandDue(now)) {
			direction.commandAttempted(now, Motors);
			++frames;
		}

		if (direction.phase() != phase) {
			phase = direction.phase();
			gaps += phase == DShotDirection::Phase::Gap;
		}

		now += 250;
	}

	EXPECT_EQ(frames, static_cast<unsigned>(DShotDirection::CommandBursts * DShotDirection::CommandRepetitions));
	EXPECT_EQ(bursts, static_cast<unsigned>(DShotDirection::CommandBursts));
	EXPECT_EQ(gaps, static_cast<unsigned>(DShotDirection::CommandBursts - 1));
	EXPECT_EQ(direction.reversedMask(), Motors);
	EXPECT_EQ(direction.phase(), DShotDirection::Phase::Complete);
	// One stop interval, every burst and gap, and the settle time, each rounded up to the 250 us step.
	EXPECT_LT(now - Start, DShotDirection::StopTime + BurstTime + DShotDirection::SettleTime + 20000);
}

TEST(DShotDirection, DisarmDuringGapRestoresDirection)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	uint64_t now = Start + DShotDirection::StopTime;
	direction.update(now);

	for (unsigned frame = 0; frame < DShotDirection::CommandRepetitions; ++frame) {
		direction.commandAttempted(now, Motors);
		now += DShotDirection::CommandInterval;
	}

	ASSERT_EQ(direction.phase(), DShotDirection::Phase::Gap);
	direction.observeArming(now, false);
	ASSERT_TRUE(direction.cancel(now));
	EXPECT_TRUE(direction.restoring());
	EXPECT_EQ(direction.phase(), DShotDirection::Phase::Stop);
	completeCommand(direction, now);
	EXPECT_EQ(direction.reversedMask(), 0u);
}

TEST(DShotDirection, DisarmCancelsReverseWhenNoTargetWasSubmitted)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	const uint64_t command = Start + DShotDirection::StopTime;
	direction.update(command);
	ASSERT_TRUE(direction.commandDue(command));
	direction.commandAttempted(command, 0x10);

	direction.observeArming(command + 1, false);
	ASSERT_TRUE(direction.cancel(command + 1));
	EXPECT_EQ(direction.requestId(), 1u);
	EXPECT_EQ(direction.reversedMask(), 0u);
	EXPECT_FALSE(direction.active());
	EXPECT_FALSE(direction.throttleInhibited());
	direction.update(command + 1 + DShotDirection::StopTime);
	EXPECT_FALSE(direction.commandDue(command + 1 + DShotDirection::StopTime));
}

TEST(DShotDirection, DisarmRestoresWholeGroupAfterPartialReverse)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	const uint64_t command = Start + DShotDirection::StopTime;
	direction.update(command);
	direction.commandAttempted(command, 0x3);

	direction.observeArming(command + 1, false);
	ASSERT_TRUE(direction.cancel(command + 1));
	EXPECT_TRUE(direction.restoring());
	EXPECT_EQ(direction.requestId(), 1u);
	EXPECT_EQ(direction.mask(), Motors);
	EXPECT_EQ(direction.reversedMask(), Motors);
	EXPECT_FALSE(direction.cancel(command + 2));
	completeCommand(direction, command + 1);
	EXPECT_EQ(direction.reversedMask(), 0u);
}

TEST(DShotDirection, DuplicateAndExtraRequestsKeepOriginalTransaction)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	EXPECT_FALSE(direction.canRequest());
	EXPECT_FALSE(direction.requestReverse(Start + 1, Start + 1, Start, 1, Motors));
	EXPECT_FALSE(direction.requestReverse(Start + 1, Start + 1, Start, 2, 0x3));
	EXPECT_EQ(direction.requestId(), 1u);
	EXPECT_EQ(direction.mask(), Motors);
	completeCommand(direction, Start);
	EXPECT_FALSE(direction.requestReverse(Restore, Restore, Restore - 1, 2, Motors));

	direction.observeArming(Restore, false);
	ASSERT_TRUE(direction.cancel(Restore));
	direction.observeArming(Restore + 1, true);
	EXPECT_FALSE(direction.canRequest());
	EXPECT_FALSE(direction.requestReverse(Restore + 2, Restore + 2, Restore + 1, 2, Motors));
	EXPECT_EQ(direction.requestId(), 1u);
	EXPECT_EQ(direction.mask(), Motors);
	completeCommand(direction, Restore);
}

TEST(DShotDirection, RejectsRequestsAuthorizedBeforeLatestDisarm)
{
	DShotDirection direction;
	direction.observeArming(10, false);
	direction.observeArming(20, true);
	direction.observeArming(30, false);
	EXPECT_FALSE(direction.requestReverse(35, 35, 34, 1, Motors));
	direction.observeArming(40, true);
	EXPECT_FALSE(direction.requestReverse(45, 45, 20, 1, Motors));
	EXPECT_FALSE(direction.requestReverse(45, 45, 30, 1, Motors));
	EXPECT_TRUE(direction.requestReverse(45, 45, 41, 1, Motors));
	EXPECT_EQ(direction.timestampDisarmed(), 30u);
}

TEST(DShotDirection, RejectsInvalidOrExpiredRequests)
{
	DShotDirection direction;
	arm(direction);
	EXPECT_FALSE(direction.requestReverse(Start, 0, Start - 1, 1, Motors));
	EXPECT_FALSE(direction.requestReverse(Start, Start, 0, 1, Motors));
	EXPECT_FALSE(direction.requestReverse(Start, Start - 1, Start, 1, Motors));
	EXPECT_FALSE(direction.requestReverse(Start, Start + 1, Start, 1, Motors));
	EXPECT_FALSE(direction.requestReverse(Start, Start, Start - 1, 0, Motors));
	EXPECT_FALSE(direction.requestReverse(Start, Start, Start - 1, 1, 0));

	constexpr uint64_t Now = Start + 500000;
	EXPECT_FALSE(direction.requestReverse(Now, Now, Start, 1, Motors));
	EXPECT_TRUE(direction.requestReverse(Now, Now, Start + 1, 1, Motors));
}

TEST(DShotDirection, RequestCanPredateObservedArming)
{
	DShotDirection direction;
	direction.observeArming(10, false);
	EXPECT_FALSE(direction.canRequest());
	direction.observeArming(30, true);
	direction.observeArming(40, true);
	ASSERT_TRUE(direction.canRequest());
	EXPECT_TRUE(direction.requestReverse(45, 25, 21, 1, Motors));
	EXPECT_EQ(direction.timestampDisarmed(), 10u);
}

TEST(DShotDirection, KillClearKeepsCancelledReverseStoppedUntilDisarm)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	ASSERT_TRUE(direction.cancel(Start + 1));
	EXPECT_FALSE(direction.active());
	direction.observeArming(Start + 2, true);
	EXPECT_TRUE(direction.throttleInhibited());
	EXPECT_FALSE(direction.requestReverse(Start + 2, Start + 2, Start + 2, 2, Motors));

	direction.observeArming(0, false);
	EXPECT_TRUE(direction.throttleInhibited());
	arm(direction, Restore);
	EXPECT_FALSE(direction.throttleInhibited());
	EXPECT_TRUE(direction.requestReverse(Restore, Restore, Restore - 1, 2, Motors));
}

TEST(DShotDirection, KillClearKeepsRestoredSessionStoppedUntilDisarm)
{
	DShotDirection direction;
	ASSERT_TRUE(beginReversal(direction));
	completeCommand(direction, Start);

	ASSERT_TRUE(direction.cancel(Restore));
	EXPECT_TRUE(direction.throttleInhibited());
	direction.observeArming(Restore + 1, true);
	completeCommand(direction, Restore);
	EXPECT_EQ(direction.reversedMask(), 0u);
	EXPECT_TRUE(direction.throttleInhibited());
	constexpr uint64_t Rearm = Restore + 2000000;
	EXPECT_FALSE(direction.requestReverse(Rearm, Rearm, Rearm - 1, 2, Motors));
	arm(direction, Rearm);
	EXPECT_FALSE(direction.throttleInhibited());
	EXPECT_TRUE(direction.requestReverse(Rearm, Rearm, Rearm - 1, 2, Motors));
}

TEST(DShotDirection, KillClearDoesNotLatchStoppedOutputInNormalFlight)
{
	DShotDirection direction;
	arm(direction);
	EXPECT_FALSE(direction.cancel(Start));
	direction.observeArming(Start + 1, true);
	EXPECT_FALSE(direction.throttleInhibited());
	EXPECT_FALSE(direction.requestReverse(Start + 1, Start + 1, Start, 1, Motors));
	arm(direction, Restore);
	EXPECT_TRUE(direction.requestReverse(Restore, Restore, Restore - 1, 1, Motors));
}
