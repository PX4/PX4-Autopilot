/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * Test matrix rows S1-S7, S11 and A1-A3 (see TEST_MATRIX.md)
 */

#include <gtest/gtest.h>

#include <math.h>

#include "CanForwardSession.hpp"

using can_forward::CanForwardSession;
using Result = CanForwardSession::Result;
using State = CanForwardSession::State;

static constexpr uint64_t s = 1'000'000;
static constexpr uint8_t NumBuses = 2;
static constexpr uint32_t Gcs = 255;
static constexpr uint8_t GcsComp = 189; // MAV_COMP_ID_MAVCAN

class CanForwardSessionTest : public ::testing::Test
{
protected:
	Result start(float bus, uint64_t now, bool blocked = false, uint32_t sysid = Gcs, uint8_t compid = GcsComp)
	{
		return session.request(bus, NumBuses, blocked, now, sysid, compid);
	}

	CanForwardSession session;
};

// S1
TEST_F(CanForwardSessionTest, StartOnBus1)
{
	EXPECT_EQ(session.state(), State::Idle);
	EXPECT_FALSE(session.allowForward(0));

	EXPECT_EQ(start(1.f, 10 * s), Result::Accepted);
	EXPECT_EQ(session.state(), State::Active);
	EXPECT_EQ(session.bus(), 0);
	EXPECT_EQ(session.sysid(), Gcs);
	EXPECT_EQ(session.compid(), GcsComp);
	EXPECT_TRUE(session.allowForward(0));
	EXPECT_TRUE(session.allowInject(0));
	EXPECT_FALSE(session.allowForward(1));
	EXPECT_FALSE(session.allowInject(1));
}

// S2
TEST_F(CanForwardSessionTest, BusZeroStops)
{
	ASSERT_EQ(start(2.f, 10 * s), Result::Accepted);
	EXPECT_EQ(start(0.f, 11 * s), Result::Accepted);
	EXPECT_EQ(session.state(), State::Idle);
	EXPECT_FALSE(session.allowForward(1));

	// stopping while idle is fine as well
	EXPECT_EQ(start(0.f, 12 * s), Result::Accepted);

	// and while armed
	ASSERT_EQ(start(1.f, 13 * s), Result::Accepted);
	EXPECT_EQ(start(0.f, 14 * s, true), Result::Accepted);
	EXPECT_EQ(session.state(), State::Idle);
}

// S3
TEST_F(CanForwardSessionTest, InvalidBusIsDenied)
{
	for (float bus : {3.f, 255.f, 256.f, -1.f, 1.5f, 0.5f, NAN, INFINITY, -INFINITY, 1e9f}) {
		EXPECT_EQ(start(bus, 10 * s), Result::Denied) << bus;
		EXPECT_EQ(session.state(), State::Idle) << bus;
	}

	// an invalid request does not end a running session
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);
	EXPECT_EQ(start(7.f, 11 * s), Result::Denied);
	EXPECT_EQ(session.state(), State::Active);
	EXPECT_EQ(session.bus(), 0);
}

TEST_F(CanForwardSessionTest, NoBuses)
{
	EXPECT_EQ(session.request(1.f, 0, false, 10 * s, Gcs, GcsComp), Result::Denied);
}

TEST_F(CanForwardSessionTest, FloatRoundingTolerated)
{
	EXPECT_EQ(start(1.0001f, 10 * s), Result::Accepted);
	EXPECT_EQ(session.bus(), 0);
	EXPECT_EQ(start(1.9999f, 10 * s), Result::Accepted);
	EXPECT_EQ(session.bus(), 1);
}

// S4
TEST_F(CanForwardSessionTest, KeepaliveKeepsSessionForever)
{
	uint64_t now = 10 * s;
	ASSERT_EQ(start(1.f, now), Result::Accepted);

	for (int i = 0; i < 60; i++) {
		now += s;
		EXPECT_FALSE(session.update(false, now));
		EXPECT_EQ(start(1.f, now), Result::Accepted);
	}

	EXPECT_EQ(session.state(), State::Active);
}

// S5: timeout is driven by update(), not by traffic, so it also ends on a silent bus
TEST_F(CanForwardSessionTest, TimeoutWithoutKeepalive)
{
	const uint64_t t0 = 10 * s;
	ASSERT_EQ(start(1.f, t0), Result::Accepted);

	EXPECT_FALSE(session.update(false, t0 + CanForwardSession::KeepaliveTimeout));
	EXPECT_EQ(session.state(), State::Active);

	EXPECT_TRUE(session.update(false, t0 + CanForwardSession::KeepaliveTimeout + 1));
	EXPECT_EQ(session.state(), State::Idle);
	EXPECT_FALSE(session.allowForward(0));

	// stays idle
	EXPECT_FALSE(session.update(false, t0 + 100 * s));
}

// S6
TEST_F(CanForwardSessionTest, KeepaliveJustBeforeTimeout)
{
	const uint64_t t0 = 10 * s;
	ASSERT_EQ(start(1.f, t0), Result::Accepted);
	ASSERT_EQ(start(1.f, t0 + 4'990'000), Result::Accepted);
	EXPECT_FALSE(session.update(false, t0 + 9 * s));
	EXPECT_EQ(session.state(), State::Active);
	EXPECT_TRUE(session.update(false, t0 + 4'990'000 + CanForwardSession::KeepaliveTimeout + 1));
}

// S7
TEST_F(CanForwardSessionTest, SwitchBus)
{
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);
	ASSERT_EQ(start(2.f, 11 * s), Result::Accepted);
	EXPECT_EQ(session.bus(), 1);
	EXPECT_FALSE(session.allowForward(0));
	EXPECT_TRUE(session.allowForward(1));
}

// S8 (session part): a different client takes over
TEST_F(CanForwardSessionTest, Takeover)
{
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);
	ASSERT_EQ(start(1.f, 11 * s, false, 42, 1), Result::Accepted);
	EXPECT_EQ(session.sysid(), 42u);
	EXPECT_EQ(session.compid(), 1);
}

// S11
TEST_F(CanForwardSessionTest, ClockGoingBackwardsDoesNotTimeOut)
{
	ASSERT_EQ(start(1.f, 100 * s), Result::Accepted);
	EXPECT_FALSE(session.update(false, 1 * s));
	EXPECT_EQ(session.state(), State::Active);
	EXPECT_FALSE(session.update(false, 0));
	EXPECT_EQ(session.state(), State::Active);
}

// A1
TEST_F(CanForwardSessionTest, NoNewSessionWhileBlocked)
{
	EXPECT_EQ(start(1.f, 10 * s, true), Result::TemporarilyRejected);
	EXPECT_EQ(session.state(), State::Idle);
}

TEST_F(CanForwardSessionTest, NoBusChangeOrTakeoverWhileBlocked)
{
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);

	EXPECT_EQ(start(2.f, 11 * s, true), Result::TemporarilyRejected);
	EXPECT_EQ(start(1.f, 11 * s, true, 42, 1), Result::TemporarilyRejected);
	EXPECT_EQ(session.bus(), 0);
	EXPECT_EQ(session.sysid(), Gcs);
}

// A2
TEST_F(CanForwardSessionTest, ArmingSwitchesToMonitorOnly)
{
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);

	EXPECT_TRUE(session.update(true, 11 * s));
	EXPECT_EQ(session.state(), State::MonitorOnly);
	EXPECT_TRUE(session.allowForward(0));
	EXPECT_FALSE(session.allowInject(0));

	// keepalive from the owner while blocked keeps the session in MonitorOnly
	EXPECT_EQ(start(1.f, 12 * s, true), Result::Accepted);
	EXPECT_EQ(session.state(), State::MonitorOnly);
	EXPECT_FALSE(session.allowInject(0));

	// MonitorOnly still times out
	EXPECT_TRUE(session.update(true, 12 * s + CanForwardSession::KeepaliveTimeout + 1));
	EXPECT_EQ(session.state(), State::Idle);
}

TEST_F(CanForwardSessionTest, KeepaliveWhileBlockedBeforeUpdate)
{
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);
	EXPECT_EQ(start(1.f, 11 * s, true), Result::Accepted);
	EXPECT_EQ(session.state(), State::MonitorOnly);
}

// A3
TEST_F(CanForwardSessionTest, DisarmResumesInjection)
{
	ASSERT_EQ(start(1.f, 10 * s), Result::Accepted);
	ASSERT_TRUE(session.update(true, 11 * s));
	EXPECT_TRUE(session.update(false, 12 * s));
	EXPECT_EQ(session.state(), State::Active);
	EXPECT_TRUE(session.allowInject(0));
	EXPECT_FALSE(session.update(false, 13 * s));
}
