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

#include "TurtleSession.hpp"
#include <gtest/gtest.h>

TEST(TurtleSession, CenterBeforeDrive)
{
	TurtleSession session;
	EXPECT_FALSE(session.update(100, false, 1000));
	EXPECT_FALSE(session.update(200, true, 1000));
	EXPECT_TRUE(session.update(300, false, 1000));
	EXPECT_TRUE(session.update(1299, false, 1000));
	EXPECT_FALSE(session.update(1300, false, 1000));
	EXPECT_FALSE(session.update(1400, false, 1000));
	EXPECT_FALSE(session.update(1500, true, 1000));
	EXPECT_TRUE(session.update(1600, false, 1000));
}

TEST(TurtleSession, CenterStopsAndStartsNewAttempt)
{
	TurtleSession session;
	EXPECT_FALSE(session.update(100, true, 1000));
	EXPECT_TRUE(session.update(200, false, 1000));
	EXPECT_FALSE(session.update(300, true, 1000));
	EXPECT_TRUE(session.update(400, false, 1000));
	EXPECT_TRUE(session.update(1399, false, 1000));
	EXPECT_FALSE(session.update(1400, false, 1000));
}

TEST(TurtleSession, ResetRequiresCenterBeforeDrivingAgain)
{
	TurtleSession session;
	EXPECT_FALSE(session.update(100, true, 1000));
	EXPECT_TRUE(session.update(200, false, 1000));
	session.reset();
	EXPECT_FALSE(session.update(300, false, 1000));
	EXPECT_FALSE(session.update(400, true, 1000));
	EXPECT_TRUE(session.update(500, false, 1000));
}

TEST(TurtleSession, ClockRegressionStopsDrive)
{
	TurtleSession session;
	EXPECT_FALSE(session.update(100, true, 1000));
	EXPECT_TRUE(session.update(200, false, 1000));
	EXPECT_FALSE(session.update(199, false, 1000));
	EXPECT_FALSE(session.update(300, false, 1000));
}
