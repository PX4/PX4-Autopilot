/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file hysteresis_test.cpp
 * Tests for system timing hysteresis.
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include "hysteresis.h"


#if defined(__PX4_DARWIN) ||  defined(__PX4_CYGWIN)
static const int f = 10;
#else
static const int f = 1;
#endif


TEST(Hysteresis, InitFalse)
{
	systemlib::Hysteresis hysteresis(false);
	EXPECT_FALSE(hysteresis.get_state());
}

TEST(Hysteresis, InitTrue)
{
	systemlib::Hysteresis hysteresis(true);
	EXPECT_TRUE(hysteresis.get_state());
}

TEST(Hysteresis, Zero)
{
	// Default is 0 hysteresis.
	systemlib::Hysteresis hysteresis(false);
	EXPECT_FALSE(hysteresis.get_state());

	// Change and see result immediately.
	hysteresis.set_state_and_update(true);
	EXPECT_TRUE(hysteresis.get_state());
	hysteresis.set_state_and_update(false);
	EXPECT_FALSE(hysteresis.get_state());
	hysteresis.set_state_and_update(true);
	EXPECT_TRUE(hysteresis.get_state());

	// A wait won't change anything.
	px4_usleep(1000 * f);
	hysteresis.update();
	EXPECT_TRUE(hysteresis.get_state());
}

TEST(Hysteresis, ChangeAfterTime)
{

	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);
	hysteresis.set_hysteresis_time_from(true, 3000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(4000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(2000 * f);
	hysteresis.update();
	EXPECT_TRUE(hysteresis.get_state());

	// Change back to false.
	hysteresis.set_state_and_update(false);
	EXPECT_TRUE(hysteresis.get_state());
	px4_usleep(1000 * f);
	hysteresis.update();
	EXPECT_TRUE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());
}

TEST(Hysteresis, HysteresisChanged)
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(true, 2000 * f);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	EXPECT_TRUE(hysteresis.get_state());

	// Change hysteresis time.
	hysteresis.set_hysteresis_time_from(true, 10000 * f);

	// Change back to false.
	hysteresis.set_state_and_update(false);
	EXPECT_TRUE(hysteresis.get_state());
	px4_usleep(7000 * f);
	hysteresis.update();
	EXPECT_TRUE(hysteresis.get_state());
	px4_usleep(5000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());
}

TEST(Hysteresis, ChangeAfterMultipleSets)
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(true, 5000 * f);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(true);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(true);
	EXPECT_TRUE(hysteresis.get_state());

	// Change to false.
	hysteresis.set_state_and_update(false);
	EXPECT_TRUE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(false);
	EXPECT_TRUE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(false);
	EXPECT_FALSE(hysteresis.get_state());
}

TEST(Hysteresis, TakeChangeBack)
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());
	// Change your mind to false.
	hysteresis.set_state_and_update(false);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(6000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());

	// And true again
	hysteresis.set_state_and_update(true);
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	EXPECT_FALSE(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	EXPECT_TRUE(hysteresis.get_state());

	// The other directory is immediate.
	hysteresis.set_state_and_update(false);
	EXPECT_FALSE(hysteresis.get_state());
}
