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

#include "GnssReceiverOrder.hpp"

using sensors::gnss_order::receiverOrder;

namespace
{
struct Receivers {
	uint8_t first_publication[2] {};
	int8_t slot[2] {-1, -1};
	bool slot_configured[2] {};
	int preferred{-1};
	bool preference_configured{false};

	void order(int8_t (&order)[2]) const
	{
		receiverOrder(first_publication, slot, slot_configured, preferred, preference_configured, order);
	}
};
}

TEST(GnssReceiverOrder, WithoutConfigurationFollowsFirstPublication)
{
	Receivers receivers{};
	int8_t order[2];

	receivers.order(order);
	EXPECT_EQ(order[0], -1);
	EXPECT_EQ(order[1], -1);

	receivers.first_publication[1] = 1;
	receivers.order(order);
	EXPECT_EQ(order[0], -1);
	EXPECT_EQ(order[1], 0);

	receivers.first_publication[0] = 2;
	receivers.order(order);
	EXPECT_EQ(order[0], 1);
	EXPECT_EQ(order[1], 0);
}

TEST(GnssReceiverOrder, PreferredFirstAndHeldUntilItPublishes)
{
	Receivers receivers{};
	receivers.preference_configured = true;
	receivers.slot[0] = 1; // slots don't apply with a preference
	receivers.slot_configured[1] = true;
	int8_t order[2];

	receivers.first_publication[0] = 1;
	receivers.order(order);
	EXPECT_EQ(order[0], 1);
	EXPECT_EQ(order[1], -1);

	receivers.first_publication[1] = 2;
	receivers.preferred = 1;
	receivers.order(order);
	EXPECT_EQ(order[0], 1);
	EXPECT_EQ(order[1], 0);
}

TEST(GnssReceiverOrder, SlotsGiveTheOrderRegardlessOfPublication)
{
	Receivers receivers{};
	receivers.slot_configured[0] = true;
	receivers.slot_configured[1] = true;
	receivers.slot[0] = 1;
	receivers.slot[1] = 0;
	int8_t order[2];

	// the receiver in slot 0 publishes second, its position stays free until then
	receivers.first_publication[0] = 1;
	receivers.order(order);
	EXPECT_EQ(order[0], 1);
	EXPECT_EQ(order[1], -1);

	receivers.first_publication[1] = 2;
	receivers.order(order);
	EXPECT_EQ(order[0], 1);
	EXPECT_EQ(order[1], 0);
}

TEST(GnssReceiverOrder, UnmatchedReceiverTakesTheFreePosition)
{
	Receivers receivers{};
	receivers.slot_configured[1] = true;
	receivers.slot[1] = 1;
	int8_t order[2];

	receivers.first_publication[1] = 1;
	receivers.first_publication[0] = 2;
	receivers.order(order);
	EXPECT_EQ(order[0], 0);
	EXPECT_EQ(order[1], 1);
}

TEST(GnssReceiverOrder, UnmatchedReceiverIsReportedWhenAllPositionsAreHeld)
{
	// both slots configured, but receiver 0 matches neither: it takes the position held for the slot that doesn't publish
	Receivers receivers{};
	receivers.slot_configured[0] = true;
	receivers.slot_configured[1] = true;
	receivers.slot[1] = 1;
	int8_t order[2];

	receivers.first_publication[0] = 1;
	receivers.order(order);
	EXPECT_EQ(order[0], 0);

	receivers.first_publication[1] = 2;
	receivers.order(order);
	EXPECT_EQ(order[0], 0);
	EXPECT_EQ(order[1], 1);
}
