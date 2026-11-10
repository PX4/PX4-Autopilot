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

#include <gtest/gtest.h>

#include "CanForward.hpp"

using namespace can_forward;

// DroneCAN CAN IDs: priority 24..28, message type 8..23 or service type 16..23, service bit 7, source node 0..6
static constexpr uint32_t messageId(uint16_t type, uint8_t src) { return CanFlagEff | (16U << 24) | (uint32_t(type) << 8) | src; }
static constexpr uint32_t serviceId(uint8_t type, uint8_t dst, uint8_t src) { return CanFlagEff | (uint32_t(type) << 16) | 0x80U | (uint32_t(dst) << 8) | src; }

TEST(CanForwardTest, ValidFrame)
{
	EXPECT_TRUE(isValidFrame(messageId(341, 10), 8));
	EXPECT_TRUE(isValidFrame(0x7FF, 0));
	EXPECT_FALSE(isValidFrame(messageId(341, 10), 9)); // CAN FD length
	EXPECT_FALSE(isValidFrame(0x800, 1));              // standard ID above 11 bits
	EXPECT_FALSE(isValidFrame(messageId(341, 10) | CanFlagRtr, 0));
	EXPECT_FALSE(isValidFrame(messageId(341, 10) | CanFlagErr, 0));
}

TEST(CanForwardTest, TypeId)
{
	EXPECT_EQ(dronecanTypeId(messageId(341, 10)), 341);
	EXPECT_EQ(dronecanTypeId(serviceId(48, 10, 127)), 48);
	EXPECT_EQ(dronecanTypeId(messageId(1, 0)), 0); // anonymous
}

TEST(CanForwardTest, EmptyFilterForwardsAll)
{
	Filter filter;
	EXPECT_TRUE(filter.matches(messageId(341, 10)));
	EXPECT_TRUE(filter.matches(0x123));
}

TEST(CanForwardTest, FilterOperations)
{
	Filter filter;
	const uint16_t ids[] = {341, 48, 341};

	EXPECT_TRUE(filter.apply(Filter::Replace, ids, 3));
	EXPECT_EQ(filter.size(), 2);
	EXPECT_TRUE(filter.matches(messageId(341, 10)));
	EXPECT_TRUE(filter.matches(serviceId(48, 10, 127)));
	EXPECT_FALSE(filter.matches(messageId(1030, 10)));
	EXPECT_FALSE(filter.matches(0x123)); // standard frame

	const uint16_t add[] = {1030};
	EXPECT_TRUE(filter.apply(Filter::Add, add, 1));
	EXPECT_TRUE(filter.matches(messageId(1030, 10)));

	const uint16_t remove[] = {341, 1030};
	EXPECT_TRUE(filter.apply(Filter::Remove, remove, 2));
	EXPECT_EQ(filter.size(), 1);
	EXPECT_FALSE(filter.matches(messageId(341, 10)));

	EXPECT_TRUE(filter.apply(Filter::Replace, nullptr, 0));
	EXPECT_TRUE(filter.matches(messageId(341, 10)));

	EXPECT_FALSE(filter.apply(3, add, 1));
}

TEST(CanForwardTest, FilterFull)
{
	Filter filter;
	uint16_t ids[Filter::Capacity + 1];

	for (uint16_t i = 0; i < Filter::Capacity + 1; i++) {
		ids[i] = i + 1;
	}

	EXPECT_FALSE(filter.apply(Filter::Add, ids, Filter::Capacity + 1));
	EXPECT_EQ(filter.size(), Filter::Capacity);
}
