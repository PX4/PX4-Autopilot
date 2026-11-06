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
 * Test matrix rows F1-F9 (see TEST_MATRIX.md)
 */

#include <gtest/gtest.h>

#include "CanForwardFilter.hpp"
#include "CanForwardTestHelpers.hpp"

using can_forward::CanForwardFilter;
using namespace can_forward_test;

using Result = CanForwardFilter::Result;

static constexpr uint8_t Replace = 0;
static constexpr uint8_t Add = 1;
static constexpr uint8_t Remove = 2;

static void expectContents(const CanForwardFilter &filter, std::initializer_list<uint16_t> expected)
{
	ASSERT_EQ(filter.size(), expected.size());
	uint16_t i = 0;

	for (uint16_t id : expected) {
		EXPECT_EQ(filter.at(i++), id);
	}
}

// F7: type ID extraction
TEST(CanForwardFilter, TypeIdOfMessage)
{
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(messageId(NodeStatus, 42)), NodeStatus);
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(messageId(1063, 125)), 1063);
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(messageId(0xFFFF, 1)), 0xFFFF);
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(messageId(Allocation, 1)), Allocation);
}

TEST(CanForwardFilter, TypeIdOfServiceRequestAndResponse)
{
	for (uint8_t type : {GetNodeInfo, BeginFirmwareUpdate, FileGetInfo, FileRead, ParamGetSet}) {
		EXPECT_EQ(CanForwardFilter::dronecanTypeId(serviceId(type, true, 42, 127)), type);
		EXPECT_EQ(CanForwardFilter::dronecanTypeId(serviceId(type, false, 127, 42)), type);
	}

	EXPECT_EQ(CanForwardFilter::dronecanTypeId(serviceId(255, true, 1, 2)), 255);
}

TEST(CanForwardFilter, TypeIdOfAnonymousIsZero)
{
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(anonymousId(0x1234, Allocation)), 0);
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(anonymousId(0, 0)), 0);
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(anonymousId(0x3FFF, 3, 31)), 0);
}

TEST(CanForwardFilter, TypeIdIgnoresFlagBits)
{
	EXPECT_EQ(CanForwardFilter::dronecanTypeId(messageId(NodeStatus, 42) & ~Eff), NodeStatus);
}

// F1
TEST(CanForwardFilter, EmptyMatchesEverything)
{
	CanForwardFilter filter;
	EXPECT_TRUE(filter.empty());
	EXPECT_TRUE(filter.matches(messageId(NodeStatus, 42)));
	EXPECT_TRUE(filter.matches(serviceId(FileRead, true, 127, 42)));
	EXPECT_TRUE(filter.matches(anonymousId(1, 1)));
	EXPECT_TRUE(filter.matches(0x123)); // standard frame
}

// F2
TEST(CanForwardFilter, ReplaceWithNothingClears)
{
	CanForwardFilter filter;
	const uint16_t ids[] = {NodeStatus};
	ASSERT_EQ(filter.apply(Replace, ids, 1), Result::Ok);
	EXPECT_FALSE(filter.matches(serviceId(FileRead, true, 127, 42)));

	EXPECT_EQ(filter.apply(Replace, nullptr, 0), Result::Ok);
	EXPECT_TRUE(filter.empty());
	EXPECT_TRUE(filter.matches(serviceId(FileRead, true, 127, 42)));
}

// F3
TEST(CanForwardFilter, ReplaceSortsAndRemovesDuplicates)
{
	CanForwardFilter filter;
	const uint16_t ids[] = {341, 48, 1, 48, 0, 341, 40};
	ASSERT_EQ(filter.apply(Replace, ids, 7), Result::Ok);
	expectContents(filter, {0, 1, 40, 48, 341});

	const uint16_t other[] = {7};
	ASSERT_EQ(filter.apply(Replace, other, 1), Result::Ok);
	expectContents(filter, {7});
}

TEST(CanForwardFilter, AddMerges)
{
	CanForwardFilter filter;
	const uint16_t a[] = {40, 10};
	const uint16_t b[] = {5, 40, 50, 10, 20};
	ASSERT_EQ(filter.apply(Add, a, 2), Result::Ok);
	ASSERT_EQ(filter.apply(Add, b, 5), Result::Ok);
	expectContents(filter, {5, 10, 20, 40, 50});

	// adding only existing IDs changes nothing
	ASSERT_EQ(filter.apply(Add, a, 2), Result::Ok);
	expectContents(filter, {5, 10, 20, 40, 50});

	// add nothing
	ASSERT_EQ(filter.apply(Add, nullptr, 0), Result::Ok);
	expectContents(filter, {5, 10, 20, 40, 50});
}

TEST(CanForwardFilter, AddToEmptyAndBoundaries)
{
	CanForwardFilter filter;
	const uint16_t ids[] = {0xFFFF, 0};
	ASSERT_EQ(filter.apply(Add, ids, 2), Result::Ok);
	expectContents(filter, {0, 0xFFFF});
	EXPECT_TRUE(filter.matches(anonymousId(3, 1)));
	EXPECT_TRUE(filter.matches(messageId(0xFFFF, 3)));
	EXPECT_FALSE(filter.matches(messageId(0xFFFE, 3)));
}

// F4
TEST(CanForwardFilter, Remove)
{
	CanForwardFilter filter;
	const uint16_t ids[] = {1, 2, 3, 4};
	ASSERT_EQ(filter.apply(Replace, ids, 4), Result::Ok);

	const uint16_t remove[] = {4, 2, 99};
	ASSERT_EQ(filter.apply(Remove, remove, 3), Result::Ok);
	expectContents(filter, {1, 3});

	// removing an absent ID is a no-op
	const uint16_t absent[] = {77};
	ASSERT_EQ(filter.apply(Remove, absent, 1), Result::Ok);
	expectContents(filter, {1, 3});

	// removing down to empty means forward everything again
	const uint16_t rest[] = {1, 3};
	ASSERT_EQ(filter.apply(Remove, rest, 2), Result::Ok);
	EXPECT_TRUE(filter.empty());
	EXPECT_TRUE(filter.matches(messageId(NodeStatus, 9)));
}

// F5
TEST(CanForwardFilter, RejectsTooManyIdsAndUnknownOp)
{
	CanForwardFilter filter;
	const uint16_t initial[] = {5};
	ASSERT_EQ(filter.apply(Replace, initial, 1), Result::Ok);

	uint16_t ids[17] {};

	for (uint16_t i = 0; i < 17; i++) { ids[i] = i + 100; }

	EXPECT_EQ(filter.apply(Replace, ids, 17), Result::TooManyIds);
	EXPECT_EQ(filter.apply(Add, ids, 255), Result::TooManyIds);
	EXPECT_EQ(filter.apply(3, ids, 1), Result::UnknownOp);
	EXPECT_EQ(filter.apply(255, ids, 1), Result::UnknownOp);
	expectContents(filter, {5});

	// exactly 16 is fine
	EXPECT_EQ(filter.apply(Replace, ids, 16), Result::Ok);
	EXPECT_EQ(filter.size(), 16);
}

// F6
TEST(CanForwardFilter, AddBeyondCapacityIsAtomic)
{
	CanForwardFilter filter;
	uint16_t next = 0;

	for (int round = 0; round < 4; round++) {
		uint16_t ids[16];

		for (auto &id : ids) { id = next++; }

		ASSERT_EQ(filter.apply(Add, ids, 16), Result::Ok);
	}

	ASSERT_EQ(filter.size(), CanForwardFilter::Capacity);

	// one new ID plus one existing: overflows, nothing changes
	const uint16_t more[] = {1000, 3};
	EXPECT_EQ(filter.apply(Add, more, 2), Result::Overflow);
	EXPECT_EQ(filter.size(), CanForwardFilter::Capacity);
	EXPECT_FALSE(filter.matches(messageId(1000, 1)));

	// only existing IDs: still fine when full
	const uint16_t existing[] = {3, 4};
	EXPECT_EQ(filter.apply(Add, existing, 2), Result::Ok);

	// replace always fits
	EXPECT_EQ(filter.apply(Replace, more, 2), Result::Ok);
	expectContents(filter, {3, 1000});
}

// F7 / F9: the minimal set needed for a firmware update
TEST(CanForwardFilter, FirmwareUpdateSet)
{
	CanForwardFilter filter;
	const uint16_t ids[] = {0, GetNodeInfo, BeginFirmwareUpdate, FileGetInfo, FileRead, NodeStatus};
	ASSERT_EQ(filter.apply(Replace, ids, 6), Result::Ok);

	EXPECT_TRUE(filter.matches(messageId(NodeStatus, 42)));
	EXPECT_TRUE(filter.matches(serviceId(GetNodeInfo, false, 127, 42)));
	EXPECT_TRUE(filter.matches(serviceId(BeginFirmwareUpdate, false, 127, 42)));
	EXPECT_TRUE(filter.matches(serviceId(FileRead, true, 127, 42)));
	EXPECT_TRUE(filter.matches(serviceId(FileGetInfo, true, 127, 42)));
	EXPECT_TRUE(filter.matches(anonymousId(0x55, 1))); // bootloader without a node ID yet
	EXPECT_TRUE(filter.matches(messageId(Allocation, 10))); // allocation server response (type 1)

	EXPECT_FALSE(filter.matches(messageId(1063, 42)));  // e.g. GNSS fix
	EXPECT_FALSE(filter.matches(serviceId(ParamGetSet, true, 42, 127)));
	EXPECT_FALSE(filter.matches(0x341)); // standard frame when filtering
}
