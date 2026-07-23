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

#include "FailureTable.hpp"

using namespace failure_injection;
using AckResult = FailureTable::AckResult;

namespace
{

constexpr uint8_t GYRO  = failure_injection_s::FAILURE_UNIT_SENSOR_GYRO;
constexpr uint8_t GPS   = failure_injection_s::FAILURE_UNIT_SENSOR_GPS;
constexpr uint8_t MOTOR = failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR;

constexpr uint8_t OK      = failure_injection_s::FAILURE_TYPE_OK;
constexpr uint8_t OFF     = failure_injection_s::FAILURE_TYPE_OFF;
constexpr uint8_t STUCK   = failure_injection_s::FAILURE_TYPE_STUCK;
constexpr uint8_t WRONG   = failure_injection_s::FAILURE_TYPE_WRONG;
constexpr uint8_t GARBAGE = failure_injection_s::FAILURE_TYPE_GARBAGE;

} // namespace

TEST(FailureTable, SupportedCatalogueMatchesInventory)
{
	EXPECT_TRUE(FailureTable::isSupported(GYRO, OFF));
	EXPECT_TRUE(FailureTable::isSupported(GYRO, STUCK));
	EXPECT_FALSE(FailureTable::isSupported(GYRO, WRONG));   // no gyro WRONG today
	EXPECT_TRUE(FailureTable::isSupported(GPS, WRONG));
	EXPECT_TRUE(FailureTable::isSupported(MOTOR, WRONG));
	EXPECT_FALSE(FailureTable::isSupported(GYRO, GARBAGE)); // GARBAGE unimplemented
	// Distance sensor (rangefinder) supports OFF/STUCK on hardware, but not WRONG.
	EXPECT_TRUE(FailureTable::isSupported(failure_injection_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR, OFF));
	EXPECT_TRUE(FailureTable::isSupported(failure_injection_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR, STUCK));
	EXPECT_FALSE(FailureTable::isSupported(failure_injection_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR, WRONG));
	// Unimplemented units.
	EXPECT_FALSE(FailureTable::isSupported(failure_injection_s::FAILURE_UNIT_SYSTEM_RC_SIGNAL, OFF));
}

TEST(FailureTable, UnsupportedIsRejectedWithoutChange)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, WRONG, 0), AckResult::Unsupported);
	EXPECT_FALSE(table.changed());
	EXPECT_EQ(table.count(), 0);
}

TEST(FailureTable, InjectSpecificInstanceSetsBit)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, OFF, 2), AckResult::Accepted); // 1-based instance 2
	EXPECT_TRUE(table.changed());
	EXPECT_EQ(table.count(), 1);

	failure_injection_s msg{};
	table.fill(msg);
	EXPECT_EQ(msg.count, 1);
	EXPECT_EQ(msg.unit[0], GYRO);
	EXPECT_EQ(msg.failure_type[0], OFF);
	EXPECT_EQ(msg.instance_mask[0], 0x2); // bit (2 - 1)
}

TEST(FailureTable, InjectAllInstancesUsesFullMask)
{
	FailureTable table;
	EXPECT_EQ(table.inject(MOTOR, OFF, 0), AckResult::Accepted); // 0 = all
	failure_injection_s msg{};
	table.fill(msg);
	EXPECT_EQ(msg.instance_mask[0], 0xFFFF);
}

TEST(FailureTable, DuplicateInjectDoesNotChange)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, OFF, 1), AckResult::Accepted);
	EXPECT_TRUE(table.changed());
	table.clearChanged();

	// Re-inject the exact same failure: accepted, but the table did not change.
	EXPECT_EQ(table.inject(GYRO, OFF, 1), AckResult::Accepted);
	EXPECT_FALSE(table.changed());
}

TEST(FailureTable, OkClearsTheInstance)
{
	FailureTable table;
	table.inject(GYRO, OFF, 0); // all gyros off
	table.clearChanged();

	EXPECT_EQ(table.inject(GYRO, OK, 0), AckResult::Accepted);
	EXPECT_TRUE(table.changed());
	EXPECT_EQ(table.count(), 0);
}

TEST(FailureTable, OkClearsOnlyTheAddressedInstance)
{
	FailureTable table;
	table.inject(GYRO, OFF, 0); // mask 0xFFFF

	// Clear only instance 2.
	EXPECT_EQ(table.inject(GYRO, OK, 2), AckResult::Accepted);
	failure_injection_s msg{};
	table.fill(msg);
	ASSERT_EQ(msg.count, 1);
	EXPECT_EQ(msg.instance_mask[0], 0xFFFF & ~0x2);
}

TEST(FailureTable, NewModeOnInstanceClearsOldMode)
{
	FailureTable table;
	table.inject(GYRO, OFF, 1);   // instance 1 OFF
	table.inject(GYRO, STUCK, 1); // instance 1 now STUCK -> OFF entry must drop

	failure_injection_s msg{};
	table.fill(msg);
	ASSERT_EQ(msg.count, 1);
	EXPECT_EQ(msg.unit[0], GYRO);
	EXPECT_EQ(msg.failure_type[0], STUCK);
	EXPECT_EQ(msg.instance_mask[0], 0x1);
}

TEST(FailureTable, MotorMaskAccumulatesInstances)
{
	FailureTable table;
	table.inject(MOTOR, OFF, 1);
	table.inject(MOTOR, OFF, 3);
	table.inject(MOTOR, OFF, 5);

	failure_injection_s msg{};
	table.fill(msg);
	ASSERT_EQ(msg.count, 1); // single entry, three instances
	EXPECT_EQ(msg.instance_mask[0], 0x1 | 0x4 | 0x10);
}

TEST(FailureTable, OverflowBeyondFourDistinctFailuresIsRejected)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_BARO, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.count(), 4);

	// A fifth distinct (unit, type) must be rejected and leave the table intact.
	table.clearChanged();
	EXPECT_EQ(table.inject(GPS, OFF, 1), AckResult::Rejected);
	EXPECT_FALSE(table.changed());
	EXPECT_EQ(table.count(), 4);
}

TEST(FailureTable, AddingInstanceToExistingEntryDoesNotCountAsNew)
{
	FailureTable table;
	table.inject(GYRO, OFF, 1);
	table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL, OFF, 1);
	table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, OFF, 1);
	table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_BARO, OFF, 1);
	ASSERT_EQ(table.count(), 4);

	// Table is full, but extending an existing entry to another instance must succeed.
	EXPECT_EQ(table.inject(GYRO, OFF, 2), AckResult::Accepted);
	EXPECT_EQ(table.count(), 4);

	failure_injection_s msg{};
	table.fill(msg);
	// gyro entry now covers instances 1 and 2
	EXPECT_EQ(msg.instance_mask[0], 0x1 | 0x2);
}

TEST(FailureTable, InjectMaskSetsMultipleInstances)
{
	FailureTable table;
	// bits 0 and 2 -> instances 1 and 3 (MAV_CMD_INJECT_FAILURE param4)
	EXPECT_EQ(table.injectMask(GYRO, OFF, 0x5), AckResult::Accepted);
	EXPECT_TRUE(table.changed());
	ASSERT_EQ(table.count(), 1);

	failure_injection_s msg{};
	table.fill(msg);
	EXPECT_EQ(msg.unit[0], GYRO);
	EXPECT_EQ(msg.failure_type[0], OFF);
	EXPECT_EQ(msg.instance_mask[0], 0x5);
}

TEST(FailureTable, InjectMaskZeroIsAcceptedNoOp)
{
	FailureTable table;
	EXPECT_EQ(table.injectMask(GYRO, OFF, 0), AckResult::Accepted);
	EXPECT_FALSE(table.changed());
	EXPECT_EQ(table.count(), 0);
}

TEST(FailureTable, InjectMaskUnsupportedIsRejected)
{
	FailureTable table;
	EXPECT_EQ(table.injectMask(GYRO, WRONG, 0x1), AckResult::Unsupported);
	EXPECT_EQ(table.count(), 0);
}
