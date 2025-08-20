/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ManualControlSelector.hpp"
#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

static constexpr uint64_t SOME_TIME = 12345678;
static constexpr uint8_t SOURCE_RC = manual_control_setpoint_s::SOURCE_RC;
static constexpr uint8_t SOURCE_MAVLINK_0 = manual_control_setpoint_s::SOURCE_MAVLINK_0;
static constexpr uint8_t SOURCE_MAVLINK_1 = manual_control_setpoint_s::SOURCE_MAVLINK_1;
static constexpr uint8_t SOURCE_MAVLINK_2 = manual_control_setpoint_s::SOURCE_MAVLINK_2;
static constexpr uint8_t SOURCE_MAVLINK_3 = manual_control_setpoint_s::SOURCE_MAVLINK_3;
static constexpr uint8_t SOURCE_MAVLINK_4 = manual_control_setpoint_s::SOURCE_MAVLINK_4;


TEST(ManualControlSelector, RcInputInvalidValid)
{
	ManualControlSelector selector;
	selector.setRcInMode(0); // RC Transmitter only
	selector.setTimeout(500_ms);

	constexpr uint8_t kInstanceIndex = 1;
	constexpr uint8_t kExpectedDataSource = SOURCE_RC;

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {}; // inputs[0] remains unused

	// Configure inputs[1] with SOURCE_RC, initially invalid
	inputs[kInstanceIndex].data_source = kExpectedDataSource;
	inputs[kInstanceIndex].valid = false;
	inputs[kInstanceIndex].timestamp_sample = timestamp;

	// Run two iterations with invalid input
	for (int i = 0; i < 2; i++) {
		selector.updateWithNewInputSamples(timestamp, inputs, 2);

		EXPECT_FALSE(selector.setpoint().valid);
		EXPECT_EQ(selector.setpoint().timestamp_sample, 0);
		EXPECT_EQ(selector.instance(), -1);
		EXPECT_EQ(selector.setpoint().data_source, 0);

		// Advance time for next sample
		timestamp += 100_ms;
		inputs[kInstanceIndex].timestamp_sample = timestamp;
	}

	// Now make the input valid
	inputs[kInstanceIndex].valid = true;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().timestamp_sample, timestamp);
	EXPECT_EQ(selector.instance(), kInstanceIndex);
	EXPECT_EQ(selector.setpoint().data_source, kExpectedDataSource);
}


TEST(ManualControlSelector, RcInputContinuous)
{
	ManualControlSelector selector;
	selector.setRcInMode(0); // RC only
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {}; // Only using index 1

	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	for (int i = 0; i < 5; i++) {
		selector.updateWithNewInputSamples(timestamp, inputs, 2);

		EXPECT_TRUE(selector.setpoint().valid);
		EXPECT_EQ(selector.setpoint().timestamp_sample, timestamp);
		EXPECT_EQ(selector.instance(), 1);
		EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);

		timestamp += 100_ms;
		inputs[1].timestamp_sample = timestamp;
	}
}



TEST(ManualControlSelector, RcInputOnly)
{
	ManualControlSelector selector;
	selector.setRcInMode(0); // RC only
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// First: only MAVLink input — should be ignored
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_FALSE(selector.setpoint().valid);

	// Second: RC input becomes available
	timestamp += 100_ms;
	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 1);
}


TEST(ManualControlSelector, MavlinkInputOnly)
{
	ManualControlSelector selector;
	selector.setRcInMode(1); // MAVLink only
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// First: RC input (wrong source)
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_FALSE(selector.setpoint().valid);

	// Second: valid MAVLink input (MAVLINK_3)
	timestamp += 100_ms;
	inputs[1].data_source = SOURCE_MAVLINK_3;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_3);
	EXPECT_EQ(selector.instance(), 1);

	// Third: different MAVLink source — ignored (MAVLINK_3 is still active)
	timestamp += 100_ms;
	inputs[1].data_source = SOURCE_MAVLINK_4;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_3);
	EXPECT_EQ(selector.instance(), 1);
}


TEST(ManualControlSelector, RcMavlinkInputFallback)
{
	ManualControlSelector selector;
	selector.setRcInMode(2); // Fallback mode
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// First: valid RC input
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// Second: valid MAVLink input, RC still valid — RC remains active
	timestamp += 100_ms;
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// Third: RC times out — MAVLink takes over
	timestamp += 500_ms;
	inputs[1].timestamp_sample = timestamp;

	// RC no update, so input[0].timestamp_sample remains outdated
	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// Fourth: RC comes back, but fallback does not switch back
	inputs[1].data_source = SOURCE_RC;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);
}


TEST(ManualControlSelector, RcMavlinkInputKeepFirst)
{
	ManualControlSelector selector;
	selector.setRcInMode(3); // Keep first valid input
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// RC is first valid input
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// MAVLink input arrives, should be ignored due to mode 3
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// RC input times out
	timestamp += 500_ms;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
	EXPECT_NE(selector.setpoint().data_source, SOURCE_MAVLINK_0);

	// RC returns — should be accepted again
	timestamp += 100_ms;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);
}

TEST(ManualControlSelector, DisabledInput)
{
	ManualControlSelector selector;
	selector.setRcInMode(4); // Stick input disabled
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// MAVLink stick input should be ignored
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);

	// RC stick input should also be ignored
	timestamp += 100_ms;
	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}


TEST(ManualControlSelector, RcTimeout)
{
	ManualControlSelector selector;
	selector.setRcInMode(0); // RC only
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// Valid RC input
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// Advance time past timeout
	timestamp += 600_ms;

	// Manually call updateValidityOfChosenInput to simulate timeout
	selector.updateValidityOfChosenInput(timestamp);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}

TEST(ManualControlSelector, RcOutdated)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp - 600_ms;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);

	// Try again with same outdated timestamp
	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}

TEST(ManualControlSelector, RcMavlinkInputRcPriority)
{
	ManualControlSelector selector;
	selector.setRcInMode(5); // RC priority
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// RC input comes first
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// MAVLink input arrives
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// RC times out
	timestamp += 500_ms;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// RC returns
	inputs[0].timestamp_sample = timestamp;
	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);
}

TEST(ManualControlSelector, RcMavlinkInputMavlinkPriority)
{
	ManualControlSelector selector;
	selector.setRcInMode(6); // MAVLink priority
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// MAVLink input comes first
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// RC input arrives
	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	// MAVLink times out, RC is now valid
	timestamp += 500_ms;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 1);

	// MAVLink returns
	inputs[0].timestamp_sample = timestamp;
	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);
}

TEST(ManualControlSelector, MavlinkTwoInstanceInputMavlinkPriority)
{
	ManualControlSelector selector;
	selector.setRcInMode(6); // MAVLink priority
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// First MAVLink input
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	// Second MAVLink arrives
	timestamp += 100_ms;
	inputs[1].data_source = SOURCE_MAVLINK_1;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	// MAVLink 0 times out
	timestamp += 500_ms;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_1);
	EXPECT_EQ(selector.instance(), 1);

	// MAVLink 0 returns, should not override
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_1);
	EXPECT_EQ(selector.instance(), 1);
}


TEST(ManualControlSelector, InputTimeoutDoesNotPreventFallbackToStillValidInput)
{
	ManualControlSelector selector;
	selector.setRcInMode(2); // Fallback mode
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// MAVLink comes first
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// Near timeout
	timestamp += 499_ms;

	// RC arrives now
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	// Still MAVLink
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);

	// Now MAVLink is outdated
	timestamp += 1_ms;
	inputs[0].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);
}
TEST(ManualControlSelector, AllInputsTimeoutSimultaneously)
{
	ManualControlSelector selector;
	selector.setRcInMode(2); // Fallback
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// Both inputs valid
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);

	// Timeout both
	timestamp += 501_ms;

	selector.updateWithNewInputSamples(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}

TEST(ManualControlSelector, ThreeInputsFallbackOrder)
{
	ManualControlSelector selector;
	selector.setRcInMode(2); // Fallback mode (RC and Joystick)
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[3] {};

	// Input 0: RC, valid
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	// Input 1: MAVLINK_0, valid
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	// Input 2: MAVLINK_1, valid
	inputs[2].data_source = SOURCE_MAVLINK_1;
	inputs[2].valid = true;
	inputs[2].timestamp_sample = timestamp;

	// Should select RC (input 0)
	selector.updateWithNewInputSamples(timestamp, inputs, 3);
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// RC times out, only MAVLINK_0 and MAVLINK_1 are valid
	timestamp += 600_ms;
	inputs[0].timestamp_sample = timestamp - 600_ms; // Outdated
	inputs[1].timestamp_sample = timestamp;
	inputs[2].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 3);
	// Should select MAVLINK_0 (input 1, first valid in order)
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// MAVLINK_0 times out, only MAVLINK_1 is valid
	timestamp += 600_ms;
	inputs[1].timestamp_sample = timestamp - 600_ms; // Outdated
	inputs[2].timestamp_sample = timestamp;

	selector.updateWithNewInputSamples(timestamp, inputs, 3);
	// Should select MAVLINK_1 (input 2)
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_1);
	EXPECT_EQ(selector.instance(), 2);

	// All inputs timeout
	timestamp += 600_ms;
	inputs[2].timestamp_sample = timestamp - 600_ms;

	selector.updateWithNewInputSamples(timestamp, inputs, 3);
	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}
