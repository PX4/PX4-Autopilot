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
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	constexpr uint8_t kInstanceIndex = 1;
	constexpr uint8_t kExpectedDataSource = SOURCE_RC;

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {}; // inputs[0] remains default

	// Configure inputs[1] with SOURCE_RC, initially invalid
	inputs[kInstanceIndex].data_source = kExpectedDataSource;
	inputs[kInstanceIndex].valid = false;
	inputs[kInstanceIndex].timestamp_sample = timestamp;

	// Run two iterations with invalid input
	for (int i = 0; i < 2; i++) {
		selector.processInputSample(timestamp, inputs[kInstanceIndex], kInstanceIndex);
		selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	selector.processInputSample(timestamp, inputs[kInstanceIndex], kInstanceIndex);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().timestamp_sample, timestamp);
	EXPECT_EQ(selector.instance(), kInstanceIndex);
	EXPECT_EQ(selector.setpoint().data_source, kExpectedDataSource);
}

TEST(ManualControlSelector, RcInputContinuous)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {}; // Only using index 1

	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	for (int i = 0; i < 5; i++) {
		selector.processInputSample(timestamp, inputs[1], 1);
		selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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
	selector.setRcInMode(0); // RC input only
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// Provide valid MAVLink input (wrong source)
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);

	timestamp += 100_ms;

	// Provide valid RC input (correct source)
	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 1);
}

TEST(ManualControlSelector, MavlinkInputOnly)
{
	ManualControlSelector selector;
	selector.setRcInMode(1); // MAVLink input only
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// Provide valid RC input (ignored)
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);

	timestamp += 100_ms;

	// Provide valid MAVLink input (first accepted)
	inputs[1].data_source = SOURCE_MAVLINK_3;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_3);
	EXPECT_EQ(selector.instance(), 1);

	timestamp += 100_ms;

	// Provide later MAVLink input from different source (ignored)
	inputs[1].data_source = SOURCE_MAVLINK_4;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	// Valid RC input
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// MAVLink input should be ignored (RC still valid)
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// RC times out, MAVLink becomes active
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// RC comes back immediately, but fallback keeps MAVLink
	inputs[1].data_source = SOURCE_RC;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);
}

TEST(ManualControlSelector, RcMavlinkInputKeepFirst)
{
	ManualControlSelector selector;
	selector.setRcInMode(3); // Keep first input mode
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	// First valid input: RC
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// MAVLink input arrives, should be ignored
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// RC times out, but system should NOT switch to MAVLink
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_NE(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), -1);

	timestamp += 100_ms;

	// RC returns, and is accepted again since it was the first
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	// Reject MAVLink stick input
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);

	timestamp += 100_ms;

	// Reject RC stick input
	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}

TEST(ManualControlSelector, RcTimeout)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = SOME_TIME;
	manual_control_setpoint_s inputs[2] {};

	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	// Advance time beyond timeout
	timestamp += 600_ms;

	// Let selector update internal validity
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

	// Input already outdated
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp - 600_ms;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);

	// Try again with same outdated timestamp
	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	// Valid RC input comes first
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// MAVLink input ignored due to RC priority
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// RC times out, now switch to MAVLink
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// RC returns — should now take priority again
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	// First valid MAVLink input is selected
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// RC input arrives, but should be ignored due to MAVLink priority
	inputs[1].data_source = SOURCE_RC;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// MAVLink times out, RC becomes valid
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_RC);
	EXPECT_EQ(selector.instance(), 1);

	// MAVLink returns, and should take over again
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	// MAVLink 0 input is initially used
	inputs[0].data_source = SOURCE_MAVLINK_0;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// MAVLink 1 input arrives: ignored for now
	inputs[1].data_source = SOURCE_MAVLINK_1;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// MAVLink 0 times out, MAVLink 1 is now used
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_1);
	EXPECT_EQ(selector.instance(), 1);

	// MAVLink 0 returns, but priority is equal: it should *not* switch back
	inputs[0].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

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

	// Step 1: MAVLink input comes first and is selected
	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// Step 2: Advance time near the timeout threshold
	timestamp += 499_ms;

	// RC input arrives now and is valid
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	// MAVLink is still considered valid (just under timeout)
	inputs[1].timestamp_sample = SOME_TIME; // Still the old/stale timestamp

	selector.processInputSample(timestamp, inputs[0], 0);  // RC
	selector.processInputSample(timestamp, inputs[1], 1);  // MAVLink (stale)

	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	// At this point, MAVLink is still active
	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_EQ(selector.setpoint().data_source, SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);

	// Step 3: Advance time by 1ms: MAVLink should now timeout
	timestamp += 1_ms;

	// Do not update MAVLink: it remains stale
	inputs[0].timestamp_sample = timestamp; // RC is still valid

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	// RC input should now be selected as fallback
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

	// Both inputs valid at t0
	inputs[0].data_source = SOURCE_RC;
	inputs[0].valid = true;
	inputs[0].timestamp_sample = timestamp;

	inputs[1].data_source = SOURCE_MAVLINK_0;
	inputs[1].valid = true;
	inputs[1].timestamp_sample = timestamp;

	selector.processInputSample(timestamp, inputs[0], 0);
	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_TRUE(selector.setpoint().valid);

	// Advance time so both inputs are stale
	timestamp += 501_ms;

	// Do not update inputs: they stay old
	selector.processInputSample(timestamp, inputs[0], 0);
	selector.processInputSample(timestamp, inputs[1], 1);
	selector.evaluateAndSetBestInput(timestamp, inputs, 2);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}
